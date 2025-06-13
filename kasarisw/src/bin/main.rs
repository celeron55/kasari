#![no_std]
#![no_main]

use core::cell::RefCell;
use core::ops::DerefMut;
use critical_section::Mutex;
use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, blocking_mutex::raw::NoopRawMutex, signal::Signal,
};
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    gpio::{AnyPin, Level, Output, OutputConfig, Pin},
    ledc::{
        channel as ledc_channel, channel::ChannelIFace, timer as ledc_timer, timer::TimerIFace,
        Ledc, channel::ChannelHW
    },
    rmt::{PulseCode, Rmt, RxChannelAsync, RxChannelConfig, RxChannelCreatorAsync},
    spi::master::Spi,
    time::Rate,
    timer::timg::TimerGroup,
    uart,
    uart::{Uart, UartRx, UartTx},
    Async,
};
use esp_println::println;
use ringbuffer::{ConstGenericRingBuffer, RingBuffer};
use static_cell::StaticCell; // For allocator

// LIDAR constants
const PACKET_SIZE: usize = 22;
const HEAD_BYTE: u8 = 0xFA;
const READ_BUF_SIZE: usize = 64;

// Global event queue to be fed to main logic
static GLOBAL_EVENT_QUEUE: Mutex<RefCell<Option<ConstGenericRingBuffer<kasari::InputEvent, 300>>>> =
    Mutex::new(RefCell::new(None));

#[embassy_executor::task]
async fn lidar_writer(
    mut tx: UartTx<'static, Async>,
    signal: &'static Signal<NoopRawMutex, usize>,
) {
    use core::fmt::Write;
    embedded_io_async::Write::write(&mut tx, b"Hello async serial\r\n")
        .await
        .unwrap();
    embedded_io_async::Write::flush(&mut tx).await.unwrap();
    loop {
        let bytes_read = signal.wait().await;
        signal.reset();
        write!(&mut tx, "\r\n-- received {} bytes --\r\n", bytes_read).unwrap();
        embedded_io_async::Write::flush(&mut tx).await.unwrap();
    }
}

#[embassy_executor::task]
async fn lidar_reader(
    mut rx: UartRx<'static, Async>,
    signal: &'static Signal<NoopRawMutex, usize>,
) {
    const MAX_BUFFER_SIZE: usize = 2 * READ_BUF_SIZE + 16;
    let mut rbuf: [u8; MAX_BUFFER_SIZE] = [0u8; MAX_BUFFER_SIZE];
    let mut offset = 0;
    let mut ring_buf: ConstGenericRingBuffer<u8, MAX_BUFFER_SIZE> = ConstGenericRingBuffer::new();
    let mut log_i: u32 = 0;

    loop {
        // Original reading method
        let r = embedded_io_async::Read::read(&mut rx, &mut rbuf[offset..]).await;
        match r {
            Ok(len) => {
                offset += len;
                // Push read data into ring buffer
                for &byte in &rbuf[..offset] {
                    ring_buf.push(byte);
                }
                //println!("Read: {len}, data: {:?}", &rbuf[..offset]);
                offset = 0;

                // Process complete packets
                while ring_buf.len() >= PACKET_SIZE {
                    // Check for packet header
                    if ring_buf.get(0) != Some(&HEAD_BYTE) {
                        // Skip until HEAD_BYTE
                        while let Some(byte) = ring_buf.get(0) {
                            if *byte == HEAD_BYTE {
                                break;
                            }
                            ring_buf.dequeue();
                        }
                        continue;
                    }

                    // Extract 22-byte packet
                    let mut packet = [0u8; PACKET_SIZE];
                    for i in 0..PACKET_SIZE {
                        packet[i] = ring_buf.get(i).copied().unwrap_or(0);
                    }

                    // Parse, queue and print packet
                    if let Some(parsed) = parse_packet(&packet) {
                        critical_section::with(|cs| {
                            if let Some(ref mut event_queue) =
                                GLOBAL_EVENT_QUEUE.borrow(cs).borrow_mut().deref_mut()
                            {
                                let timestamp = embassy_time::Instant::now().as_ticks();
                                let event = kasari::InputEvent::Lidar(
                                    timestamp,
                                    parsed.distances[0] as f32,
                                    parsed.distances[1] as f32,
                                    parsed.distances[2] as f32,
                                    parsed.distances[3] as f32,
                                );
                                event_queue.push(event);
                            }
                        });

                        log_i += 1;
                        if log_i % 20 == 1 {
                            println!(
                                "Packet {}: distances=[{}, {}, {}, {}] mm",
                                log_i,
                                parsed.distances[0],
                                parsed.distances[1],
                                parsed.distances[2],
                                parsed.distances[3]
                            );
                        }
                    } else {
                        println!("Invalid packet");
                    }

                    // Remove processed packet
                    for _ in 0..PACKET_SIZE {
                        ring_buf.dequeue();
                    }
                    signal.signal(PACKET_SIZE);
                }
            }
            Err(e) => println!("RX Error: {:?}", e),
        }
    }
}

// Parsed packet data
struct ParsedPacket {
    distances: [u16; 4], // mm
}

// Parse a 22-byte packet
fn parse_packet(packet: &[u8]) -> Option<ParsedPacket> {
    if packet.len() != PACKET_SIZE || packet[0] != HEAD_BYTE {
        return None;
    }

    // Angle: byte 1, (ANGLE - 0xA0) * 4
    let angle = ((packet[1] as u16).wrapping_sub(0xA0)) * 4;
    if angle > 360 {
        return None;
    }

    // Speed: bytes 2 (LSB), 3 (MSB), (MSB << 8) | LSB / 64
    //let speed = ((packet[3] as u16) << 8) | (packet[2] as u16);
    //let speed = speed / 64;

    // Distances: bytes 4–19, 4 sets of (LSB, MSB & 0x3F)
    let mut distances = [0u16; 4];
    for (i, dist) in distances.iter_mut().enumerate() {
        let idx = 4 + i * 4;
        *dist = ((packet[idx + 1] as u16 & 0x3F) << 8) | (packet[idx] as u16);
    }

    // Checksum: bytes 20 (LSB), 21 (MSB)
    let received_sum = ((packet[21] as u16) << 8) | (packet[20] as u16);
    let computed_sum = compute_checksum(&packet[..PACKET_SIZE - 2]);
    if computed_sum != received_sum {
        return None;
    }

    Some(ParsedPacket { distances })
}

// Compute checksum for packet (excluding last 2 bytes)
fn compute_checksum(data: &[u8]) -> u16 {
    let mut chk32: u32 = 0;
    for chunk in data.chunks(2) {
        let word = if chunk.len() == 2 {
            ((chunk[1] as u16) << 8) + (chunk[0] as u16)
        } else {
            chunk[0] as u16
        };
        chk32 = (chk32 << 1) + (word as u32);
    }
    let checksum = (chk32 & 0x7FFF) + (chk32 >> 15);
    (checksum & 0x7FFF) as u16
}

#[embassy_executor::task]
async fn encoder_emulation_task(encoder_emulation_output_pinmap: AnyPin) {
    let mut encoder_emulation_output_pin = Output::new(
        encoder_emulation_output_pinmap,
        Level::High,
        OutputConfig::default(),
    );

    loop {
        // 10+10ms seems to be the lowest we can go, resulting in reported
        // 198 RPM and thus about 397 reports per second, each containing 4
        // measurements, hopefully measured at a consistent 1188/s interval
        encoder_emulation_output_pin.set_low();
        Timer::after(Duration::from_millis(10)).await;
        encoder_emulation_output_pin.set_high();
        Timer::after(Duration::from_millis(10)).await;
    }
}

#[embassy_executor::task]
async fn accelerometer_task(mut spi: Spi<'static, esp_hal::Blocking>) {
    // Read DEVID_AD
    let mut buf = [(0x00 << 1) | 1, 0]; // DEVID_AD (should read 0xad)
    spi.transfer(&mut buf).unwrap(); // TODO: Error handling
    println!("DEVID_AD = {:#02x} (should be 0xad)", buf[1]);

    // Turn on full bandwidth measurement mode, enable lpf, disable hpf
    let mut buf = [(0x3f << 1) | 0, 0b00010111]; // POWER_CTL
    spi.transfer(&mut buf).unwrap(); // TODO: Error handling

    // Read XDATA...ZDATA
    //let mut loop_i = 0;
    loop {
        let mut buf = [(0x08 << 1) | 1, 0, 0, 0, 0, 0, 0]; // XDATA
        spi.transfer(&mut buf).unwrap(); // TODO: Error handling
        let x_raw = (((buf[1] as i16) << 8) | buf[2] as i16) >> 4;
        let y_raw = (((buf[3] as i16) << 8) | buf[4] as i16) >> 4;
        let z_raw = (((buf[5] as i16) << 8) | buf[6] as i16) >> 4;
        let x_g = x_raw as f32 * 0.2;
        let y_g = y_raw as f32 * 0.2;
        let z_g = z_raw as f32 * 0.2;

        // Produce an event to the global event queue
        critical_section::with(|cs| {
            if let Some(ref mut event_queue) =
                GLOBAL_EVENT_QUEUE.borrow(cs).borrow_mut().deref_mut()
            {
                let timestamp = embassy_time::Instant::now().as_ticks();
                let event = kasari::InputEvent::Accelerometer(
                    timestamp, -y_g, // Positive proportional value when the robot spins
                );
                event_queue.push(event);
            }
        });

        // We should be getting negative Y in the spinning robot
        /*if loop_i % 100 == 0 {
            println!("x ={: >5.1}, y ={: >5.1}, z ={: >5.1}", x_g, y_g, z_g);
        }
        loop_i += 1;*/
        Timer::after_millis(10).await;
    }
}

#[embassy_executor::task]
async fn rmt_task(mut ch0: esp_hal::rmt::Channel<esp_hal::Async, 0>) {
    let mut buffer0: [u32; 1] = [PulseCode::empty(); 1];

    loop {
        for x in buffer0.iter_mut() {
            x.reset();
        }
        let i = 0;
        let result = ch0.receive(&mut buffer0).await;
        let mut ch0_final_result: Option<f32> = None;
        //esp_println::println!("RMT Results:");
        match result {
            Ok(()) => {
                for entry in buffer0.iter().take_while(|e| e.length1() != 0) {
                    /*esp_println::println!("{:?}, {:?}, {:?}, {:?}",
                    entry.level1(), entry.length1(),
                    entry.level2(), entry.length2());*/

                    let length_raw = if entry.level1() == Level::High && entry.length1() > 10 {
                        entry.length1()
                    } else if entry.level2() == Level::High && entry.length2() > 10 {
                        entry.length2()
                    } else {
                        0
                    };

                    if length_raw > 10 {
                        //esp_println::println!("* {:?}", length_raw);
                        let pulse_width_us = length_raw as f32;
                        ch0_final_result = Some(pulse_width_us);
                    }
                }
            }
            Err(_err) => {
                //esp_println::println!("Channel {} error: {:?}", i, err);
            }
        }
        if ch0_final_result.is_none() {
            //esp_println::println!("Channel {} no valid result", i);
        }

        // Produce an event to the global event queue
        critical_section::with(|cs| {
            if let Some(ref mut event_queue) =
                GLOBAL_EVENT_QUEUE.borrow(cs).borrow_mut().deref_mut()
            {
                let timestamp = embassy_time::Instant::now().as_ticks();
                let event = kasari::InputEvent::Receiver(timestamp, 0, ch0_final_result);
                event_queue.push(event);
            }
        });
    }
}

mod kasari {
    pub enum InputEvent {
        Lidar(u64, f32, f32, f32, f32), // timestamp, distance samples (mm)
        Accelerometer(u64, f32),        // timestamp, acceleration (G)
        Receiver(u64, u8, Option<f32>), // timestamp, channel (0=throttle), pulse length (us)
    }

    pub struct MotorControlPlan {
        throttle: f32,             // 0.0...1.0
        modulation_amplitude: f32, // -0.0....1.0
        modulation_phase: f32,     // Some kind of an angle
    }

    pub struct MainLogic {
        pub motor_control_plan: Option<MotorControlPlan>,

        acceleration: f32, // Acceleration to the normal direction re. rotation
    }

    impl MainLogic {
        pub fn new() -> Self {
            Self {
                motor_control_plan: None,
                acceleration: 0.0,
            }
        }

        pub fn feed_event(&mut self, event: InputEvent) {
            match event {
                InputEvent::Lidar(timestamp, d1, d2, d3, d4) => {
                    //esp_println::println!("Lidar event");
                }
                InputEvent::Accelerometer(timestamp, acceleration) => {
                    //esp_println::println!("Accelerometer event");
                    self.acceleration = acceleration;
                }
                InputEvent::Receiver(timestamp, ch, pulse_length) => {
                    esp_println::println!("Receiver event: ch{:?}: {:?}", ch, pulse_length);
                }
            }
        }

        pub fn step(&mut self) {}
    }
}

const PWM_HZ: f32 = 400.0;

// ESC target speed (bidirectional) to servo signal PWM duty cycle percent
fn target_speed_to_pwm_duty(speed_percent: f32, duty_range: u32) -> u32 {
    let center_pwm = 0.00149 * PWM_HZ;
    let pwm_amplitude = 0.000350 * PWM_HZ;
    let duty_percent = (center_pwm * 100.0 + pwm_amplitude * speed_percent)
        .min(100.0)
        .max(-100.0);
    (duty_range * duty_percent as u32) / 100
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::println!("Init!");
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    // GPIO

    let encoder_emulation_output_pinmap = peripherals.GPIO13.degrade();
    let rc_receiver_ch1_pinmap = peripherals.GPIO34;

    // PWM output to ESCs

    let esc_right_pin = Output::new(peripherals.GPIO26, Level::Low, OutputConfig::default());
    let esc_left_pin = Output::new(peripherals.GPIO27, Level::Low, OutputConfig::default());

    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(esp_hal::ledc::LSGlobalClkSource::APBClk);

    // Configure PWM timer
    let mut lstimer0 = ledc.timer::<esp_hal::ledc::LowSpeed>(ledc_timer::Number::Timer0);
    lstimer0
        .configure(ledc_timer::config::Config {
            duty: ledc_timer::config::Duty::Duty8Bit,
            clock_source: ledc_timer::LSClockSource::APBClk,
            frequency: Rate::from_hz(PWM_HZ as u32),
        })
        .unwrap();
    let mut lstimer1 = ledc.timer::<esp_hal::ledc::LowSpeed>(ledc_timer::Number::Timer1);
    lstimer1
        .configure(ledc_timer::config::Config {
            duty: ledc_timer::config::Duty::Duty8Bit,
            clock_source: ledc_timer::LSClockSource::APBClk,
            frequency: Rate::from_hz(PWM_HZ as u32),
        })
        .unwrap();

    // Configure PWM channels for GPIO26 and GPIO27
    let mut channel0: esp_hal::ledc::channel::Channel<'_, esp_hal::ledc::LowSpeed> =
        ledc.channel(ledc_channel::Number::Channel0, esc_right_pin);
    channel0
        .configure(ledc_channel::config::Config {
            timer: &lstimer0,
            duty_pct: 10,
            pin_config: ledc_channel::config::PinConfig::PushPull,
        })
        .unwrap();

    let mut channel1: esp_hal::ledc::channel::Channel<'_, esp_hal::ledc::LowSpeed> =
        ledc.channel(ledc_channel::Number::Channel1, esc_left_pin);
    channel1
        .configure(ledc_channel::config::Config {
            timer: &lstimer1,
            duty_pct: 10,
            pin_config: ledc_channel::config::PinConfig::PushPull,
        })
        .unwrap();

    // Set initial PWM output to 0 (=disable)
    channel0.set_duty(0);
    channel1.set_duty(0);

    // UART2

    // Default pins for Uart communication
    let (tx_pin, rx_pin) = (peripherals.GPIO17, peripherals.GPIO16);

    let config = uart::Config::default()
        .with_rx(uart::RxConfig::default().with_fifo_full_threshold(READ_BUF_SIZE as u16))
        .with_baudrate(115200);

    let uart2 = Uart::new(peripherals.UART2, config)
        .unwrap()
        .with_tx(tx_pin)
        .with_rx(rx_pin)
        .into_async();

    let (rx, tx) = uart2.split();

    // SPI (ADXL373)

    let sclk = peripherals.GPIO18;
    let miso = peripherals.GPIO19;
    let mosi = peripherals.GPIO23;
    let cs = peripherals.GPIO5;

    let spi = Spi::new(
        peripherals.SPI2, // Is this the right one?
        esp_hal::spi::master::Config::default().with_frequency(Rate::from_khz(100)),
    )
    .unwrap()
    .with_sck(sclk)
    .with_mosi(mosi)
    .with_miso(miso)
    .with_cs(cs);

    // RC receiver PWM input

    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80)).unwrap();
    let rmt = rmt.into_async();

    let rx_config = RxChannelConfig::default()
        .with_clk_divider(80)
        .with_idle_threshold(3000)
        .with_filter_threshold(100)
        .with_carrier_modulation(false);

    let rmt_ch0 = rmt
        .channel0
        .configure(rc_receiver_ch1_pinmap, rx_config.clone())
        .unwrap();
    //let mut rmt_ch1 = rmt.channel1.configure(gpio35, rx_config.clone()).unwrap();
    //let mut rmt_ch2 = rmt.channel2.configure(gpio36, rx_config).unwrap();

    // Spawn tasks

    static SIGNAL: StaticCell<Signal<NoopRawMutex, usize>> = StaticCell::new();
    let signal = &*SIGNAL.init(Signal::new());

    spawner.spawn(lidar_reader(rx, &signal)).ok();
    spawner.spawn(lidar_writer(tx, &signal)).ok();
    spawner
        .spawn(encoder_emulation_task(encoder_emulation_output_pinmap))
        .ok();
    spawner.spawn(accelerometer_task(spi)).ok();
    spawner.spawn(rmt_task(rmt_ch0)).ok();

    // Main loop

    let mut logic = kasari::MainLogic::new();

    loop {
        //esp_println::println!("Main loop");

        // Get the event queue, swapping an empty one in place
        let event_queue = critical_section::with(|cs| {
            GLOBAL_EVENT_QUEUE
                .borrow(cs)
                .replace(Some(ConstGenericRingBuffer::new()))
        });

        // Feed the entire event queue one at a time
        if let Some(mut event_queue) = event_queue {
            while let Some(event) = event_queue.dequeue() {
                // TODO: Remove: Direct ESC control
                if let kasari::InputEvent::Receiver(timestamp, ch, pulse_length) = event {
                    //esp_println::println!("Receiver event (main loop): ch{:?}: {:?}", ch, pulse_length);
                    match pulse_length {
                        Some(pulse_length) => {
                            let target_speed_percent = (pulse_length - 1500.0) * 0.2;
                            esp_println::println!(
                                "pulse_length -> target_speed_percent: {:?} -> {:?}",
                                pulse_length,
                                target_speed_percent
                            );
                            let mut duty = target_speed_to_pwm_duty(target_speed_percent, 2u32.pow(8));
                            esp_println::println!("Setting duty cycle: {:?}", duty);
                            // Right motor; positive speed is downwards, causing
                            // clockwise robot rotation
                            channel0.set_duty_hw(duty);
                            // Left motor; positive speed is downwards, causing
                            // counter-clockwise robot rotation (not verified in
                            // hardware)
                            channel1.set_duty_hw(duty);
                        }
                        None => {
                            channel0.set_duty(0);
                            channel1.set_duty(0);
                        }
                    }
                }

                logic.feed_event(event);
            }
        }

        logic.step();

        // TODO: Implement motor control in a task and send logic.motor_control_plan there

        Timer::after_millis(20).await;
    }
}
