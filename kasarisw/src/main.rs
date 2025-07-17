#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    analog::{adc, adc::Adc, adc::AdcConfig},
    gpio::{AnyPin, Level, Output, OutputConfig, Pin},
    ledc::{
        channel as ledc_channel, channel::ChannelHW, timer as ledc_timer, Ledc,
    },
    rmt::{RxChannelAsync, RxChannelConfig, RxChannelCreatorAsync},
    spi::master::Spi,
    time::Rate,
    timer::timg::TimerGroup,
    uart::{Uart, RxConfig, UartInterrupt},
    Async,
    Blocking,
};
use esp_hal::interrupt;
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::ledc::channel::ChannelIFace;
use esp_println::{print, println};
use static_cell::StaticCell;
use ringbuffer::ConstGenericRingBuffer;
use core::net::Ipv4Addr;
use embassy_net::{
    IpListenEndpoint,
    Ipv4Cidr,
    Runner,
    StackResources,
    StaticConfigV4,
    tcp::TcpSocket,
};
use esp_wifi::{
    EspWifiController,
    init,
    wifi::{
        AccessPointConfiguration,
        ClientConfiguration,
        Configuration,
        WifiController,
        WifiEvent,
        WifiState,
    },
};
use embassy_sync::pubsub::PubSubChannel;
use esp_wifi::wifi::WifiDevice;
use embassy_sync::signal::Signal;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
extern crate alloc;
use core::cell::RefCell;
use critical_section::Mutex;
use alloc::collections::VecDeque;
use ringbuffer::RingBuffer;
use core::sync::atomic::{AtomicU32, Ordering};

mod sensors;
mod shared;

use shared::kasari;

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

const PWM_HZ: f32 = 400.0;

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

// ESC target speed (bidirectional) to servo signal PWM duty cycle percent
fn target_speed_to_pwm_duty(speed_percent: f32, duty_range: u32) -> u32 {
    let center_pwm = 0.00149 * PWM_HZ;
    let pwm_amplitude = 0.000350 * PWM_HZ;
    let duty_percent = (center_pwm * 100.0 + pwm_amplitude * speed_percent)
        .min(100.0)
        .max(-100.0);
    (duty_range * duty_percent as u32) / 100
}

const LIDAR_BUFFER_SIZE: usize = sensors::PACKET_SIZE * 4;

static UART2: StaticCell<Mutex<RefCell<Option<Uart<'static, Blocking>>>>> = StaticCell::new();
static LIDAR_BYTE_BUFFER: StaticCell<Mutex<RefCell<ConstGenericRingBuffer<u8, LIDAR_BUFFER_SIZE>>>> = StaticCell::new();
static LIDAR_EVENT_QUEUE: StaticCell<Mutex<RefCell<VecDeque<kasari::InputEvent>>>> = StaticCell::new();
static SIGNAL_LIDAR: StaticCell<Signal<NoopRawMutex, ()>> = StaticCell::new();

static mut UART2_REF: Option<&'static Mutex<RefCell<Option<Uart<'static, Blocking>>>>> = None;
static mut LIDAR_BYTE_BUFFER_REF: Option<&'static Mutex<RefCell<ConstGenericRingBuffer<u8, LIDAR_BUFFER_SIZE>>>> = None;
static mut LIDAR_EVENT_QUEUE_REF: Option<&'static Mutex<RefCell<VecDeque<kasari::InputEvent>>>> = None;
static mut SIGNAL_LIDAR_REF: Option<&'static Signal<NoopRawMutex, ()>> = None;

static LIDAR_PACKET_COUNT: AtomicU32 = AtomicU32::new(0);

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    esp_println::println!("Init!");
    let peripherals = esp_hal::init(esp_hal::Config::default());

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let mut rng = esp_hal::rng::Rng::new(peripherals.RNG);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let timg1 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timg1.timer0);

    // This has to be initialized before any tasks are started
    let event_channel = &*shared::EVENT_CHANNEL.init(PubSubChannel::new());

    // GPIO
    let rc_receiver_ch1_pinmap = peripherals.GPIO34;

    // ADC (battery voltage monitoring)
    let mut adc1_config = AdcConfig::new();
    let mut vbat_pin = adc1_config.enable_pin(peripherals.GPIO39, adc::Attenuation::_11dB);
    let mut adc1 = Adc::new(peripherals.ADC1, adc1_config);

    // PWM output to ESCs (LEDC channels 0 and 1)
    let esc_right_pin = Output::new(peripherals.GPIO26, Level::Low, OutputConfig::default());
    let esc_left_pin = Output::new(peripherals.GPIO27, Level::Low, OutputConfig::default());

    let mut ledc = Ledc::new(peripherals.LEDC);
    ledc.set_global_slow_clock(esp_hal::ledc::LSGlobalClkSource::APBClk);

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

    let mut channel0 = ledc.channel(ledc_channel::Number::Channel0, esc_right_pin);
    channel0
        .configure(ledc_channel::config::Config {
            timer: &lstimer0,
            duty_pct: 10,
            pin_config: ledc_channel::config::PinConfig::PushPull,
        })
        .unwrap();
    let mut channel1 = ledc.channel(ledc_channel::Number::Channel1, esc_left_pin);
    channel1
        .configure(ledc_channel::config::Config {
            timer: &lstimer1,
            duty_pct: 10,
            pin_config: ledc_channel::config::PinConfig::PushPull,
        })
        .unwrap();

    _ = channel0.set_duty(0);
    _ = channel1.set_duty(0);

    // 50Hz square wave to LIDAR encoder input (LEDC channel 2)
    let lidar_encoder_output_pin = Output::new(peripherals.GPIO13, Level::Low, OutputConfig::default());

    let mut lstimer2 = ledc.timer::<esp_hal::ledc::LowSpeed>(ledc_timer::Number::Timer2);
    lstimer2
        .configure(ledc_timer::config::Config {
            duty: ledc_timer::config::Duty::Duty8Bit,
            clock_source: ledc_timer::LSClockSource::APBClk,
            frequency: Rate::from_hz(50.0 as u32),
        })
        .unwrap();

    let mut channel2 = ledc.channel(ledc_channel::Number::Channel2, lidar_encoder_output_pin);
    channel2
        .configure(ledc_channel::config::Config {
            timer: &lstimer2,
            duty_pct: 50,
            pin_config: ledc_channel::config::PinConfig::PushPull,
        })
        .unwrap();

    // UART2
    let (tx_pin, rx_pin) = (peripherals.GPIO17, peripherals.GPIO16);
    let config = esp_hal::uart::Config::default()
        .with_rx(esp_hal::uart::RxConfig::default().with_fifo_full_threshold(sensors::PACKET_SIZE as u16))
        .with_baudrate(115200);
    let mut uart2 = Uart::new(peripherals.UART2, config)
        .unwrap()
        .with_tx(tx_pin)
        .with_rx(rx_pin);

    uart2.set_interrupt_handler(uart2_handler);
    uart2.listen(UartInterrupt::RxFifoFull);

    let uart_static = UART2.init(Mutex::new(RefCell::new(Some(uart2))));
    unsafe { UART2_REF = Some(uart_static); }
    let byte_buffer = LIDAR_BYTE_BUFFER.init(Mutex::new(RefCell::new(ConstGenericRingBuffer::new())));
    unsafe { LIDAR_BYTE_BUFFER_REF = Some(byte_buffer); }
    let lidar_event_queue = LIDAR_EVENT_QUEUE.init(Mutex::new(RefCell::new(VecDeque::new())));
    unsafe { LIDAR_EVENT_QUEUE_REF = Some(lidar_event_queue); }
    let signal_lidar = SIGNAL_LIDAR.init(Signal::new());
    unsafe { SIGNAL_LIDAR_REF = Some(signal_lidar); }

    // SPI (ADXL373)
    let sclk = peripherals.GPIO18;
    let miso = peripherals.GPIO19;
    let mosi = peripherals.GPIO23;
    let cs = peripherals.GPIO5;
    let spi = Spi::new(
        peripherals.SPI2,
        esp_hal::spi::master::Config::default().with_frequency(Rate::from_khz(100)),
    )
    .unwrap()
    .with_sck(sclk)
    .with_mosi(mosi)
    .with_miso(miso)
    .with_cs(cs);

    // RC receiver PWM input
    let rmt = esp_hal::rmt::Rmt::new(peripherals.RMT, Rate::from_mhz(80)).unwrap();
    let rmt = rmt.into_async();
    let rx_config = RxChannelConfig::default()
        .with_clk_divider(80)
        .with_idle_threshold(3000)
        .with_filter_threshold(100)
        .with_carrier_modulation(false);
    let rmt_ch0 = rmt
        .channel0
        .configure(rc_receiver_ch1_pinmap, rx_config)
        .unwrap();

    // Spawn tasks
    spawner.spawn(sensors::lidar_publisher(event_channel)).ok();
    spawner.spawn(sensors::accelerometer_task(spi, event_channel)).ok();
    spawner.spawn(sensors::rmt_task(rmt_ch0, event_channel)).ok();

    // Initialize the Wi-Fi controller
    let esp_wifi_ctrl = &*mk_static!(
        EspWifiController<'static>,
        esp_wifi::init(timg0.timer0, rng.clone(), peripherals.RADIO_CLK).unwrap()
    );
    let (mut controller, interfaces) =
        esp_wifi::wifi::new(&esp_wifi_ctrl, peripherals.WIFI).unwrap();
    let wifi_ap_device = interfaces.ap;
    let wifi_sta_device = interfaces.sta;

    // Configure Wi-Fi in AP and STA mode
    let ap_config = embassy_net::Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(Ipv4Addr::new(192, 168, 2, 1), 24),
        gateway: Some(Ipv4Addr::new(192, 168, 2, 1)),
        dns_servers: Default::default(),
    });
    let sta_config = embassy_net::Config::dhcpv4(Default::default());

    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    // Init network stacks
    let (ap_stack, ap_runner) = embassy_net::new(
        wifi_ap_device,
        ap_config,
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );
    let (sta_stack, sta_runner) = embassy_net::new(
        wifi_sta_device,
        sta_config,
        mk_static!(StackResources<4>, StackResources::<4>::new()),
        seed,
    );

    let client_config = Configuration::Mixed(
        ClientConfiguration {
            ssid: SSID.into(),
            password: PASSWORD.into(),
            ..Default::default()
        },
        AccessPointConfiguration {
            ssid: "kasarisw".into(),
            ..Default::default()
        },
    );
    controller.set_configuration(&client_config).unwrap();

    spawner.spawn(connection_task(controller)).ok();
    spawner.spawn(net_task(ap_runner)).ok();
    spawner.spawn(net_task(sta_runner)).ok();

    let sta_address = loop {
        if let Some(config) = sta_stack.config_v4() {
            let address = config.address.address();
            println!("Got IP: {}", address);
            break address;
        }
        println!("Waiting for IP...");
        Timer::after(Duration::from_millis(500)).await;
    };
    loop {
        if ap_stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
    println!("Connect to the AP `esp-wifi` and connect to TCP 192.168.2.1:8080");
    println!("Or connect to the ap `{SSID}` and connect to TCP {sta_address}:8080");

    spawner.spawn(listener_task(ap_stack, sta_stack, event_channel)).ok();

    // Main loop
    let mut logic = shared::kasari::MainLogic::new();
    let mut publisher = event_channel.publisher().unwrap();
    let mut subscriber = event_channel.subscriber().unwrap();
    let mut loop_i: u64 = 0;

    loop {
        loop_i += 1;

        // Measure and publish Vbat
        if loop_i % 10 == 0 {
            let vbat_raw = nb::block!(adc1.read_oneshot(&mut vbat_pin)).unwrap();
            let vbat = (4095 - vbat_raw) as f32 * 0.01045;
            if shared::LOG_VBAT {
                esp_println::println!("vbat_raw: {:?}, vbat: {:?} V", vbat_raw, vbat);
            }

            publisher.publish_immediate(
                shared::kasari::InputEvent::Vbat(
                    embassy_time::Instant::now().as_ticks(),
                    vbat,
                )
            );
        }

        while let Some(event) = subscriber.try_next_message_pure() {
            logic.feed_event(event);
        }

        logic.step();

        if let Some(ref plan) = logic.motor_control_plan {
            let timestamp = embassy_time::Instant::now().as_ticks();
            if timestamp - plan.timestamp < 2000 {
                let target_speed_percent = plan.throttle;
                let duty = target_speed_to_pwm_duty(target_speed_percent, 2u32.pow(8));
                if shared::LOG_RECEIVER {
                    esp_println::println!("Setting duty cycle: {:?}", duty);
                }
                _ = channel0.set_duty_hw(duty);
                _ = channel1.set_duty_hw(duty);
            } else {
                _ = channel0.set_duty(0);
                _ = channel1.set_duty(0);
            }
        } else {
            _ = channel0.set_duty(0);
            _ = channel1.set_duty(0);
        }

        Timer::after_millis(20).await;
    }
}

#[esp_hal::handler]
fn uart2_handler() {
    //println!("uart2_handler");
    critical_section::with(|cs| {
        let mut uart_ref = unsafe { UART2_REF.unwrap().borrow(cs).borrow_mut() };
        if let Some(uart) = uart_ref.as_mut() {
            let mut temp = [0u8; sensors::PACKET_SIZE * 2];
            let read_bytes = uart.read_buffered(&mut temp).unwrap_or(0);
            uart.clear_interrupts(UartInterrupt::RxFifoFull.into());

            //println!("read_bytes={}", read_bytes);
            let mut buf = unsafe { LIDAR_BYTE_BUFFER_REF.unwrap().borrow(cs).borrow_mut() };
            for &byte in &temp[0..read_bytes] {
                buf.push(byte);
            }

            while buf.len() >= sensors::PACKET_SIZE {
                if buf.get(0) != Some(&sensors::HEAD_BYTE) {
                    while let Some(byte) = buf.get(0) {
                        if *byte == sensors::HEAD_BYTE {
                            break;
                        }
                        buf.dequeue();
                    }
                    continue;
                }

                let mut packet = [0u8; sensors::PACKET_SIZE];
                for i in 0..sensors::PACKET_SIZE {
                    packet[i] = buf.get(i).copied().unwrap_or(0);
                }

                if let Some(parsed) = sensors::parse_packet(&packet) {
                    let timestamp = embassy_time::Instant::now().as_ticks();
                    let event = kasari::InputEvent::Lidar(
                        timestamp,
                        parsed.distances[0] as f32,
                        parsed.distances[1] as f32,
                        parsed.distances[2] as f32,
                        parsed.distances[3] as f32,
                    );
                    let mut queue = unsafe { LIDAR_EVENT_QUEUE_REF.unwrap().borrow(cs).borrow_mut() };
                    queue.push_back(event);

                    let packet_i = LIDAR_PACKET_COUNT.fetch_add(1, Ordering::Relaxed);;
                    if packet_i % 20 == 1 && shared::LOG_LIDAR {
                        println!(
                            "Lidar packet {}: timestamp={}, distances=[{}, {}, {}, {}] mm",
                            packet_i,
                            timestamp,
                            parsed.distances[0],
                            parsed.distances[1],
                            parsed.distances[2],
                            parsed.distances[3]
                        );
                    }

                    for _ in 0..sensors::PACKET_SIZE {
                        buf.dequeue();
                    }
                } else {
                    println!("Invalid packet");
                    buf.clear();
                }
            }
        }
    });
    unsafe { SIGNAL_LIDAR_REF.unwrap().signal(()); }
}

#[embassy_executor::task]
async fn connection_task(mut controller: WifiController<'static>) {
    println!("start connection task");
    println!("Device capabilities: {:?}", controller.capabilities());

    println!("Starting wifi");
    controller.start_async().await.unwrap();
    println!("Wifi started!");

    loop {
        match esp_wifi::wifi::ap_state() {
            WifiState::ApStarted => {
                println!("About to connect...");

                match controller.connect_async().await {
                    Ok(_) => {
                        // wait until we're no longer connected
                        controller.wait_for_event(WifiEvent::StaDisconnected).await;
                        println!("STA disconnected");
                    }
                    Err(e) => {
                        println!("Failed to connect to wifi: {e:?}");
                        Timer::after(Duration::from_millis(5000)).await
                    }
                }
            }
            _ => return,
        }
    }
}

#[embassy_executor::task(pool_size = 2)]
async fn net_task(mut runner: Runner<'static, WifiDevice<'static>>) {
    runner.run().await
}

use crate::shared::EventChannel;
use embassy_futures::select::{select, Either};
use embedded_io_async::{Read, Write};

#[embassy_executor::task]
async fn listener_task(ap_stack: embassy_net::Stack<'static>, sta_stack: embassy_net::Stack<'static>, event_channel: &'static EventChannel) {
    let mut subscriber = event_channel.subscriber().unwrap();
    let mut publisher = event_channel.publisher().unwrap();

    let mut ap_rx_buffer = [0; 1536];
    let mut ap_tx_buffer = [0; 1536];
    let mut sta_rx_buffer = [0; 1536];
    let mut sta_tx_buffer = [0; 1536];

    let mut ap_socket = TcpSocket::new(ap_stack, &mut ap_rx_buffer, &mut ap_tx_buffer);
    let mut sta_socket = TcpSocket::new(sta_stack, &mut sta_rx_buffer, &mut sta_tx_buffer);

    loop {
        println!("Wait for connection...");
        let either = select(
            ap_socket.accept(IpListenEndpoint { addr: None, port: 8080 }),
            sta_socket.accept(IpListenEndpoint { addr: None, port: 8080 }),
        ).await;

        let (r, mut socket) = match either {
            Either::First(r) => (r, &mut ap_socket),
            Either::Second(r) => (r, &mut sta_socket),
        };

        if let Err(e) = r {
            println!("Accept error: {:?}", e);
            continue;
        }

        println!("Connected!");
        socket.set_timeout(None);

        let mut rx_buffer = [0u8; 512];
        let mut rx_pos = 0;
        //let mut serialized_event_count: u32 = 0; // Instrumentation

        loop {
            let read_fut = socket.read(&mut rx_buffer[rx_pos..]);
            let event_fut = subscriber.next_message_pure();

            match select(read_fut, event_fut).await {
                Either::Second(event) => {
                    let serialized = kasari::serialize_event(&event);
                    /*serialized_event_count += 1;
                    if serialized_event_count % 20 == 0 {
                        println!("-!- serialized_event_count = {}", serialized_event_count);
                    }*/
                    if let Err(e) = socket.write_all(&serialized).await {
                        println!("Write error: {:?}", e);
                        break;
                    }
                    // Don't flush here. Events are too small and throughput
                    // will be severely limited.
                }
                Either::First(Ok(0)) => {
                    println!("Client disconnected");
                    break;
                }
                Either::First(Ok(len)) => {
                    rx_pos += len;
                    // Process binary events
                    while rx_pos >= 1 {
                        let tag = rx_buffer[0];
                        if tag == 4 {  // Assuming tag 4 for WifiControl
                            if rx_pos >= 1 + 8 + 1 + 4 + 4 + 4 {  // u8 tag + u64 ts + u8 mode + 3*f32
                                let ts = u64::from_le_bytes(rx_buffer[1..9].try_into().unwrap());
                                let mode = rx_buffer[9];
                                let r = f32::from_le_bytes(rx_buffer[10..14].try_into().unwrap());
                                let m = f32::from_le_bytes(rx_buffer[14..18].try_into().unwrap());
                                let t = f32::from_le_bytes(rx_buffer[18..22].try_into().unwrap());
                                publisher.publish_immediate(
                                    shared::kasari::InputEvent::WifiControl(ts, mode, r, m, t)                        );
                                // Shift buffer
                                let shift_len = 22;
                                rx_buffer.copy_within(shift_len.., 0);
                                rx_pos -= shift_len;

                                // Flush the sending socket as we receive valid
                                // events. This should be a good interval for
                                // flushing
                                if let Err(e) = socket.flush().await {
                                    println!("Flush error: {:?}", e);
                                    break;
                                }
                            } else {
                                break;
                            }
                        } else {
                            // Unknown tag, skip or handle error
                            println!("Unknown tag: {}", tag);
                            // Shift by 1 to skip invalid tag
                            rx_buffer.copy_within(1.., 0);
                            rx_pos -= 1;
                        }
                    }
                    if rx_pos == rx_buffer.len() {
                        println!("Buffer full without complete event, dropping data");
                        rx_pos = 0;
                    }
                }
                Either::First(Err(e)) => {
                    println!("Read error: {:?}", e);
                    break;
                }
            }
        }

        socket.close();
        Timer::after(Duration::from_millis(1000)).await;
        socket.abort();
    }
}
