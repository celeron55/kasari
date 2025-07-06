#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_alloc as _;
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
    uart::{Uart, UartRx, UartTx},
    Async,
};
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::ledc::channel::ChannelIFace;
use esp_println::println;
use static_cell::StaticCell;
use ringbuffer::RingBuffer;

mod sensors;
mod shared;

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

#[embassy_executor::task]
async fn encoder_emulation_task(encoder_emulation_output_pinmap: AnyPin) {
    let mut encoder_emulation_output_pin = Output::new(
        encoder_emulation_output_pinmap,
        Level::High,
        OutputConfig::default(),
    );

    loop {
        encoder_emulation_output_pin.set_low();
        Timer::after(Duration::from_millis(10)).await;
        encoder_emulation_output_pin.set_high();
        Timer::after(Duration::from_millis(10)).await;
    }
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

    // ADC (battery voltage monitoring)
    let mut adc1_config = AdcConfig::new();
    let mut vbat_pin = adc1_config.enable_pin(peripherals.GPIO39, adc::Attenuation::_11dB);
    let mut adc1 = Adc::new(peripherals.ADC1, adc1_config);

    // PWM output to ESCs
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

    channel0.set_duty(0);
    channel1.set_duty(0);

    // UART2
    let (tx_pin, rx_pin) = (peripherals.GPIO17, peripherals.GPIO16);
    let config = esp_hal::uart::Config::default()
        .with_rx(esp_hal::uart::RxConfig::default().with_fifo_full_threshold(sensors::READ_BUF_SIZE as u16))
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
    static SIGNAL: StaticCell<embassy_sync::signal::Signal<embassy_sync::blocking_mutex::raw::NoopRawMutex, usize>> = StaticCell::new();
    let signal = &*SIGNAL.init(embassy_sync::signal::Signal::new());

    spawner.spawn(sensors::lidar_reader(rx, signal)).ok();
    spawner.spawn(sensors::lidar_writer(tx, signal)).ok();
    spawner.spawn(encoder_emulation_task(encoder_emulation_output_pinmap)).ok();
    spawner.spawn(sensors::accelerometer_task(spi)).ok();
    spawner.spawn(sensors::rmt_task(rmt_ch0)).ok();

    // Main loop
    let mut logic = shared::kasari::MainLogic::new();

    loop {
        let vbat_raw = nb::block!(adc1.read_oneshot(&mut vbat_pin)).unwrap();
        let vbat = (4095 - vbat_raw) as f32 * 0.01045;
        if shared::LOG_VBAT {
            esp_println::println!("vbat_raw: {:?}, vbat: {:?} V", vbat_raw, vbat);
        }

        let event_queue = critical_section::with(|cs| {
            shared::GLOBAL_EVENT_QUEUE
                .borrow(cs)
                .replace(Some(ringbuffer::ConstGenericRingBuffer::new()))
        });

        logic.feed_event(shared::kasari::InputEvent::Vbat(
            embassy_time::Instant::now().as_ticks(),
            vbat,
        ));

        if let Some(mut event_queue) = event_queue {
            while let Some(event) = event_queue.dequeue() {
                logic.feed_event(event);
            }
        }

        logic.step();

        if let Some(ref plan) = logic.motor_control_plan {
            let target_speed_percent = plan.throttle;
            let duty = target_speed_to_pwm_duty(target_speed_percent, 2u32.pow(8));
            if shared::LOG_RECEIVER {
                esp_println::println!("Setting duty cycle: {:?}", duty);
            }
            channel0.set_duty_hw(duty);
            channel1.set_duty_hw(duty);
        } else {
            channel0.set_duty(0);
            channel1.set_duty(0);
        }

        Timer::after_millis(20).await;
    }
}
