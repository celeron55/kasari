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
    uart::{Uart, UartRx, UartTx},
    Async,
};
use esp_hal::ledc::timer::TimerIFace;
use esp_hal::ledc::channel::ChannelIFace;
use esp_println::{print, println};
use static_cell::StaticCell;
use ringbuffer::RingBuffer;
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
extern crate alloc;

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

#[embassy_executor::task]
async fn encoder_emulation_task(encoder_emulation_output_pinmap: AnyPin<'static>) {
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

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let mut rng = esp_hal::rng::Rng::new(peripherals.RNG);
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let timg1 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timg1.timer0);

	// This has to be initialized before any tasks are started
	let event_channel = &*shared::EVENT_CHANNEL.init(PubSubChannel::new());

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

    _ = channel0.set_duty(0);
    _ = channel1.set_duty(0);

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

    // Spawn tasks
    static SIGNAL: StaticCell<embassy_sync::signal::Signal<embassy_sync::blocking_mutex::raw::NoopRawMutex, usize>> = StaticCell::new();
    let signal = &*SIGNAL.init(embassy_sync::signal::Signal::new());

    spawner.spawn(sensors::lidar_reader(rx, signal, event_channel)).ok();
    //spawner.spawn(sensors::lidar_writer(tx, signal)).ok();
    spawner.spawn(encoder_emulation_task(encoder_emulation_output_pinmap)).ok();
    spawner.spawn(sensors::accelerometer_task(spi, event_channel)).ok();
    spawner.spawn(sensors::rmt_task(rmt_ch0, event_channel)).ok();

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

        loop {
            let read_fut = socket.read(&mut rx_buffer[rx_pos..]);
            let event_fut = subscriber.next_message_pure();

            match select(read_fut, event_fut).await {
                Either::Second(event) => {
                    let serialized = kasari::serialize_event(&event);
                    if let Err(e) = socket.write_all(&serialized).await {
                        println!("Write error: {:?}", e);
                        break;
                    }
                    if let Err(e) = socket.flush().await {
                        println!("Flush error: {:?}", e);
                        break;
                    }
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
                                    shared::kasari::InputEvent::WifiControl(ts, mode, r, m, t)
                                );
                                // Shift buffer
                                let shift_len = 22;
                                rx_buffer.copy_within(shift_len.., 0);
                                rx_pos -= shift_len;
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
