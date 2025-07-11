#![no_std]
use core::cell::RefCell;
use critical_section::Mutex;
use ringbuffer::ConstGenericRingBuffer;
use embassy_sync::pubsub::PubSubChannel;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use static_cell::StaticCell;

pub const LOG_LIDAR: bool = false;
pub const LOG_RECEIVER: bool = false;
pub const LOG_VBAT: bool = false;

pub type EventChannel = PubSubChannel<CriticalSectionRawMutex, kasari::InputEvent, 64, 2, 6>;
pub static EVENT_CHANNEL: StaticCell<EventChannel> = StaticCell::new();

pub mod kasari {
    use crate::shared::LOG_RECEIVER;
    use esp_println::println;

	#[derive(Clone)]
    pub enum InputEvent {
        Lidar(u64, f32, f32, f32, f32), // timestamp, distance samples (mm)
        Accelerometer(u64, f32, f32),        // timestamp, acceleration Y (G), acceleration Z (G)
        Receiver(u64, u8, Option<f32>), // timestamp, channel (0=throttle), pulse length (us)
        Vbat(u64, f32),                 // timestamp, battery voltage (V)
    }

    pub struct MotorControlPlan {
        pub throttle: f32,             // -1.0...1.0
        pub modulation_amplitude: f32, // -0.0....1.0
        pub modulation_phase: f32,     // Some kind of an angle
    }

    pub struct MainLogic {
        pub motor_control_plan: Option<MotorControlPlan>,
        acceleration_y: f32,
        acceleration_z: f32,
        vbat: f32,
        vbat_ok: bool,
    }

    impl MainLogic {
        pub fn new() -> Self {
            Self {
                motor_control_plan: None,
                acceleration_y: 0.0,
                acceleration_z: 0.0,
                vbat: 0.0,
                vbat_ok: false,
            }
        }

        pub fn feed_event(&mut self, event: InputEvent) {
            match event {
                InputEvent::Lidar(_timestamp, _d1, _d2, _d3, _d4) => {}
                InputEvent::Accelerometer(_timestamp, a_y, a_z) => {
                    self.acceleration_y = a_y;
                    self.acceleration_z = a_z;
                }
                InputEvent::Receiver(_timestamp, _ch, pulse_length) => {
                    if !self.vbat_ok {
                        self.motor_control_plan = None;
                        return;
                    }
                    match pulse_length {
                        Some(pulse_length) => {
                            let target_speed_percent = (pulse_length - 1500.0) * 0.2;
                            if LOG_RECEIVER {
                                println!(
                                    "pulse_length -> target_speed_percent: {:?} -> {:?}",
                                    pulse_length,
                                    target_speed_percent
                                );
                            }
                            self.motor_control_plan = Some(MotorControlPlan {
                                throttle: target_speed_percent,
                                modulation_amplitude: 0.0,
                                modulation_phase: 0.0,
                            });
                        }
                        None => {
                            self.motor_control_plan = None;
                        }
                    }
                }
                InputEvent::Vbat(_timestamp, vbat) => {
                    self.vbat = vbat;
                    if self.vbat_ok {
                        self.vbat_ok = vbat > 9.0;
                    } else {
                        self.vbat_ok = vbat > 10.0;
                    }
                }
            }
        }

        pub fn step(&mut self) {}
    }
}
