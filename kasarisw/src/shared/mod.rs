// kasarisw/src/shared/mod.rs
#![cfg_attr(target_os = "none", no_std)]
use core::cell::RefCell;
use critical_section::Mutex;
use ringbuffer::ConstGenericRingBuffer;
use embassy_sync::pubsub::PubSubChannel;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use static_cell::StaticCell;
use num_traits::ops::euclid::Euclid;
use num_traits::float::FloatCore;

#[cfg(target_os = "none")]
use embassy_time::Instant;
#[cfg(not(target_os = "none"))]
use std::time::{SystemTime, UNIX_EPOCH};

mod algorithm;
use algorithm::ObjectDetector;

pub const LOG_LIDAR: bool = false;
pub const LOG_ALL_LIDAR: bool = false;
pub const LOG_RECEIVER: bool = false;
pub const LOG_VBAT: bool = false;

pub type EventChannel = PubSubChannel<CriticalSectionRawMutex, kasari::InputEvent, 32, 2, 6>;
pub static EVENT_CHANNEL: StaticCell<EventChannel> = StaticCell::new();

pub fn get_current_timestamp() -> u64 {
    #[cfg(target_os = "none")]
    {
        Instant::now().as_ticks()
    }
    #[cfg(not(target_os = "none"))]
    {
        SystemTime::now().duration_since(UNIX_EPOCH).expect("Time went backwards").as_micros() as u64
    }
}

// We need to define this on our own, because the num-traits one for some reason
// wants &f32 on ESP32 and f32 on PC
pub fn rem_euclid_f32(x: f32, y: f32) -> f32 {
    #[cfg(not(target_os = "none"))]
    { x.rem_euclid(y) }
    #[cfg(target_os = "none")]
    { x.rem_euclid(&y) }
}

pub mod kasari {
    use crate::shared::{LOG_RECEIVER, get_current_timestamp};
    #[cfg(target_os = "none")]
	use alloc::vec::Vec;
    #[cfg(not(target_os = "none"))]
    use std::vec::Vec;
    #[cfg(target_os = "none")]
    use esp_println::println;
	use crate::shared::ObjectDetector;

	#[derive(Clone)]
    pub enum InputEvent {
        Lidar(u64, f32, f32, f32, f32), // timestamp, distance samples (mm)
        Accelerometer(u64, f32, f32),        // timestamp, acceleration Y (G), acceleration Z (G)
        Receiver(u64, u8, Option<f32>), // timestamp, channel (0=throttle), pulse length (us)
        Vbat(u64, f32),                 // timestamp, battery voltage (V)
        WifiControl(u64, u8, f32, f32, f32),// timestamp, mode, rotation speed, movement speed, turning speed
    }

    pub struct MotorControlPlan {
		pub timestamp: u64,
        pub throttle: f32,             // -1.0...1.0
        pub modulation_amplitude: f32, // -0.0....1.0
        pub modulation_phase: f32,     // Some kind of an angle
    }

    pub struct MainLogic {
        pub motor_control_plan: Option<MotorControlPlan>,
        pub detector: ObjectDetector,
        acceleration_y: f32,
        acceleration_z: f32,
        vbat: f32,
        vbat_ok: bool,
        control_mode: u8,
        control_rotation_speed: f32,
        control_movement_speed: f32,
        control_turning_speed: f32,
    }

    impl MainLogic {
        pub fn new() -> Self {
            Self {
                motor_control_plan: None,
				detector: ObjectDetector::new(),
                acceleration_y: 0.0,
                acceleration_z: 0.0,
                vbat: 0.0,
                vbat_ok: false,
				control_mode: 0,
				control_rotation_speed: 0.0,
				control_movement_speed: 0.0,
				control_turning_speed: 0.0,
            }
        }

        pub fn feed_event(&mut self, event: InputEvent) {
            self.detector.update(&event);

            match event {
                InputEvent::Lidar(_timestamp, _d1, _d2, _d3, _d4) => {}
                InputEvent::Accelerometer(_timestamp, a_y, a_z) => {
                    self.acceleration_y = a_y;
                    self.acceleration_z = a_z;
                }
                InputEvent::Receiver(timestamp, _ch, pulse_length) => {
					if self.control_mode != 0 {
						return;
					}
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
								timestamp: timestamp,
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
				InputEvent::WifiControl(_timestamp, mode, r, m, t) => {
                    println!("WifiControl({}, {}, {}, {})", mode, r, m, t);
					self.control_mode = mode;
					self.control_rotation_speed = r;
					self.control_movement_speed = m;
					self.control_turning_speed = t;
					if self.control_mode != 1 && self.control_mode != 2 {
						return;
					}
                    if !self.vbat_ok {
                        self.motor_control_plan = None;
                        return;
                    }
                    if self.control_mode == 1 {
						let timestamp = get_current_timestamp();
						self.motor_control_plan = Some(MotorControlPlan {
							timestamp: timestamp,
							throttle: self.control_rotation_speed,
							modulation_amplitude: 0.0,
							modulation_phase: 0.0,
						});
					}
				}
            }
        }

        pub fn step(&mut self) {
            let (closest_wall, open_space, object_pos) = self.detector.detect_objects(false);
            println!("Detected: Wall {:?}, Open {:?}, Object {:?} (points.len()={})",
                    closest_wall, open_space, object_pos, self.detector.points.len());
        }
    }

	const TAG_XOR: u16 = 0x5555;

	pub fn serialize_event(event: &InputEvent) -> Vec<u8> {
		let mut buf = Vec::with_capacity(32);
		match event {
			InputEvent::Lidar(ts, d1, d2, d3, d4) => {
				let tag = (0u16 ^ TAG_XOR).to_le_bytes();
				buf.extend_from_slice(&tag);
				buf.extend_from_slice(&ts.to_le_bytes());
				buf.extend_from_slice(&d1.to_le_bytes());
				buf.extend_from_slice(&d2.to_le_bytes());
				buf.extend_from_slice(&d3.to_le_bytes());
				buf.extend_from_slice(&d4.to_le_bytes());
			}
			InputEvent::Accelerometer(ts, accel_y, accel_z) => {
				let tag = (1u16 ^ TAG_XOR).to_le_bytes();
				buf.extend_from_slice(&tag);
				buf.extend_from_slice(&ts.to_le_bytes());
				buf.extend_from_slice(&accel_y.to_le_bytes());
				buf.extend_from_slice(&accel_z.to_le_bytes());
			}
			InputEvent::Receiver(ts, channel, pulse_length) => {
				let tag = (2u16 ^ TAG_XOR).to_le_bytes();
				buf.extend_from_slice(&tag);
				buf.extend_from_slice(&ts.to_le_bytes());
				buf.push(*channel);
				match pulse_length {
					Some(pl) => {
						buf.push(1);  // Flag 1
						buf.extend_from_slice(&pl.to_le_bytes());
					}
					None => {
						buf.push(0);  // Flag 0
					}
				}
			}
			InputEvent::Vbat(ts, voltage) => {
				let tag = (3u16 ^ TAG_XOR).to_le_bytes();
				buf.extend_from_slice(&tag);
				buf.extend_from_slice(&ts.to_le_bytes());
				buf.extend_from_slice(&voltage.to_le_bytes());
			}
			InputEvent::WifiControl(ts, mode, r, m, t) => {
				let tag = (4u16 ^ TAG_XOR).to_le_bytes();
				buf.extend_from_slice(&tag);
				buf.extend_from_slice(&ts.to_le_bytes());
				buf.push(*mode);
				buf.extend_from_slice(&r.to_le_bytes());
				buf.extend_from_slice(&m.to_le_bytes());
				buf.extend_from_slice(&t.to_le_bytes());
			}
		}
		buf
	}
}
