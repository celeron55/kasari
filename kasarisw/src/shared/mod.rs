// mod.rs
#![cfg_attr(target_os = "none", no_std)]
use core::cell::RefCell;
use critical_section::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pubsub::PubSubChannel;
use num_traits::float::FloatCore;
use num_traits::ops::euclid::Euclid;
use ringbuffer::ConstGenericRingBuffer;
use static_cell::StaticCell;

#[cfg(target_os = "none")]
use embassy_time::Instant;
#[cfg(not(target_os = "none"))]
use std::time::{SystemTime, UNIX_EPOCH};

pub mod algorithm;
use algorithm::{DetectionResult, ObjectDetector};

pub const TARGET_RPM: f32 = 1000.0;
pub const MIN_MOVE_RPM: f32 = 550.0;
pub const MIN_ATTACK_RPM: f32 = 800.0;

pub const MAX_RPM_RAMP_RATE: f32 = 2000.0; // rpm/s
pub const RPM_INITIAL_JUMP: f32 = 500.0; // rpm

pub const LOG_LIDAR: bool = false;
pub const LOG_ALL_LIDAR: bool = false;
pub const LOG_RECEIVER: bool = false;
pub const LOG_WIFI_CONTROL: bool = false;
pub const LOG_MOTOR_CONTROL: bool = false;
pub const LOG_VBAT: bool = false;
pub const LOG_DETECTION: bool = false;

pub type EventChannel = PubSubChannel<CriticalSectionRawMutex, kasari::InputEvent, 32, 2, 6>;
pub static EVENT_CHANNEL: StaticCell<EventChannel> = StaticCell::new();

pub fn get_current_timestamp() -> u64 {
    #[cfg(target_os = "none")]
    {
        Instant::now().as_ticks()
    }
    #[cfg(not(target_os = "none"))]
    {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("Time went backwards")
            .as_micros() as u64
    }
}

// We need to define this on our own, because the num-traits one for some reason
// wants &f32 on ESP32 and f32 on PC
pub fn rem_euclid_f32(x: f32, y: f32) -> f32 {
    #[cfg(not(target_os = "none"))]
    {
        x.rem_euclid(y)
    }
    #[cfg(target_os = "none")]
    {
        x.rem_euclid(&y)
    }
}

pub mod kasari {
    use crate::shared::rem_euclid_f32;
    use crate::shared::CriticalSectionRawMutex;
    use crate::shared::ObjectDetector;
    use crate::shared::{
        get_current_timestamp, LOG_DETECTION, LOG_RECEIVER, LOG_VBAT, LOG_WIFI_CONTROL,
        MAX_RPM_RAMP_RATE, MIN_ATTACK_RPM, MIN_MOVE_RPM, RPM_INITIAL_JUMP, TARGET_RPM,
    };
    #[cfg(target_os = "none")]
    use alloc::vec::Vec;
    use core::f32::consts::PI;
    #[cfg(target_os = "none")]
    use esp_println::println;
    use libm::{atan2f, cosf, fabsf, sqrtf};
    #[cfg(not(target_os = "none"))]
    use std::vec::Vec;

    #[derive(Clone, Debug)]
    pub enum InputEvent {
        Lidar(u64, f32, f32, f32, f32), // timestamp, distance samples (mm)
        Accelerometer(u64, f32, f32),   // timestamp, acceleration Y (G), acceleration Z (G)
        Receiver(u64, u8, Option<f32>), // timestamp, channel (0=throttle), pulse length (us)
        Vbat(u64, f32),                 // timestamp, battery voltage (V)
        WifiControl(u64, u8, f32, f32, f32), // timestamp, mode, rotation speed, movement speed, turning speed
        Planner(
            u64,
            MotorControlPlan,
            (f32, f32),
            (f32, f32),
            (f32, f32),
            f32,
            f32,
        ), // timestamp, MCP, latest_closest_wall, latest_open_space, latest_object_pos, measured_rpm
    }

    #[derive(Clone, Copy, Debug)]
    pub struct MotorControlPlan {
        pub timestamp: u64,
        pub rotation_speed: f32,
        pub movement_x: f32,
        pub movement_y: f32,
    }

    pub struct MainLogic {
        pub motor_control_plan: Option<MotorControlPlan>,
        pub detector: ObjectDetector,
        vbat: f32,
        vbat_ok: bool,
        pub battery_present: bool,
        control_mode: u8,
        control_rotation_speed: f32,
        control_movement_speed: f32,
        control_turning_speed: f32,
        pub latest_closest_wall: (f32, f32),
        pub latest_open_space: (f32, f32),
        pub latest_object_pos: (f32, f32),
        pub latest_wall_distances: (f32, f32, f32, f32),
        pub latest_position: (f32, f32),
        pub latest_velocity: (f32, f32),
        last_planner_ts: u64,
        autonomous_enabled: bool,
        autonomous_start_ts: Option<u64>,
        autonomous_cycle_period_us: u64,
        autonomous_duty_cycle: f32,
        last_rpm_update_ts: Option<u64>,
        current_rotation_speed: f32,
    }

    impl MainLogic {
        pub fn new() -> Self {
            Self {
                motor_control_plan: None,
                detector: ObjectDetector::new(),
                vbat: 0.0,
                vbat_ok: false,
                battery_present: false,
                control_mode: 0,
                control_rotation_speed: 0.0,
                control_movement_speed: 0.0,
                control_turning_speed: 0.0,
                latest_closest_wall: (0.0, 0.0),
                latest_open_space: (0.0, 0.0),
                latest_object_pos: (0.0, 0.0),
                latest_wall_distances: (0.0, 0.0, 0.0, 0.0),
                latest_position: (0.0, 0.0),
                latest_velocity: (0.0, 0.0),
                last_planner_ts: 0,
                autonomous_enabled: false,
                autonomous_start_ts: None,
                autonomous_cycle_period_us: 5_000_000, // 5 seconds
                autonomous_duty_cycle: 0.6,            // 60% towards center, 40% towards object
                last_rpm_update_ts: None,
                current_rotation_speed: 0.0,
            }
        }

        pub fn feed_event(&mut self, event: InputEvent) {
            self.detector.update(&event);
            self.detector.rpm = self.detector.rpm.abs() * self.current_rotation_speed.signum();

            match event {
                InputEvent::Lidar(_timestamp, _d1, _d2, _d3, _d4) => {}
                InputEvent::Accelerometer(_timestamp, _a_y, _a_z) => {}
                InputEvent::Receiver(timestamp, _ch, pulse_length) => {
                    // Mode 0 = RC Receiver control mode
                    if self.control_mode != 0 {
                        return;
                    }
                    if !self.vbat_ok {
                        self.autonomous_enabled = false;
                        self.motor_control_plan = None;
                        return;
                    }
                    match pulse_length {
                        Some(pulse_length) => {
                            let stick_percent = (pulse_length - 1500.0) * 0.2;
                            if LOG_RECEIVER {
                                println!(
                                    "pulse_length -> stick_percent: {:?} -> {:?}",
                                    pulse_length, stick_percent
                                );
                            }
                            if stick_percent >= 5.0 {
                                self.autonomous_enabled = true;
                            } else {
                                self.autonomous_enabled = false;
                                // We want a zeroed out motor control plan
                                // instead of None because the ESCs won't
                                // initialize unless we give them a moment of
                                // zeroed out throttle input first.
                                // Unfortunately this will brake hard when
                                // autonomous mode is turned off afterwards,
                                // which could be fixed by adding a variable
                                // which tracks whether autonomous mode was
                                // turned on previously and if it was, then this
                                // would be set to None to avoid braking hard.
                                self.motor_control_plan = Some(MotorControlPlan {
                                    timestamp: timestamp,
                                    rotation_speed: 0.0,
                                    movement_x: 0.0,
                                    movement_y: 0.0,
                                });
                            }
                        }
                        None => {
                            self.autonomous_enabled = false;
                            self.motor_control_plan = None;
                        }
                    }
                }
                InputEvent::Vbat(_timestamp, vbat) => {
                    self.vbat = vbat;
                    self.battery_present = if self.battery_present {
                        vbat >= 4.0
                    } else {
                        vbat >= 4.5
                    };
                    self.vbat_ok = if self.vbat_ok {
                        vbat >= 9.0
                    } else {
                        vbat >= 10.0
                    };
                    if LOG_VBAT {
                        println!(
                            "Vbat: {} V, ok: {}, present: {}",
                            self.vbat, self.vbat_ok, self.battery_present
                        );
                    }
                }
                InputEvent::WifiControl(_timestamp, mode, r, m, t) => {
                    if LOG_WIFI_CONTROL {
                        println!("WifiControl({}, {}, {}, {})", mode, r, m, t);
                    }
                    self.control_mode = mode;
                    self.autonomous_enabled = mode == 2;
                    if !self.autonomous_enabled {
                        self.autonomous_start_ts = None;
                    }
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
                            rotation_speed: self.control_rotation_speed,
                            movement_x: 0.0,
                            movement_y: 0.0,
                        });
                    }
                }
                InputEvent::Planner(..) => {}
            }
        }

        pub fn autonomous_update(&mut self, ts: u64) {
            if !self.vbat_ok {
                self.motor_control_plan = None;
                return;
            }
            let rotation_speed = TARGET_RPM;

            let mut movement_x = 0.0;
            let mut movement_y = 0.0;

            if fabsf(self.detector.rpm) >= MIN_MOVE_RPM {
                let wall_x = self.latest_closest_wall.0;
                let wall_y = self.latest_closest_wall.1;
                let wall_dist = sqrtf(wall_x * wall_x + wall_y * wall_y);
                let obj_x = self.latest_object_pos.0;
                let obj_y = self.latest_object_pos.1;
                let obj_dist = sqrtf(obj_x * obj_x + obj_y * obj_y);

                let detection_failing = self.detector.arena_w == 0.0 || obj_x == 100.0;

                if detection_failing {
                    // Fallback: move away from closest wall towards open space
                    let away_x = -wall_x;
                    let away_y = -wall_y;
                    let target_x = (away_x + self.latest_open_space.0) / 2.0;
                    let target_y = (away_y + self.latest_open_space.1) / 2.0;
                    let target_len = sqrtf(target_x * target_x + target_y * target_y);
                    if target_len > 0.0 {
                        movement_x = target_x / target_len * 1.0;
                        movement_y = target_y / target_len * 1.0;
                    }
                } else {
                    // Normal mode: alternate between center and object
                    if self.autonomous_start_ts.is_none() {
                        self.autonomous_start_ts = Some(ts);
                    }
                    let cycle_ts = ts - self.autonomous_start_ts.unwrap();
                    let phase = (cycle_ts % self.autonomous_cycle_period_us) as f32
                        / self.autonomous_cycle_period_us as f32;
                    // Pick a target and a speed depending on what the target is
                    let ((target_x, target_y), speed_suggestion) = if phase
                        < self.autonomous_duty_cycle
                        || fabsf(self.detector.rpm) < MIN_ATTACK_RPM
                    {
                        // Towards center
                        let center_x = self.latest_open_space.0;
                        let center_y = self.latest_open_space.1;
                        let center_len = sqrtf(center_x * center_x + center_y * center_y);
                        let obj_len = sqrtf(obj_x * obj_x + obj_y * obj_y);

                        // Check if object is nearly aligned with center and
                        // closer than center, meaning we have to go around the
                        // object in order to reach center
                        let angle_center = atan2f(center_y, center_x);
                        let angle_object = atan2f(obj_y, obj_x);
                        let angle_diff =
                            fabsf(rem_euclid_f32(angle_center - angle_object + PI, 2.0 * PI) - PI);
                        if obj_len < center_len
                            && angle_diff < PI / 4.0
                            && center_len > 0.0
                            && obj_len > 0.0
                        {
                            // Compute perpendicular vectors
                            let perp_x1 = -center_y;
                            let perp_y1 = center_x;
                            let perp_x2 = center_y;
                            let perp_y2 = -center_x;

                            // Choose the one farther from object
                            let dot1 = perp_x1 * obj_x + perp_y1 * obj_y;
                            let dot2 = perp_x2 * obj_x + perp_y2 * obj_y;
                            let (perp_x, perp_y) = if dot1.abs() > dot2.abs() {
                                (perp_x1, perp_y1)
                            } else {
                                (perp_x2, perp_y2)
                            };

                            // Normalize perp
                            let perp_len = sqrtf(perp_x * perp_x + perp_y * perp_y);
                            let norm_perp_x = if perp_len > 0.0 {
                                perp_x / perp_len
                            } else {
                                0.0
                            };
                            let norm_perp_y = if perp_len > 0.0 {
                                perp_y / perp_len
                            } else {
                                0.0
                            };

                            // Blend
                            let k = 0.5; // Bias factor
                            let blended_x = center_x + k * norm_perp_x * center_len;
                            let blended_y = center_y + k * norm_perp_y * center_len;
                            ((blended_x, blended_y), 1.0)
                        } else {
                            // Not aligned, use center directly
                            ((center_x, center_y), 1.0)
                        }
                    } else {
                        // Towards object
                        ((obj_x, obj_y), 2.0)
                    };
                    let target_len = sqrtf(target_x * target_x + target_y * target_y);
                    let intended_speed = if target_len > 0.0 {
                        speed_suggestion
                    } else {
                        0.0
                    };
                    let intended_x = if target_len > 0.0 {
                        target_x / target_len * intended_speed
                    } else {
                        0.0
                    };
                    let intended_y = if target_len > 0.0 {
                        target_y / target_len * intended_speed
                    } else {
                        0.0
                    };

                    // Cancel unwanted velocity
                    // NOTE: This doesn't work properly so it is commented out
                    /*let vel_x = self.latest_velocity.0;
                    let vel_y = self.latest_velocity.1;
                    let vel_mag = sqrtf(vel_x * vel_x + vel_y * vel_y);
                    if vel_mag > 0.0 {
                        let intended_dot_vel = intended_x * vel_x + intended_y * vel_y;
                        let proj = intended_dot_vel / (vel_mag * intended_speed);
                        let unwanted_vel_x = vel_x - proj * intended_x;
                        let unwanted_vel_y = vel_y - proj * intended_y;
                        let gain = 0.5;
                        movement_x = intended_x - gain * unwanted_vel_x / vel_mag;
                        movement_y = intended_y - gain * unwanted_vel_y / vel_mag;
                        // Re-normalize
                        let new_len = sqrtf(movement_x * movement_x + movement_y * movement_y);
                        if new_len > 0.0 {
                            movement_x /= new_len;
                            movement_y /= new_len;
                        }
                    } else*/
                    {
                        movement_x = intended_x;
                        movement_y = intended_y;
                    }
                }
            }

            self.motor_control_plan = Some(MotorControlPlan {
                timestamp: ts,
                rotation_speed,
                movement_x,
                movement_y,
            });
        }

        pub fn step(
            &mut self,
            publisher: Option<
                &mut embassy_sync::pubsub::Publisher<CriticalSectionRawMutex, InputEvent, 32, 2, 6>,
            >,
        ) {
            if let Some(last_ts) = self.detector.last_ts {
                if last_ts - self.last_planner_ts >= 100_000 {
                    self.last_planner_ts = last_ts;

                    let result = self.detector.detect_objects(false);
                    self.latest_closest_wall = result.closest_wall;
                    self.latest_open_space = result.open_space;
                    self.latest_object_pos = result.object_pos;
                    self.latest_wall_distances = result.wall_distances;
                    self.latest_position = result.position;
                    self.latest_velocity = result.velocity;
                    if LOG_DETECTION {
                        println!(
                            "Detected: Wall {:?}, Open {:?}, Object {:?} (bins={})",
                            result.closest_wall,
                            result.open_space,
                            result.object_pos,
                            self.detector.bin_count()
                        );
                    }

                    if self.autonomous_enabled {
                        self.autonomous_update(last_ts);
                    } else {
                        self.autonomous_start_ts = None;
                    }

                    if let Some(ref mut plan) = self.motor_control_plan {
                        let dt = if let Some(last) = self.last_rpm_update_ts {
                            ((last_ts - last) as f32) / 1_000_000.0
                        } else {
                            0.0
                        };
                        // Allow MAX_RPM_RAMP_RATE in change in RPM Tolerate
                        // negative dt (sometimes happens)
                        let max_rpm_delta = MAX_RPM_RAMP_RATE * dt.max(0.0);
                        let target = plan.rotation_speed;
                        self.current_rotation_speed += (target - self.current_rotation_speed)
                            .clamp(-max_rpm_delta, max_rpm_delta);

                        // Allow initial jump in RPM within RPM_INITIAL_JUMP
                        let jump_to_if_needed = target.max(-RPM_INITIAL_JUMP).min(RPM_INITIAL_JUMP);
                        if jump_to_if_needed.signum() != self.current_rotation_speed.signum()
                            || (self.current_rotation_speed.abs() < RPM_INITIAL_JUMP
                                && target.abs() > self.current_rotation_speed.abs())
                        {
                            self.current_rotation_speed = jump_to_if_needed;
                        }

                        plan.rotation_speed = self.current_rotation_speed;
                    }
                    self.last_rpm_update_ts = Some(last_ts);

                    let plan = if let Some(ref p) = self.motor_control_plan {
                        p.clone()
                    } else {
                        MotorControlPlan {
                            timestamp: last_ts,
                            rotation_speed: 0.0,
                            movement_x: 0.0,
                            movement_y: 0.0,
                        }
                    };
                    if let Some(publisher) = publisher {
                        publisher.publish_immediate(InputEvent::Planner(
                            last_ts,
                            plan,
                            result.closest_wall,
                            result.open_space,
                            result.object_pos,
                            self.detector.theta,
                            self.detector.rpm,
                        ));
                    }
                }
            }
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
                        buf.push(1); // Flag 1
                        buf.extend_from_slice(&pl.to_le_bytes());
                    }
                    None => {
                        buf.push(0); // Flag 0
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
            InputEvent::Planner(ts, plan, cw, os, op, theta, rpm) => {
                let tag = (5u16 ^ TAG_XOR).to_le_bytes();
                buf.extend_from_slice(&tag);
                buf.extend_from_slice(&ts.to_le_bytes());
                buf.extend_from_slice(&plan.rotation_speed.to_le_bytes());
                buf.extend_from_slice(&plan.movement_x.to_le_bytes());
                buf.extend_from_slice(&plan.movement_y.to_le_bytes());
                buf.extend_from_slice(&cw.0.to_le_bytes());
                buf.extend_from_slice(&cw.1.to_le_bytes());
                buf.extend_from_slice(&os.0.to_le_bytes());
                buf.extend_from_slice(&os.1.to_le_bytes());
                buf.extend_from_slice(&op.0.to_le_bytes());
                buf.extend_from_slice(&op.1.to_le_bytes());
                buf.extend_from_slice(&theta.to_le_bytes());
                buf.extend_from_slice(&rpm.to_le_bytes());
            }
        }
        buf
    }

    pub const MODULATION_AMPLITUDE: f32 = 250.0; // RPM

    pub struct MotorModulator {
        last_ts: u64,
        theta: f32,
        rpm: f32,
        pub mcp: Option<MotorControlPlan>,
        flipped: bool,
    }

    impl MotorModulator {
        pub fn new() -> Self {
            Self {
                last_ts: 0,
                theta: 0.0,
                rpm: 0.0,
                mcp: None,
                flipped: false,
            }
        }

        pub fn sync(&mut self, ts: u64, theta: f32, plan: MotorControlPlan, flipped: bool) {
            self.last_ts = ts;
            self.theta = theta;
            self.mcp = Some(plan);
            self.flipped = flipped;
        }

        pub fn step(&mut self, ts: u64) -> (f32, f32) {
            let base_rpm = self.mcp.as_ref().map_or(0.0, |p| p.rotation_speed);

            // Cast to i64 so that ts sometimes going backwards is fine
            let dt = (ts as i64 - self.last_ts as i64) as f32 / 1_000_000.0;
            self.theta += base_rpm / 60.0 * 2.0 * PI * dt;
            self.theta = rem_euclid_f32(self.theta, 2.0 * PI);
            self.last_ts = ts;

            if let Some(plan) = self.mcp.as_ref() {
                // Cast to i64 so that ts sometimes going backwards is fine
                if ts as i64 - plan.timestamp as i64 > 500_000 {
                    self.mcp = None;
                }
            }

            let target_rotation_speed = self.mcp.as_ref().map_or(0.0, |p| p.rotation_speed);
            let target_movement_x = self.mcp.as_ref().map_or(0.0, |p| p.movement_x);
            let target_movement_y = self.mcp.as_ref().map_or(0.0, |p| p.movement_y);

            let mag = sqrtf(
                target_movement_x * target_movement_x + target_movement_y * target_movement_y,
            );
            if mag == 0.0 {
                return (base_rpm, base_rpm);
            }

            let phase = atan2f(target_movement_y, target_movement_x);

            let (left_rpm, right_rpm) = if self.flipped {
                // Swap motors and negate modulation when flipped
                let mod_half = -MODULATION_AMPLITUDE * mag * cosf(self.theta - phase);
                (base_rpm - mod_half, base_rpm + mod_half)
            } else {
                let mod_half = MODULATION_AMPLITUDE * mag * cosf(self.theta - phase);
                (base_rpm + mod_half, base_rpm - mod_half)
            };
            (left_rpm, right_rpm)
        }
    }
}
