// shared/flip_detector.rs
use arrayvec::ArrayVec;
use libm::fabsf;
use num_traits::float::FloatCore;
// This makes println accessible on both ESP32 and PC targets
#[cfg(target_os = "none")]
use esp_println::println;

pub const CALIB_COUNT: usize = 50;
pub const CALIB_MIN_G: f32 = -4.0;
pub const CALIB_MAX_G: f32 = 4.0;
pub const CALIB_OFFSET_ADD: f32 = 1.0;
pub const CALIB_AY_VAR_THRESH: f32 = 0.1;
pub const MEDIAN_KERNEL: usize = 5;
pub const EMA_ALPHA: f32 = 0.01;

pub const REG_WINDOW: usize = 50;
pub const REG_MIN_AY: f32 = 2.0;
pub const REG_ALPHA: f32 = 0.1;
pub const REG_CORR_THRESH: f32 = 0.1;

pub const FLIP_LOW_THRESH: f32 = -2.0;
pub const FLIP_HYST_HIGH: f32 = 0.2;
// The behavior doesn't exactly match tools/analyze_flip.py and changing this
// value from 0.55 to 0.80 mostly fixes that
pub const FLIP_THRESH: f32 = 0.80;

pub struct FlipDetector {
    calib_az_samples: ArrayVec<f32, CALIB_COUNT>,
    calib_ay_samples: ArrayVec<f32, CALIB_COUNT>,
    calib_done: bool,
    calib_just_done: bool,
    offset: f32,
    smoothed_az: f32,
    smoothed_ay: f32,
    corrected_az: f32,
    gravity_score: f32,
    flipped: bool,
    prev_flipped: bool,
    az_buffer: ArrayVec<f32, MEDIAN_KERNEL>,
    ay_buffer: ArrayVec<f32, MEDIAN_KERNEL>,
    reg_window: ArrayVec<(f32, f32), REG_WINDOW>,
}

impl FlipDetector {
    pub fn new() -> Self {
        Self {
            calib_az_samples: ArrayVec::new(),
            calib_ay_samples: ArrayVec::new(),
            calib_done: false,
            calib_just_done: false,
            offset: 0.0,
            smoothed_az: 0.0,
            smoothed_ay: 0.0,
            corrected_az: 0.0,
            gravity_score: 0.0,
            flipped: false,
            prev_flipped: false,
            az_buffer: ArrayVec::new(),
            ay_buffer: ArrayVec::new(),
            reg_window: ArrayVec::new(),
        }
    }

    pub fn update(&mut self, raw_ay: f32, raw_az: f32) {
        // Median filter on raw_az
        self.az_buffer.push(raw_az);
        let az_input = if self.az_buffer.len() >= MEDIAN_KERNEL {
            let mut sorted = self.az_buffer.clone();
            sorted.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
            let median = sorted[MEDIAN_KERNEL / 2];
            self.az_buffer.remove(0);
            median
        } else {
            self.az_buffer.iter().sum::<f32>() / self.az_buffer.len() as f32
        };

        // Median filter on raw_ay
        self.ay_buffer.push(raw_ay);
        let ay_input = if self.ay_buffer.len() >= MEDIAN_KERNEL {
            let mut sorted = self.ay_buffer.clone();
            sorted.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
            let median = sorted[MEDIAN_KERNEL / 2];
            self.ay_buffer.remove(0);
            median
        } else {
            self.ay_buffer.iter().sum::<f32>() / self.ay_buffer.len() as f32
        };

        // EMA smoothing on median-filtered AZ and AY
        if self.smoothed_az == 0.0 {
            self.smoothed_az = az_input;
        } else {
            self.smoothed_az = EMA_ALPHA * az_input + (1.0 - EMA_ALPHA) * self.smoothed_az;
        }

        if self.smoothed_ay == 0.0 {
            self.smoothed_ay = ay_input;
        } else {
            self.smoothed_ay = EMA_ALPHA * ay_input + (1.0 - EMA_ALPHA) * self.smoothed_ay;
        }

        let az_sm = self.smoothed_az;
        let ay_sm = self.smoothed_ay;

        // Calibration
        if !self.calib_done {
            self.calib_az_samples.push(az_sm);
            self.calib_ay_samples.push(ay_sm);
            if self.calib_az_samples.len() >= CALIB_COUNT {
                // Compute AY variance
                let ay_mean = self.calib_ay_samples.iter().sum::<f32>() / CALIB_COUNT as f32;
                let ay_var = self.calib_ay_samples.iter().map(|&ay| (ay - ay_mean).powi(2)).sum::<f32>() / CALIB_COUNT as f32;
                println!("Calibration attempt: AY var={:0.3}, thresh={:0.3}", ay_var, CALIB_AY_VAR_THRESH);
                if ay_var <= CALIB_AY_VAR_THRESH {
                    // Use median for offset
                    let mut sorted_az = self.calib_az_samples.clone();
                    sorted_az.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));
                    let median_az = sorted_az[CALIB_COUNT / 2];
                    self.offset = median_az + CALIB_OFFSET_ADD;
                    self.calib_done = true;
                    self.calib_just_done = true;
                    println!("Calibration done: median_az={:0.3}, offset={:0.3}", median_az, self.offset);
                } else {
                    // Shift window (remove oldest)
                    self.calib_az_samples.remove(0);
                    self.calib_ay_samples.remove(0);
                    println!("Calibration delayed: High AY variance");
                }
            }
        }

        // Post-calib: compute corrected_az, gravity_score, and detect flip
        if self.calib_done {
            self.corrected_az = az_sm - self.offset;

            // Regression for gravity_score
            let mut update_with_b = false;
            let mut b = 0.0;
            if fabsf(ay_sm) > REG_MIN_AY {
                if self.reg_window.len() == REG_WINDOW {
                    self.reg_window.remove(0);
                }
                self.reg_window.push((ay_sm, az_sm));

                if self.reg_window.len() == REG_WINDOW {
                    let mut sum_ay = 0.0f32;
                    let mut sum_az = 0.0f32;
                    let mut sum_ay2 = 0.0f32;
                    let mut sum_ay_az = 0.0f32;
                    for &(ay, az) in self.reg_window.iter() {
                        sum_ay += ay;
                        sum_az += az;
                        sum_ay2 += ay * ay;
                        sum_ay_az += ay * az;
                    }
                    let n = REG_WINDOW as f32;
                    let denom = n * sum_ay2 - sum_ay * sum_ay;
                    if denom != 0.0 {
                        let a = (n * sum_ay_az - sum_ay * sum_az) / denom;
                        b = (sum_az - a * sum_ay) / n;
                        if fabsf(a) > REG_CORR_THRESH {
                            update_with_b = true;
                        }
                    }
                }
            } else {
                // Note: if not adding, but we still update gravity_score below
            }

            // Update gravity_score
            if update_with_b {
                if self.calib_just_done {
                    self.gravity_score = b;
                } else {
                    self.gravity_score = REG_ALPHA * b + (1.0 - REG_ALPHA) * self.gravity_score;
                }
            } else {
                if self.calib_just_done {
                    self.gravity_score = self.corrected_az;
                } else {
                    self.gravity_score = REG_ALPHA * self.corrected_az + (1.0 - REG_ALPHA) * self.gravity_score;
                }
            }
            self.calib_just_done = false;

            // Flip detection
            let score = self.corrected_az;
            if score < FLIP_LOW_THRESH {
                self.offset -= 2.0;
                self.flipped = false;
            } else if score > FLIP_HYST_HIGH && self.prev_flipped {
                self.flipped = true;
            } else if score > FLIP_THRESH {
                self.flipped = true;
            } else {
                self.flipped = false;
            }

            // Debug print every 100 samples (approximate, using az_buffer.len() as counter)
            if self.az_buffer.len() % 100 == 0 {
                println!("Sample: raw_az={:0.3}, smoothed_az={:0.3}, corrected_az={:0.3}, flipped={}", raw_az, az_sm, self.corrected_az, self.flipped);
            }

            if self.flipped != self.prev_flipped {
                println!("State change: flipped={} (corrected_az={:0.3})", self.flipped, self.corrected_az);
            }

            self.prev_flipped = self.flipped;
        }
    }

    pub fn is_flipped(&self) -> bool {
        self.flipped
    }

    pub fn is_calib_done(&self) -> bool {
        self.calib_done
    }

    pub fn get_corrected_az(&self) -> f32 {
        self.corrected_az
    }

    pub fn get_gravity_score(&self) -> f32 {
        self.gravity_score
    }
}
