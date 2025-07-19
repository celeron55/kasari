// algorithm.rs
#![no_std]

use crate::shared::rem_euclid_f32;
use arrayvec::ArrayVec;
use core::cmp::Ordering;
use core::f32::consts::PI;
use libm::{atan2f, cosf, fabsf, sinf, sqrtf};

pub const NUM_BINS: usize = 90; // 4° per bin
pub const BIN_ANGLE_STEP: f32 = 2.0 * PI / NUM_BINS as f32;
const MAX_POINTS_PER_UPDATE: usize = 4;
const CALIBRATION_COUNT: usize = 10;
const CALIBRATION_MIN_G: f32 = -8.0;
const CALIBRATION_MAX_G: f32 = 8.0;

pub struct ObjectDetector {
    pub theta: f32,
    pub rpm: f32,
    pub last_ts: Option<u64>,
    pub bins_dist: [f32; NUM_BINS], // dist per bin, INF if invalid
    pub last_xys: ArrayVec<(f32, f32), MAX_POINTS_PER_UPDATE>,
    accel_offset: f32,
    calibration_samples: ArrayVec<f32, CALIBRATION_COUNT>,
    calibration_done: bool,
    smoothed_accel_y: f32,
}

impl ObjectDetector {
    pub fn new() -> Self {
        let mut bins_dist = [f32::INFINITY; NUM_BINS];
        Self {
            theta: 0.0,
            rpm: 0.0,
            last_ts: None,
            bins_dist,
            last_xys: ArrayVec::new(),
            accel_offset: 0.0,
            calibration_samples: ArrayVec::new(),
            calibration_done: false,
            smoothed_accel_y: 0.0,
        }
    }

    fn accel_to_rpm(&self, accel_g: f32) -> f32 {
        let g = 9.81;
        let r = 0.0145;
        let a_calibrated = accel_g - self.accel_offset;
        let a = a_calibrated * g;
        let mut omega = sqrtf(fabsf(a) / r);
        if a < 0.0 {
            omega = -omega;
        }
        (omega * 60.0) / (2.0 * PI)
    }

    pub fn update(&mut self, event: &super::kasari::InputEvent) {
        use super::kasari::InputEvent::*;

        let ts = match event {
            Accelerometer(ts, _, _) | Lidar(ts, _, _, _, _) => *ts,
            _ => return,
        };

        if self.last_ts.is_none() {
            self.last_ts = Some(ts);
            return;
        }

        let last_ts = self.last_ts.unwrap();
        if ts < last_ts {
            return;
        }
        let dt = (ts - last_ts) as f32 / 1_000_000.0;

        self.theta += (self.rpm / 60.0) * 2.0 * PI * dt;
        self.theta = rem_euclid_f32(self.theta, 2.0 * PI);

        self.last_ts = Some(ts);

        match event {
            Accelerometer(_, ay, _) => {
                let raw_accel_y = *ay;
                self.smoothed_accel_y = 0.2 * raw_accel_y + 0.8 * self.smoothed_accel_y;
                let accel_y = self.smoothed_accel_y;
                if !self.calibration_done {
                    if CALIBRATION_MIN_G <= raw_accel_y && raw_accel_y <= CALIBRATION_MAX_G {
                        if self.calibration_samples.len() < CALIBRATION_COUNT {
                            self.calibration_samples.push(raw_accel_y);
                        }
                    }
                    if self.calibration_samples.len() >= CALIBRATION_COUNT {
                        self.accel_offset = self.calibration_samples.iter().sum::<f32>()
                            / self.calibration_samples.len() as f32;
                        self.calibration_done = true;
                    }
                }
                if self.calibration_done {
                    self.rpm = self.accel_to_rpm(accel_y);
                }
            }
            Lidar(_, d1, d2, d3, d4) => {
                let distances = [*d1, *d2, *d3, *d4];
                let delta_theta = if self.rpm != 0.0 {
                    0.00167 * ((self.rpm / 60.0) * 2.0 * PI)
                } else {
                    0.0
                };
                let step_theta = delta_theta / distances.len() as f32;

                self.last_xys.clear();

                for (i, &d) in distances.iter().enumerate() {
                    if 50.0 < d && d < 1600.0 {
                        let angle = rem_euclid_f32(
                            self.theta - ((distances.len() as f32 - i as f32 - 1.0) * step_theta),
                            2.0 * PI,
                        );
                        let x = d * cosf(angle);
                        let y = d * sinf(angle);
                        self.last_xys.push((x, y));
                        let bin_idx = ((angle / BIN_ANGLE_STEP) as usize) % NUM_BINS;
                        if self.bins_dist[bin_idx].is_finite() {
                            self.bins_dist[bin_idx] = (self.bins_dist[bin_idx] + d) / 2.0;
                        // Average if existing
                        } else {
                            self.bins_dist[bin_idx] = d;
                        }
                    }
                }
            }
            _ => {}
        }
    }

    pub fn bin_count(&self) -> usize {
        self.bins_dist.iter().filter(|&&d| d.is_finite()).count()
    }

    pub fn detect_objects(&self, debug: bool) -> ((f32, f32), (f32, f32), (f32, f32)) {
        let n = NUM_BINS;
        let wall_window_size = 5.max(n / 12).min(15); // Reduced max

        // Averages (instead of medians) for windows
        let mut avgs: [f32; NUM_BINS] = [0.0; NUM_BINS];
        let mut min_avg = f32::INFINITY;
        let mut max_avg = f32::NEG_INFINITY;
        let mut min_idx = 0;
        let mut max_idx = 0;

        for i in 0..n {
            let mut sum = 0.0;
            let mut count = 0.0;
            for j in 0..wall_window_size {
                let d = self.bins_dist[(i + j) % n];
                if d.is_finite() {
                    sum += d;
                    count += 1.0;
                }
            }
            let avg = if count > 0.0 {
                sum / count
            } else {
                f32::INFINITY
            };
            avgs[i] = avg;

            if 120.0 <= avg && avg <= 1500.0 {
                if avg < min_avg {
                    min_avg = avg;
                    min_idx = i;
                }
                if avg > max_avg {
                    max_avg = avg;
                    max_idx = i;
                }
            }
        }

        // Prefix for avgs
        let mut prefix_avgs = [0.0; NUM_BINS + 1];
        for i in 1..=n {
            prefix_avgs[i] = prefix_avgs[i - 1] + avgs[i - 1];
        }

        // Large changes only
        let mut large_changes: ArrayVec<(f32, usize), 32> = ArrayVec::new();
        for i in 0..n {
            let next_i = (i + 1) % n;
            let diff = fabsf(avgs[next_i] - avgs[i]);
            if diff > 80.0 {
                large_changes.push((diff, i));
            }
        }

        // Object: Simplified dip scan + paired on large_changes
        let mut best_score = f32::NEG_INFINITY;
        let mut best_start = 0;
        let mut best_end = 0;
        let mut best_is_single = false;

        // Paired on limited changes
        for i in 0..large_changes.len() {
            for j in (i + 1)..large_changes.len() {
                let mut idx1 = large_changes[i].1;
                let mut idx2 = large_changes[j].1;
                let (min_idx, max_idx) = if idx1 < idx2 {
                    (idx1, idx2)
                } else {
                    (idx2, idx1)
                };
                let diff = max_idx - min_idx;
                let wrap_diff = n - diff;
                let is_wrap = wrap_diff < diff;
                let effective_diff = if is_wrap { wrap_diff } else { diff };
                if effective_diff < 4 || effective_diff > 30 {
                    continue;
                }
                let middle_count = (effective_diff - 1) as f32;
                let sum_middle = if !is_wrap {
                    prefix_avgs[max_idx] - prefix_avgs[min_idx + 1]
                } else {
                    (prefix_avgs[n] - prefix_avgs[min_idx + 1]) + prefix_avgs[max_idx]
                };
                let avg_middle = sum_middle / middle_count;
                let start_avg = avgs[idx1];
                let end_avg = avgs[idx2];
                if avg_middle < start_avg.min(end_avg) - 100.0 {
                    let depth = start_avg.min(end_avg) - avg_middle;
                    let union_len = (wall_window_size + effective_diff - 2) as f32;
                    let score = depth * union_len.min(3.0);
                    if score > best_score {
                        best_score = score;
                        best_start = idx1;
                        best_end = idx2;
                        best_is_single = false;
                    }
                }
            }
        }

        // Single
        for &(dist_diff, mut idx) in &large_changes {
            if dist_diff < 80.0 {
                continue;
            }
            let mut local_idx2 = (idx + 1) % n;
            let mut local_idx1 = idx;
            let mut local_depth = avgs[local_idx2] - avgs[local_idx1];
            let mut local_is_flipped = false;
            if local_depth < -80.0 {
                local_depth = -local_depth;
                local_is_flipped = true;
                core::mem::swap(&mut local_idx1, &mut local_idx2);
            } else if local_depth < 80.0 {
                continue;
            }
            let avg_middle_dist = avgs[local_idx1];
            let union_len = wall_window_size as f32;
            let score = local_depth * union_len.min(3.0);
            if local_depth > 80.0 && 120.0 <= avg_middle_dist && avg_middle_dist <= 1200.0 {
                if score > best_score {
                    best_score = score;
                    best_start = local_idx1;
                    best_end = local_idx2;
                    best_is_single = true;
                }
            }
        }

        // Fallback
        let mut max_protrusion = f32::NEG_INFINITY;
        let mut best_fallback_idx = 0;
        if best_score == f32::NEG_INFINITY {
            for i in 0..n {
                let dist = self.bins_dist[i];
                if !dist.is_finite() {
                    continue;
                }
                let mut sum_neighbor = 0.0;
                let mut neighbor_count = 0.0;
                for offset in 1..=3 {
                    let left = (n + i - offset) % n;
                    let right = (i + offset) % n;
                    if self.bins_dist[left].is_finite() {
                        sum_neighbor += self.bins_dist[left];
                        neighbor_count += 1.0;
                    }
                    if self.bins_dist[right].is_finite() {
                        sum_neighbor += self.bins_dist[right];
                        neighbor_count += 1.0;
                    }
                }
                if neighbor_count < 2.0 {
                    continue;
                }
                let avg_neighbor = sum_neighbor / neighbor_count;
                let protrusion = avg_neighbor - dist;
                if protrusion > 50.0 && 120.0 <= dist && dist <= 1200.0 {
                    if protrusion > max_protrusion {
                        max_protrusion = protrusion;
                        best_fallback_idx = i;
                    }
                }
            }
        }

        // Precompute bin cos/sin for efficiency
        static mut BIN_COS: [f32; NUM_BINS] = [0.0; NUM_BINS];
        static mut BIN_SIN: [f32; NUM_BINS] = [0.0; NUM_BINS];
        static INIT: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);
        if !INIT.load(core::sync::atomic::Ordering::SeqCst) {
            for i in 0..NUM_BINS {
                let angle = i as f32 * BIN_ANGLE_STEP;
                unsafe {
                    BIN_COS[i] = cosf(angle);
                    BIN_SIN[i] = sinf(angle);
                }
            }
            INIT.store(true, core::sync::atomic::Ordering::SeqCst);
        }

        // Compute positions
        let mut closest_wall = (0.0, 0.0);
        if min_avg != f32::INFINITY {
            let mut sum_x = 0.0;
            let mut sum_y = 0.0;
            let mut count = 0.0;
            for j in 0..wall_window_size {
                let bin = (min_idx + j) % n;
                let d = self.bins_dist[bin];
                if d.is_finite() {
                    sum_x += d * unsafe { BIN_COS[bin] };
                    sum_y += d * unsafe { BIN_SIN[bin] };
                    count += 1.0;
                }
            }
            if count > 0.0 {
                closest_wall = (sum_x / count, sum_y / count);
            }
        }

        let mut open_space = (0.0, 0.0);
        if max_avg != f32::NEG_INFINITY {
            let mut sum_x = 0.0;
            let mut sum_y = 0.0;
            let mut count = 0.0;
            for j in 0..wall_window_size {
                let bin = (max_idx + j) % n;
                let d = self.bins_dist[bin];
                if d.is_finite() {
                    sum_x += d * unsafe { BIN_COS[bin] };
                    sum_y += d * unsafe { BIN_SIN[bin] };
                    count += 1.0;
                }
            }
            if count > 0.0 {
                open_space = (sum_x / count, sum_y / count);
            }
        }

        let mut object_pos = (100.0, 100.0);
        if best_score != f32::NEG_INFINITY {
            let start_pos: usize;
            let num_points: usize;
            if best_is_single {
                start_pos = best_start % n;
                num_points = wall_window_size;
            } else {
                start_pos = (best_start + 1) % n;
                let diff = if best_end > best_start {
                    best_end - best_start
                } else {
                    (best_end + n) - best_start
                };
                num_points = (diff + wall_window_size - 2) as usize;
            }
            let mut sum_x = 0.0;
            let mut sum_y = 0.0;
            let mut count = 0.0;
            for k in 0..num_points {
                let bin = (start_pos + k) % n;
                let d = self.bins_dist[bin];
                if d.is_finite() {
                    sum_x += d * unsafe { BIN_COS[bin] };
                    sum_y += d * unsafe { BIN_SIN[bin] };
                    count += 1.0;
                }
            }
            if count > 0.0 {
                object_pos = (sum_x / count, sum_y / count);
            }
        } else if max_protrusion != f32::NEG_INFINITY {
            let d = self.bins_dist[best_fallback_idx];
            if d.is_finite() {
                object_pos = (
                    d * unsafe { BIN_COS[best_fallback_idx] },
                    d * unsafe { BIN_SIN[best_fallback_idx] },
                );
            }
        }

        (closest_wall, open_space, object_pos)
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }
}
