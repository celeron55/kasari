// algorithm.rs
#![no_std]

#[cfg(target_os = "none")]
use alloc::vec;
#[cfg(target_os = "none")]
use alloc::vec::Vec;
#[cfg(not(target_os = "none"))]
use std::vec;
#[cfg(not(target_os = "none"))]
use std::vec::Vec;
use arrayvec::ArrayVec;
use core::cmp::Ordering;
use core::f32::consts::PI;
use libm::{atan2f, cosf, fabsf, sinf, sqrtf};
use crate::shared::rem_euclid_f32;

const MAX_POINTS_CAP: usize = 256;
const MAX_POINTS_PER_UPDATE: usize = 4;
const POINTS_PRUNE_LENGTH: usize = MAX_POINTS_CAP - MAX_POINTS_PER_UPDATE;
const MAX_WINDOW_SIZE: usize = 32; // Sufficient for wall_window_size up to ~21
const CALIBRATION_COUNT: usize = 10;
const CALIBRATION_MIN_G: f32 = -8.0;
const CALIBRATION_MAX_G: f32 = 8.0;

pub struct ObjectDetector {
    pub theta: f32,
    pub rpm: f32,
    pub last_ts: Option<u64>,
    pub points: ArrayVec<(f32, f32, u64), MAX_POINTS_CAP>,
    pub last_lidar_theta: f32,
    accel_offset: f32,
    calibration_samples: ArrayVec<f32, CALIBRATION_COUNT>,
    calibration_done: bool,
    smoothed_accel_y: f32,
}

impl ObjectDetector {
    pub fn new() -> Self {
        Self {
            theta: 0.0,
            rpm: 0.0,
            last_ts: None,
            points: ArrayVec::new(),
            last_lidar_theta: 0.0,
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

    fn prune_points(&mut self, current_ts: u64, fade_time_us: u64) {
        self.points.retain(|pt| current_ts >= pt.2 && current_ts - pt.2 <= fade_time_us);
        if self.points.len() > POINTS_PRUNE_LENGTH {
            self.points.drain(0..self.points.len() - POINTS_PRUNE_LENGTH);
        }
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

        let fade_time_us = if self.rpm == 0.0 {
            100_000
        } else {
            ((1.5 * 60.0 * 1_000_000.0) / self.rpm.abs()) as u64
        };
        self.prune_points(ts, fade_time_us);

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
                        self.accel_offset = self.calibration_samples.iter().sum::<f32>() / self.calibration_samples.len() as f32;
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

                let mut points_this_event: ArrayVec<(f32, f32, u64), MAX_POINTS_PER_UPDATE> = ArrayVec::new();
                for (i, &d) in distances.iter().enumerate() {
                    if 50.0 < d && d < 1600.0 {
                        let angle = self.theta - ((distances.len() as f32 - i as f32 - 1.0) * step_theta);
                        let x = d * cosf(angle);
                        let y = d * sinf(angle);
                        points_this_event.push((x, y, ts));
                    }
                }

                let angular_tolerance = 0.01;
                for new_pt in &points_this_event {
                    let a = rem_euclid_f32(atan2f(new_pt.1, new_pt.0), 2.0 * PI);
                    self.points.retain(|pt| {
                        let pa = rem_euclid_f32(atan2f(pt.1, pt.0), 2.0 * PI);
                        let d = rem_euclid_f32(pa - a, 2.0 * PI);
                        let min_diff = d.min(2.0 * PI - d);
                        min_diff > angular_tolerance
                    });
                }
                self.points.extend(points_this_event);
                self.last_lidar_theta = self.theta;
            }
            _ => {}
        }
    }

    pub fn detect_objects(&self, debug: bool) -> ((f32, f32), (f32, f32), (f32, f32)) {
        if self.points.len() < 10 {
            return ((0.0, 0.0), (0.0, 0.0), (100.0, 100.0));
        }

        // Sorted indices by angle
        let mut sorted_indices: ArrayVec<usize, MAX_POINTS_CAP> = (0..self.points.len()).collect();
        sorted_indices.sort_by(|&a, &b| {
            let angle_a = rem_euclid_f32(atan2f(self.points[a].1, self.points[a].0), 2.0 * PI);
            let angle_b = rem_euclid_f32(atan2f(self.points[b].1, self.points[b].0), 2.0 * PI);
            angle_a.partial_cmp(&angle_b).unwrap_or(Ordering::Equal)
        });

        let n = self.points.len();
        let wall_window_size = 5.max(n / 12).min(MAX_WINDOW_SIZE);

        // Medians for each window
        let mut medians: ArrayVec<f32, MAX_POINTS_CAP> = ArrayVec::new();
        let mut temp_dists: ArrayVec<f32, MAX_WINDOW_SIZE> = ArrayVec::new();

        let mut min_median = f32::INFINITY;
        let mut max_median = f32::NEG_INFINITY;
        let mut min_median_idx = 0;
        let mut max_median_idx = 0;

        for i in 0..n {
            temp_dists.clear();
            for j in 0..wall_window_size {
                let idx = sorted_indices[(i + j) % n];
                let dist = sqrtf(self.points[idx].0 * self.points[idx].0 + self.points[idx].1 * self.points[idx].1);
                temp_dists.push(dist);
            }
            temp_dists.sort_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal));
            let median = temp_dists[temp_dists.len() / 2];
            medians.push(median);

            if 120.0 <= median && median <= 1500.0 {
                if median < min_median {
                    min_median = median;
                    min_median_idx = i;
                }
                if median > max_median {
                    max_median = median;
                    max_median_idx = i;
                }
            }
        }

        // Compute prefix sums for medians
        let mut prefix_medians: ArrayVec<f32, {MAX_POINTS_CAP + 1}> = ArrayVec::new();
        prefix_medians.push(0.0);
        for &med in &medians {
            prefix_medians.push(prefix_medians.last().unwrap() + med);
        }

        // Distance changes
        let mut distance_changes: ArrayVec<(f32, usize), MAX_POINTS_CAP> = ArrayVec::new();
        for i in 0..n - 1 {
            let dist_diff = fabsf(medians[i + 1] - medians[i]);
            distance_changes.push((dist_diff, i));
        }
        // Add wrap-around change
        let wrap_diff = fabsf(medians[0] - medians[n - 1]);
        distance_changes.push((wrap_diff, n - 1));
        distance_changes.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(Ordering::Equal));

        // Object detection
        let mut best_score = f32::NEG_INFINITY;
        let mut best_idx1 = 0;
        let mut best_idx2 = 0;
        let mut best_is_single = false;

        // Paired
        for i in 0..distance_changes.len() {
            for j in (i + 1)..distance_changes.len() {
                let mut idx1 = distance_changes[i].1;
                let mut idx2 = distance_changes[j].1;
                if idx1 > idx2 {
                    core::mem::swap(&mut idx1, &mut idx2);
                }
                if idx2 - idx1 < 4 || idx2 - idx1 > 30 {
                    continue;
                }
                let middle_count = (idx2 - idx1 - 1) as f32;
                if middle_count < 1.0 {
                    continue;
                }
                let sum_middle = prefix_medians[idx2] - prefix_medians[idx1 + 1];
                let avg_middle_dist = sum_middle / middle_count;
                let start_dist = medians[idx1];
                let end_dist = medians[idx2];
                if avg_middle_dist < start_dist.min(end_dist) - 100.0 {
                    let depth = start_dist.min(end_dist) - avg_middle_dist;
                    let union_len = (wall_window_size + (idx2 - idx1 - 2)) as f32; // Approximate union
                    let score = depth * union_len.min(3.0);
                    if depth > 100.0 && 120.0 <= avg_middle_dist && avg_middle_dist <= 1200.0 {
                        if score > best_score {
                            best_score = score;
                            best_idx1 = idx1;
                            best_idx2 = idx2;
                            best_is_single = false;
                        }
                    }
                }
            }
        }

        // Single
        for &(dist_diff, mut idx) in &distance_changes {
            if dist_diff < 80.0 {
                continue;
            }
            let mut local_idx2 = if idx == n - 1 { 0 } else { idx + 1 };
            let mut local_idx1 = idx;
            let mut local_depth = medians[local_idx2] - medians[local_idx1];
            let mut local_is_flipped = false;
            if local_depth < -80.0 {
                local_depth = -local_depth;
                local_is_flipped = true;
                core::mem::swap(&mut local_idx1, &mut local_idx2);
            } else if local_depth < 80.0 {
                continue;
            }
            let avg_middle_dist = medians[local_idx1];
            let union_len = wall_window_size as f32;
            let score = local_depth * union_len.min(3.0);
            if local_depth > 80.0 && 120.0 <= avg_middle_dist && avg_middle_dist <= 1200.0 {
                if score > best_score {
                    best_score = score;
                    best_idx1 = local_idx1;
                    best_idx2 = local_idx2;
                    best_is_single = true;
                }
            }
        }

        // Fallback
        let mut max_protrusion = f32::NEG_INFINITY;
        let mut best_fallback_idx = 0;
        if best_score == f32::NEG_INFINITY {
            for i in 0..n {
                let dist = sqrtf(self.points[sorted_indices[i]].0.powi(2) + self.points[sorted_indices[i]].1.powi(2));
                let mut neighbor_indices: ArrayVec<usize, 6> = ArrayVec::new(); // Up to 3 left + 3 right
                for offset in 1..=3 {
                    if i >= offset {
                        neighbor_indices.push((i - offset) % n);
                    } else {
                        neighbor_indices.push((n + i - offset) % n);
                    }
                    neighbor_indices.push((i + offset) % n);
                }
                if neighbor_indices.len() < 2 {
                    continue;
                }
                let mut sum_neighbor_dist = 0.0;
                for &ni in &neighbor_indices {
                    sum_neighbor_dist += medians[ni];
                }
                let avg_neighbor_dist = sum_neighbor_dist / neighbor_indices.len() as f32;
                let protrusion = avg_neighbor_dist - dist;
                if protrusion > 50.0 && 120.0 <= dist && dist <= 1200.0 {
                    if protrusion > max_protrusion {
                        max_protrusion = protrusion;
                        best_fallback_idx = i;
                    }
                }
            }
        }

        // Compute positions
        let mut closest_wall = (0.0, 0.0);
        if min_median != f32::INFINITY {
            let mut sum_x = 0.0;
            let mut sum_y = 0.0;
            for j in 0..wall_window_size {
                let idx = sorted_indices[(min_median_idx + j) % n];
                sum_x += self.points[idx].0;
                sum_y += self.points[idx].1;
            }
            closest_wall = (sum_x / wall_window_size as f32, sum_y / wall_window_size as f32);
        }

        let mut open_space = (0.0, 0.0);
        if max_median != f32::NEG_INFINITY {
            let mut sum_x = 0.0;
            let mut sum_y = 0.0;
            for j in 0..wall_window_size {
                let idx = sorted_indices[(max_median_idx + j) % n];
                sum_x += self.points[idx].0;
                sum_y += self.points[idx].1;
            }
            open_space = (sum_x / wall_window_size as f32, sum_y / wall_window_size as f32);
        }

        let mut object_pos = (100.0, 100.0);
        if best_score != f32::NEG_INFINITY {
            let start_pos: usize;
            let num_points: usize;
            if best_is_single {
                start_pos = best_idx1 % n;
                num_points = wall_window_size;
            } else {
                start_pos = (best_idx1 + 1) % n;
                num_points = wall_window_size + (best_idx2 - best_idx1 - 2);
            }
            let mut sum_x = 0.0;
            let mut sum_y = 0.0;
            for k in 0..num_points {
                let idx = sorted_indices[(start_pos + k) % n];
                sum_x += self.points[idx].0;
                sum_y += self.points[idx].1;
            }
            object_pos = (sum_x / num_points as f32, sum_y / num_points as f32);
        } else if max_protrusion != f32::NEG_INFINITY {
            object_pos = (self.points[sorted_indices[best_fallback_idx]].0, self.points[sorted_indices[best_fallback_idx]].1);
        }

        (closest_wall, open_space, object_pos)
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }
}
