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

pub struct ObjectDetector {
    pub theta: f32,
    pub rpm: f32,
    pub last_ts: Option<u64>,
    pub points: ArrayVec<(f32, f32, u64), MAX_POINTS_CAP>,
    pub last_lidar_theta: f32,
    accel_offset: f32,
    calibration_samples: Vec<f32>,
    calibration_done: bool,
    smoothed_accel_y: f32,
}

const CALIBRATION_COUNT: usize = 10;
const CALIBRATION_MIN_G: f32 = -8.0;
const CALIBRATION_MAX_G: f32 = 8.0;

impl ObjectDetector {
    pub fn new() -> Self {
        Self {
            theta: 0.0,
            rpm: 0.0,
            last_ts: None,
            points: ArrayVec::new(),
            last_lidar_theta: 0.0,
            accel_offset: 0.0,
            calibration_samples: Vec::new(),
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
                        self.calibration_samples.push(raw_accel_y);
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

                let mut points_this_event: Vec<(f32, f32, u64)> = Vec::new();
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

        let mut points_with_data: Vec<(f32, f32, f32, f32)> = self.points.iter().map(|&(x, y, _)| {
            let dist = sqrtf(x * x + y * y);
            let angle = rem_euclid_f32(atan2f(y, x), 2.0 * PI);
            (x, y, dist, angle)
        }).collect();
        points_with_data.sort_by(|a, b| a.3.partial_cmp(&b.3).unwrap_or(Ordering::Equal));

        let wall_window_size = 5.max(points_with_data.len() / 12);
        let mut windows: Vec<(f32, Vec<(f32, f32)>)> = Vec::new();
        let mut min_dist = f32::INFINITY;
        let mut max_dist = f32::NEG_INFINITY;
        let mut closest_wall_points: Vec<(f32, f32)> = Vec::new();
        let mut open_space_points: Vec<(f32, f32)> = Vec::new();

        for i in 0..points_with_data.len() {
            let mut window_points: Vec<(f32, f32)> = Vec::with_capacity(wall_window_size);
            let mut distances: Vec<f32> = Vec::with_capacity(wall_window_size);
            for j in 0..wall_window_size {
                let idx = (i + j) % points_with_data.len();
                let p = points_with_data[idx];
                window_points.push((p.0, p.1));
                distances.push(p.2);
            }
            distances.sort_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal));
            let median_dist = distances[distances.len() / 2];
            windows.push((median_dist, window_points.clone()));

            if 120.0 <= median_dist && median_dist <= 1500.0 {
                if median_dist < min_dist {
                    min_dist = median_dist;
                    closest_wall_points = window_points.clone();
                }
                if median_dist > max_dist {
                    max_dist = median_dist;
                    open_space_points = window_points.clone();
                }
            }
        }

        let mut distance_changes: Vec<(f32, usize)> = Vec::new();
        for i in 0..windows.len() - 1 {
            let dist_diff = fabsf(windows[i + 1].0 - windows[i].0);
            distance_changes.push((dist_diff, i));
        }
        distance_changes.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(Ordering::Equal));

        let mut best_score = f32::NEG_INFINITY;
        let mut best_region: Option<(usize, usize, Vec<(f32, f32)>, f32)> = None;
        let mut object_points: Vec<(f32, f32)> = Vec::new();

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
                let middle_dists: Vec<f32> = (idx1 + 1..idx2).map(|k| windows[k].0).collect();
                if middle_dists.is_empty() {
                    continue;
                }
                let avg_middle_dist = middle_dists.iter().sum::<f32>() / middle_dists.len() as f32;
                let start_dist = windows[idx1].0;
                let end_dist = windows[idx2].0;
                if avg_middle_dist < start_dist.min(end_dist) - 100.0 {
                    let depth = start_dist.min(end_dist) - avg_middle_dist;
                    let mut region_points: Vec<(f32, f32)> = Vec::new();
                    for k in idx1 + 1..idx2 {
                        region_points.extend(windows[k].1.clone());
                    }
                    if region_points.len() < 2 {
                        continue;
                    }
                    let score = depth * region_points.len().min(3) as f32;
                    if depth > 100.0 && 120.0 <= avg_middle_dist && avg_middle_dist <= 1200.0 {
                        if score > best_score {
                            best_score = score;
                            best_region = Some((idx1 + 1, idx2, region_points, avg_middle_dist));
                        }
                    }
                }
            }
        }

        // Single
        for &(dist_diff, idx) in &distance_changes {
            if dist_diff < 80.0 {
                continue;
            }
            if idx + 1 < windows.len() {
                if windows[idx + 1].0 < windows[idx].0 - 80.0 {
                    let avg_middle_dist = windows[idx + 1].0;
                    let region_points = windows[idx + 1].1.clone();
                    let depth = windows[idx].0 - avg_middle_dist;
                    let score = depth * region_points.len().min(3) as f32;
                    if depth > 80.0 && 120.0 <= avg_middle_dist && avg_middle_dist <= 1200.0 && region_points.len() >= 1 {
                        if score > best_score {
                            best_score = score;
                            best_region = Some((idx + 1, idx + 2, region_points, avg_middle_dist));
                        }
                    }
                } else if windows[idx].0 < windows[idx + 1].0 - 80.0 {
                    let avg_middle_dist = windows[idx].0;
                    let region_points = windows[idx].1.clone();
                    let depth = windows[idx + 1].0 - avg_middle_dist;
                    let score = depth * region_points.len().min(3) as f32;
                    if depth > 80.0 && 120.0 <= avg_middle_dist && avg_middle_dist <= 1200.0 && region_points.len() >= 1 {
                        if score > best_score {
                            best_score = score;
                            best_region = Some((idx, idx + 1, region_points, avg_middle_dist));
                        }
                    }
                }
            }
        }

        // Fallback
        if best_region.is_none() {
            let mut max_protrusion = f32::NEG_INFINITY;
            let mut best_window_points: Vec<(f32, f32)> = Vec::new();
            for point in &points_with_data {
                let (x, y, dist, _) = *point;
                let mut window_idx_opt: Option<usize> = None;
                for (i, (_, w_points)) in windows.iter().enumerate() {
                    for wp in w_points {
                        if fabsf(wp.0 - x) < 1e-6 && fabsf(wp.1 - y) < 1e-6 {
                            window_idx_opt = Some(i);
                            break;
                        }
                    }
                    if window_idx_opt.is_some() {
                        break;
                    }
                }
                let window_idx = match window_idx_opt {
                    Some(idx) => idx,
                    None => continue,
                };
                let mut window_indices: Vec<usize> = Vec::new();
                for i in 0..3 {
                    if window_idx > i {
                        window_indices.push(window_idx - 1 - i);
                    }
                    if window_idx < windows.len() - 1 - i {
                        window_indices.push(window_idx + 1 + i);
                    }
                }
                if window_indices.len() < 2 {
                    continue;
                }
                let avg_neighbor_dist = window_indices.iter().map(|&i| windows[i].0).sum::<f32>() / window_indices.len() as f32;
                let protrusion = avg_neighbor_dist - dist;
                if protrusion > 50.0 && 120.0 <= dist && dist <= 1200.0 {
                    if protrusion > max_protrusion {
                        max_protrusion = protrusion;
                        best_window_points = vec![(x, y)];
                    }
                }
            }
            if !best_window_points.is_empty() {
                object_points = best_window_points;
            }
        }

        if let Some(region) = best_region {
            object_points = region.2;
        }

        let mut closest_wall = (0.0, 0.0);
        if !closest_wall_points.is_empty() {
            let sum_x = closest_wall_points.iter().map(|p| p.0).sum::<f32>();
            let sum_y = closest_wall_points.iter().map(|p| p.1).sum::<f32>();
            closest_wall = (sum_x / closest_wall_points.len() as f32, sum_y / closest_wall_points.len() as f32);
        }

        let mut open_space = (0.0, 0.0);
        if !open_space_points.is_empty() {
            let sum_x = open_space_points.iter().map(|p| p.0).sum::<f32>();
            let sum_y = open_space_points.iter().map(|p| p.1).sum::<f32>();
            open_space = (sum_x / open_space_points.len() as f32, sum_y / open_space_points.len() as f32);
        }

        let mut object_pos = (100.0, 100.0);
        if !object_points.is_empty() {
            let sum_x = object_points.iter().map(|p| p.0).sum::<f32>();
            let sum_y = object_points.iter().map(|p| p.1).sum::<f32>();
            object_pos = (sum_x / object_points.len() as f32, sum_y / object_points.len() as f32);
        }

        (closest_wall, open_space, object_pos)
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }
}
