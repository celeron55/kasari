use std::error::Error;
use std::fs::File;
use std::io::{self, BufRead, BufReader, Error as IoError};
use std::time::Instant;

use clap::Parser;
use eframe::egui;
use egui_plot::{Line, Plot, PlotBounds, PlotPoints};
use serde_json::Value;

mod shared;
use shared::algorithm::{BIN_ANGLE_STEP, NUM_BINS};
use shared::kasari::{InputEvent, MainLogic, MotorControlPlan};

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Log file path or '-' for stdin
    source: String,

    /// Enable debug prints
    #[arg(long)]
    debug: bool,

    /// Remove all WifiControl events and inject new ones with mode=2 every 100ms starting after first Lidar event
    #[arg(long)]
    inject_autonomous: bool,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();

    let lines: Box<dyn Iterator<Item = Result<String, IoError>>> = if args.source == "-" {
        Box::new(io::stdin().lines())
    } else {
        let file = File::open(&args.source)?;
        let reader = BufReader::new(file);
        Box::new(reader.lines())
    };

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([2000.0, 2000.0]),
        ..Default::default()
    };

    eframe::run_native(
        "Robot Simulator",
        options,
        Box::new(move |_cc| {
            Ok(Box::new(MyApp::new(
                lines,
                args.debug,
                args.inject_autonomous,
            )))
        }),
    )?;

    Ok(())
}

fn parse_event(line: &str) -> Result<InputEvent, Box<dyn Error>> {
    let v: Vec<Value> = serde_json::from_str(line)?;

    if v.is_empty() {
        return Err("Empty event".into());
    }

    let typ = v[0].as_str().ok_or("No type")?;
    let ts = v[1].as_u64().ok_or("No timestamp")?;

    match typ {
        "Lidar" => {
            if v.len() != 6 {
                return Err("Invalid Lidar event length".into());
            }
            let d1 = v[2].as_f64().ok_or("Invalid d1")? as f32;
            let d2 = v[3].as_f64().ok_or("Invalid d2")? as f32;
            let d3 = v[4].as_f64().ok_or("Invalid d3")? as f32;
            let d4 = v[5].as_f64().ok_or("Invalid d4")? as f32;
            Ok(InputEvent::Lidar(ts, d1, d2, d3, d4))
        }
        "Accelerometer" => {
            if v.len() != 4 {
                return Err("Invalid Accelerometer event length".into());
            }
            let ay = v[2].as_f64().ok_or("Invalid ay")? as f32;
            let az = v[3].as_f64().ok_or("Invalid az")? as f32;
            Ok(InputEvent::Accelerometer(ts, ay, az))
        }
        "Receiver" => {
            if v.len() != 4 {
                return Err("Invalid Receiver event length".into());
            }
            let ch = v[2].as_u64().ok_or("Invalid channel")? as u8;
            let pulse = if v[3].is_null() {
                None
            } else {
                Some(v[3].as_f64().ok_or("Invalid pulse")? as f32)
            };
            Ok(InputEvent::Receiver(ts, ch, pulse))
        }
        "Vbat" => {
            if v.len() != 3 {
                return Err("Invalid Vbat event length".into());
            }
            let voltage = v[2].as_f64().ok_or("Invalid voltage")? as f32;
            Ok(InputEvent::Vbat(ts, voltage))
        }
        "WifiControl" => {
            if v.len() != 6 {
                return Err("Invalid WifiControl event length".into());
            }
            let mode = v[2].as_u64().ok_or("Invalid mode")? as u8;
            let r = v[3].as_f64().ok_or("Invalid r")? as f32;
            let m = v[4].as_f64().ok_or("Invalid m")? as f32;
            let t = v[5].as_f64().ok_or("Invalid t")? as f32;
            Ok(InputEvent::WifiControl(ts, mode, r, m, t))
        }
        "Planner" => {
            if v.len() != 12 && v.len() != 13 {
                return Err("Invalid Planner event length".into());
            }
            let rotation_speed = v[2].as_f64().ok_or("Invalid rotation_speed")? as f32;
            let movement_x = v[3].as_f64().ok_or("Invalid movement_x")? as f32;
            let movement_y = v[4].as_f64().ok_or("Invalid movement_y")? as f32;
            let cw_x = v[5].as_f64().ok_or("Invalid cw_x")? as f32;
            let cw_y = v[6].as_f64().ok_or("Invalid cw_y")? as f32;
            let os_x = v[7].as_f64().ok_or("Invalid os_x")? as f32;
            let os_y = v[8].as_f64().ok_or("Invalid os_y")? as f32;
            let op_x = v[9].as_f64().ok_or("Invalid op_x")? as f32;
            let op_y = v[10].as_f64().ok_or("Invalid op_y")? as f32;
            let theta = v[11].as_f64().ok_or("Invalid theta")? as f32;
            let rpm = if v.len() >= 13 {
                v[12].as_f64().ok_or("Invalid rpm")? as f32
            } else {
                0.0
            };
            Ok(InputEvent::Planner(
                ts,
                MotorControlPlan {
                    timestamp: 0,
                    rotation_speed,
                    movement_x,
                    movement_y,
                },
                (cw_x, cw_y),
                (os_x, os_y),
                (op_x, op_y),
                theta,
                rpm,
            ))
        }
        _ => Err("Unknown event type".into()),
    }
}

fn get_ts(event: &InputEvent) -> u64 {
    match event {
        InputEvent::Lidar(ts, ..) => *ts,
        InputEvent::Accelerometer(ts, ..) => *ts,
        InputEvent::Receiver(ts, ..) => *ts,
        InputEvent::Vbat(ts, ..) => *ts,
        InputEvent::WifiControl(ts, ..) => *ts,
        InputEvent::Planner(ts, ..) => *ts,
    }
}

struct MyApp {
    logic: MainLogic,
    lines: Box<dyn Iterator<Item = Result<String, IoError>>>,
    next_real_event: Option<InputEvent>,
    current_event_idx: usize, // For display purposes, count processed
    virtual_elapsed: f64,
    first_ts: u64,
    mode: String,
    step_requested: bool,
    last_real: Instant,
    current_lidar_points: Vec<(f32, f32)>,
    debug: bool,
    latest_planner: Option<InputEvent>,
    show_planner_theta: bool,
    theta_offset: f32,
    inject_autonomous: bool,
    first_lidar_found: bool,
    next_inject_ts: Option<u64>,
    last_read_ts: Option<u64>,
}

impl MyApp {
    fn new(
        lines: Box<dyn Iterator<Item = Result<String, IoError>>>,
        debug: bool,
        inject_autonomous: bool,
    ) -> Self {
        Self {
            logic: MainLogic::new(),
            lines,
            next_real_event: None,
            current_event_idx: 0,
            virtual_elapsed: 0.0,
            first_ts: 0,
            mode: "play".to_string(),
            step_requested: false,
            last_real: Instant::now(),
            current_lidar_points: Vec::new(),
            debug,
            latest_planner: None,
            show_planner_theta: false,
            theta_offset: 0.0,
            inject_autonomous,
            first_lidar_found: false,
            next_inject_ts: None,
            last_read_ts: None,
        }
    }

    fn reset(&mut self) {
        self.logic = MainLogic::new();
        self.next_real_event = None;
        self.current_event_idx = 0;
        self.virtual_elapsed = 0.0;
        self.mode = "play".to_string();
        self.step_requested = false;
        self.current_lidar_points.clear();
        self.latest_planner = None;
        self.show_planner_theta = false;
        self.theta_offset = 0.0;
        self.first_lidar_found = false;
        self.next_inject_ts = None;
        self.last_read_ts = None;
        // Note: lines cannot be reset unless reopen file, but assume no reset for streaming
    }

    fn rotate_point(&self, x: f32, y: f32, angle: f32) -> (f64, f64) {
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        let new_x = x * cos_a - y * sin_a;
        let new_y = x * sin_a + y * cos_a;
        (new_x as f64, new_y as f64)
    }

    fn read_next_real(&mut self) -> bool {
        loop {
            if let Some(line_res) = self.lines.next() {
                match line_res {
                    Ok(line) => {
                        if line.trim().is_empty() {
                            continue;
                        }
                        match parse_event(&line) {
                            Ok(mut event) => {
                                if self.inject_autonomous
                                    && matches!(&event, InputEvent::WifiControl(..))
                                {
                                    continue;
                                }
                                let mut adjusted_ts = get_ts(&event);
                                if matches!(&event, InputEvent::WifiControl(..)) {
                                    if let Some(last) = self.last_read_ts {
                                        if ((adjusted_ts as i128 - last as i128).abs() > 5_000_000)
                                        {
                                            adjusted_ts = last + 1000;
                                        }
                                    }
                                }
                                // Update event ts
                                event = match event {
                                    InputEvent::Lidar(_, d1, d2, d3, d4) => {
                                        InputEvent::Lidar(adjusted_ts, d1, d2, d3, d4)
                                    }
                                    InputEvent::Accelerometer(_, ay, az) => {
                                        InputEvent::Accelerometer(adjusted_ts, ay, az)
                                    }
                                    InputEvent::Receiver(_, ch, pulse) => {
                                        InputEvent::Receiver(adjusted_ts, ch, pulse)
                                    }
                                    InputEvent::Vbat(_, voltage) => {
                                        InputEvent::Vbat(adjusted_ts, voltage)
                                    }
                                    InputEvent::WifiControl(_, mode, r, m, t) => {
                                        InputEvent::WifiControl(adjusted_ts, mode, r, m, t)
                                    }
                                    InputEvent::Planner(_, plan, cw, os, op, theta, rpm) => {
                                        InputEvent::Planner(
                                            adjusted_ts,
                                            plan,
                                            cw,
                                            os,
                                            op,
                                            theta,
                                            rpm,
                                        )
                                    }
                                };
                                self.last_read_ts = Some(adjusted_ts);
                                if !self.first_lidar_found
                                    && matches!(&event, InputEvent::Lidar(..))
                                {
                                    self.first_lidar_found = true;
                                    if self.inject_autonomous {
                                        self.next_inject_ts = Some(adjusted_ts + 100_000);
                                    }
                                }
                                self.next_real_event = Some(event);
                                return true;
                            }
                            Err(e) => {
                                eprintln!("Skipping invalid line: {} (error: {})", line.trim(), e);
                                continue;
                            }
                        }
                    }
                    Err(e) => {
                        eprintln!("Error reading line: {}", e);
                        return false;
                    }
                }
            } else {
                return false;
            }
        }
    }

    fn peek_next_ts(&mut self) -> Option<u64> {
        if self.next_real_event.is_none() {
            if !self.read_next_real() {
                return self.next_inject_ts;
            }
        }
        match (
            self.next_real_event.as_ref().map(get_ts),
            self.next_inject_ts,
        ) {
            (None, None) => None,
            (Some(rt), None) => Some(rt),
            (None, Some(it)) => Some(it),
            (Some(rt), Some(it)) => Some(rt.min(it)),
        }
    }

    fn get_and_process_next(&mut self) -> Option<InputEvent> {
        let next_real_ts = self.next_real_event.as_ref().map(get_ts);
        let next_inj_ts = self.next_inject_ts;

        let next_ts_opt = match (next_real_ts, next_inj_ts) {
            (None, None) => return None,
            (Some(rt), None) => Some(rt),
            (None, Some(it)) => Some(it),
            (Some(rt), Some(it)) => Some(rt.min(it)),
        };

        if next_ts_opt.is_some() {
            let is_inject_next =
                next_inj_ts.map_or(false, |it| next_real_ts.map_or(true, |rt| it <= rt));

            let event = if is_inject_next {
                let ts = next_inj_ts.unwrap();
                self.next_inject_ts = Some(ts + 100_000);
                InputEvent::WifiControl(ts, 2, 0.0, 0.0, 0.0)
            } else {
                self.next_real_event.take().unwrap()
            };

            self.process_event(&event);
            Some(event)
        } else {
            None
        }
    }

    fn process_event(&mut self, event: &InputEvent) {
        if self.first_ts == 0 {
            self.first_ts = get_ts(event);
        }
        let cloned_event = event.clone();
        self.logic.feed_event(cloned_event);
        if self.debug {
            match event {
                InputEvent::Lidar(ts, d1, d2, d3, d4) => println!(
                    "Processed Lidar ts={} d=({:.0},{:.0},{:.0},{:.0}) theta={:.4} rpm={:.2}",
                    *ts, *d1, *d2, *d3, *d4, self.logic.detector.theta, self.logic.detector.rpm
                ),
                InputEvent::Accelerometer(ts, ay, az) => println!(
                    "Processed Accel ts={} ay={:.2} az={:.2} theta={:.4} rpm={:.2}",
                    *ts, *ay, *az, self.logic.detector.theta, self.logic.detector.rpm
                ),
                _ => {}
            }
        }
        if let InputEvent::Planner(ts, plan, cw, os, op, theta, rpm) = event {
            self.theta_offset = self.logic.detector.theta - *theta;
            self.latest_planner = Some(event.clone());
            self.show_planner_theta = true;
            if self.debug {
                println!("Processed Planner ts={} plan={:?} theta={:.4} (sim: {:.4}) rpm={:.2} (sim: {:.4})", ts, plan, *theta, self.logic.detector.theta, rpm, self.logic.detector.rpm);
            }
        }
        if matches!(event, InputEvent::Lidar(..)) {
            self.current_lidar_points = self.logic.detector.last_xys.to_vec();
        }
        self.current_event_idx += 1;
    }
}

impl eframe::App for MyApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let now = Instant::now();
        let delta_real = now.duration_since(self.last_real).as_secs_f64();
        self.last_real = now;

        let mut speed = 0.0;
        if self.mode == "play" {
            speed = 1.0;
        } else if self.mode == "slow" {
            speed = 0.25;
        } else if self.mode == "pause" {
            speed = 0.0;
        }

        self.show_planner_theta = false;
        let mut processed_events = false;

        if self.first_ts == 0 {
            if let Some(ts) = self.peek_next_ts() {
                self.first_ts = ts;
            }
        }

        if self.mode == "step" {
            if self.step_requested {
                self.step_requested = false;
                let mut found_lidar = false;
                while !found_lidar {
                    if let Some(next_ts) = self.peek_next_ts() {
                        if let Some(event) = self.get_and_process_next() {
                            processed_events = true;
                            if matches!(&event, InputEvent::Lidar(..)) {
                                found_lidar = true;
                            }
                        } else {
                            break;
                        }
                    } else {
                        break;
                    }
                }
                // Update virtual_elapsed based on last processed ts
                if let Some(last_ts) = self.logic.detector.last_ts {
                    self.virtual_elapsed = (last_ts - self.first_ts) as f64 / 1_000_000.0;
                }
            }
        } else {
            self.virtual_elapsed += delta_real * speed;
            let current_sim_ts = self.first_ts as f64 + self.virtual_elapsed * 1_000_000.0;

            loop {
                if let Some(next_ts) = self.peek_next_ts() {
                    if next_ts as f64 > current_sim_ts {
                        break;
                    }
                    if let Some(_) = self.get_and_process_next() {
                        processed_events = true;
                    } else {
                        break;
                    }
                } else {
                    break;
                }
            }
        }

        self.logic.step(None);

        let (closest_wall, open_space, object_pos) = self.logic.detector.detect_objects(self.debug);

        if self.debug && processed_events {
            println!("Simulator: ts={} theta={:.4} rpm={:.2} cw=({:.1},{:.1}) os=({:.1},{:.1}) op=({:.1},{:.1})",
                self.logic.detector.last_ts.unwrap_or(0),
                self.logic.detector.theta,
                self.logic.detector.rpm,
                closest_wall.0, closest_wall.1,
                open_space.0, open_space.1,
                object_pos.0, object_pos.1);
        }

        let target_rpm = self.latest_planner.as_ref().map_or(0.0, |p| {
            if let InputEvent::Planner(_, plan, _, _, _, _, _) = p {
                plan.rotation_speed
            } else {
                0.0
            }
        });
        let measured_rpm = self.latest_planner.as_ref().map_or(0.0, |p| {
            if let InputEvent::Planner(_, _, _, _, _, _, rpm) = p {
                *rpm
            } else {
                0.0
            }
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            ui.heading(format!(
                "Real-Time Robot LiDAR Simulation\nEvents: {}, TS: {} ms, Simulator RPM: {:.2}, Measured RPM: {:.2}, Target RPM: {:.2}",
                self.current_event_idx,
                self.logic.detector.last_ts.unwrap_or(0) / 1000,
                self.logic.detector.rpm,
                measured_rpm,
                target_rpm
            ));

            let plot = Plot::new("lidar_plot")
                .include_x(-800.0)
                .include_x(800.0)
                .include_y(-800.0)
                .include_y(800.0)
                .data_aspect(1.0)
                .show_axes(true)
                .show_grid(true)
                .allow_drag(false)
                .allow_zoom(false)
                .allow_scroll(false);

            plot.show(ui, |plot_ui| {
                plot_ui.set_plot_bounds(PlotBounds::from_min_max([-800., -800.], [800., 800.]));

                let points: Vec<[f64; 2]> = (0..NUM_BINS).filter_map(|i| {
                    let d = self.logic.detector.bins_dist[i];
                    if d.is_finite() {
                        let angle = i as f32 * BIN_ANGLE_STEP;
                        Some([(d * angle.cos()) as f64, (d * angle.sin()) as f64])
                    } else {
                        None
                    }
                }).collect();
                plot_ui.points(
                    egui_plot::Points::new(points)
                        .color(egui::Color32::BLUE)
                        .radius(5.0),
                );

                let h_points: Vec<[f64; 2]> = self.current_lidar_points
                    .iter()
                    .map(|&(x, y)| [x as f64, y as f64])
                    .collect();
                plot_ui.points(
                    egui_plot::Points::new(h_points)
                        .color(egui::Color32::RED)
                        .radius(6.0),
                );

                let heading_len: f64 = 200.0;
                let hx = heading_len * self.logic.detector.theta.cos() as f64;
                let hy = heading_len * self.logic.detector.theta.sin() as f64;
                plot_ui.line(
                    Line::new(PlotPoints::new(vec![[0.0, 0.0], [hx, hy]]))
                        .color(egui::Color32::RED)
                        .width(2.0),
                );

                if closest_wall != (0.0, 0.0) {
                    plot_ui.line(
                        Line::new(PlotPoints::new(vec![
                            [0.0, 0.0],
                            [closest_wall.0 as f64, closest_wall.1 as f64],
                        ]))
                        .color(egui::Color32::GREEN)
                        .width(2.0),
                    );
                }

                if open_space != (0.0, 0.0) {
                    plot_ui.line(
                        Line::new(PlotPoints::new(vec![
                            [0.0, 0.0],
                            [open_space.0 as f64, open_space.1 as f64],
                        ]))
                        .color(egui::Color32::BLUE)
                        .width(2.0),
                    );
                }

                if object_pos != (100.0, 100.0) {
                    plot_ui.line(
                        Line::new(PlotPoints::new(vec![
                            [0.0, 0.0],
                            [object_pos.0 as f64, object_pos.1 as f64],
                        ]))
                        .color(egui::Color32::from_rgb(128, 0, 128))
                        .width(2.0),
                    );
                }

                // Visualize simulated MotorControlPlan movement vector
                if let Some(plan) = &self.logic.motor_control_plan {
                    let speed = (plan.movement_x.powi(2) + plan.movement_y.powi(2)).sqrt();
                    if speed > 0.0 {
                        let vis_length = 200.0 * speed as f64;
                        let dir_x = plan.movement_x / speed;
                        let dir_y = plan.movement_y / speed;
                        let mv_x = vis_length * dir_x as f64;
                        let mv_y = vis_length * dir_y as f64;
                        plot_ui.line(
                            Line::new(PlotPoints::new(vec![[0.0, 0.0], [mv_x, mv_y]]))
                                .color(egui::Color32::from_rgb(255, 165, 0)) // Orange
                                .width(3.0),
                        );
                    }
                }


                if let Some(event) = &self.latest_planner {
                    if let InputEvent::Planner(ts, plan, cw, os, op, theta, rpm) = event {
                        let offset = self.theta_offset;

                        // Rotate and draw movement vector as yellow line
                        let (rot_mv_x, rot_mv_y) = self.rotate_point(plan.movement_x, plan.movement_y, offset);
                        plot_ui.line(
                            Line::new(PlotPoints::new(vec![
                                [0.0, 0.0],
                                [rot_mv_x, rot_mv_y],
                            ]))
                            .color(egui::Color32::YELLOW)
                            .width(2.0),
                        );

                        // Draw detection vectors as large dots after rotation
                        let dot_radius = 10.0;
                        if cw.0 != 0.0 || cw.1 != 0.0 {
                            let (rot_x, rot_y) = self.rotate_point(cw.0, cw.1, offset);
                            plot_ui.points(
                                egui_plot::Points::new(vec![[rot_x, rot_y]])
                                    .color(egui::Color32::GREEN)
                                    .radius(dot_radius),
                            );
                        }
                        if os.0 != 0.0 || os.1 != 0.0 {
                            let (rot_x, rot_y) = self.rotate_point(os.0, os.1, offset);
                            plot_ui.points(
                                egui_plot::Points::new(vec![[rot_x, rot_y]])
                                    .color(egui::Color32::BLUE)
                                    .radius(dot_radius),
                            );
                        }
                        if op.0 != 100.0 || op.1 != 100.0 {
                            let (rot_x, rot_y) = self.rotate_point(op.0, op.1, offset);
                            plot_ui.points(
                                egui_plot::Points::new(vec![[rot_x, rot_y]])
                                    .color(egui::Color32::from_rgb(128, 0, 128))
                                    .radius(dot_radius),
                            );
                        }

                        // Draw theta as yellow dot if show_planner_theta, rotated
                        if self.show_planner_theta {
                            let adjusted_theta = *theta + offset;
                            let theta_x = 200.0 * adjusted_theta.cos() as f64;
                            let theta_y = 200.0 * adjusted_theta.sin() as f64;
                            plot_ui.points(
                                egui_plot::Points::new(vec![[theta_x, theta_y]])
                                    .color(egui::Color32::YELLOW)
                                    .radius(8.0),
                            );
                        }
                    }
                }
            });
        });

        if ctx.input(|i| i.key_pressed(egui::Key::C)) {
            self.mode = "play".to_string();
            self.current_lidar_points.clear();
        }
        if ctx.input(|i| i.key_pressed(egui::Key::X)) {
            self.mode = "slow".to_string();
            self.current_lidar_points.clear();
        }
        if ctx.input(|i| i.key_pressed(egui::Key::Z)) {
            self.mode = "pause".to_string();
            self.current_lidar_points.clear();
        }
        if ctx.input(|i| i.key_pressed(egui::Key::E)) {
            if self.mode != "step" {
                self.mode = "step".to_string();
                self.current_lidar_points.clear();
            } else {
                self.step_requested = true;
            }
        }

        ctx.request_repaint();
    }
}
