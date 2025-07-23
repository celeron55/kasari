// simulator/app.rs
use crate::events::get_ts;
use crate::physics::{Rect, Robot, World};
use crate::sources::EventSource;
use crate::sources::FileEventSource;
use crate::sources::SimEventSource;
use eframe::egui;
use egui_plot::{Line, Plot, PlotBounds, PlotPoints};
use kasarisw::shared::algorithm::{BIN_ANGLE_STEP, NUM_BINS};
use kasarisw::shared::kasari::{InputEvent, MainLogic, MotorControlPlan};
use std::io::Error as IoError;
use std::time::Instant;

pub struct MyApp {
    event_source: Box<dyn EventSource>,
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
}

impl MyApp {
    pub fn new(
        lines: Box<dyn Iterator<Item = Result<String, IoError>>>,
        debug: bool,
        inject_autonomous: bool,
        lidar_distance_offset: f32,
        sim_mode: bool,
        arena_width: f32,
        arena_height: f32,
        object_present: bool,
        reverse_rotation: bool,
        robot_flipped: bool,
    ) -> Self {
        let event_source: Box<dyn EventSource> = if sim_mode {
            Box::new(SimEventSource::new(
                lidar_distance_offset,
                debug,
                arena_width,
                arena_height,
                object_present,
                reverse_rotation,
                robot_flipped,
            ))
        } else {
            Box::new(FileEventSource::new(
                lines,
                debug,
                inject_autonomous,
                lidar_distance_offset,
                reverse_rotation,
            ))
        };

        Self {
            event_source,
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
        }
    }

    fn rotate_point(&self, x: f32, y: f32, angle: f32) -> (f64, f64) {
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        let new_x = x * cos_a - y * sin_a;
        let new_y = x * sin_a + y * cos_a;
        (new_x as f64, new_y as f64)
    }

    fn process_event(&mut self, event: &InputEvent) {
        if let InputEvent::Planner(ts, plan, cw, os, op, theta, rpm) = event {
            self.theta_offset = self.event_source.get_logic().unwrap().detector.theta - *theta;
            self.latest_planner = Some(event.clone());
            self.show_planner_theta = true;
            if self.debug {
                println!("Processed Planner ts={} plan={:?} theta={:.4} (sim: {:.4}) rpm={:.2} (sim: {:.4})", ts, plan, *theta, self.event_source.get_logic().unwrap().detector.theta, rpm, self.event_source.get_logic().unwrap().detector.rpm);
            }
        }
        if matches!(event, InputEvent::Lidar(..)) {
            self.current_lidar_points = self
                .event_source
                .get_logic()
                .unwrap()
                .detector
                .last_xys
                .to_vec();
        }
        self.current_event_idx += 1;
    }
}

fn world_to_local(wx: f32, wy: f32, pos_x: f32, pos_y: f32, theta: f32) -> (f64, f64) {
    let dx = wx - pos_x;
    let dy = wy - pos_y;
    let cos_t = theta.cos();
    let sin_t = theta.sin();
    let lx = dx * cos_t - dy * sin_t;
    let ly = dx * sin_t + dy * cos_t;
    (lx as f64, ly as f64)
}

impl eframe::App for MyApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let now = Instant::now();
        let delta_real = now.duration_since(self.last_real).as_secs_f64();
        self.last_real = now;

        let mut speed = 0.0;
        if self.mode == "play" {
            speed = 1.0;
        } else if self.mode == "semislow" {
            speed = 0.25;
        } else if self.mode == "slow" {
            speed = 0.05;
        } else if self.mode == "pause" {
            speed = 0.0;
        }

        self.show_planner_theta = false;
        let mut processed_events = false;

        if self.first_ts == 0 {
            if let Some(ts) = self.event_source.peek_next_ts() {
                self.first_ts = ts;
            }
        }

        if self.mode == "step" {
            if self.step_requested {
                self.step_requested = false;
                let mut found_lidar = false;
                while !found_lidar {
                    if let Some(event) = self.event_source.get_next_event() {
                        self.process_event(&event);
                        processed_events = true;
                        if matches!(&event, InputEvent::Lidar(..)) {
                            found_lidar = true;
                        }
                    } else {
                        break;
                    }
                }
                // Update virtual_elapsed based on last processed ts
                if let Some(last_ts) = self.event_source.get_logic().unwrap().detector.last_ts {
                    self.virtual_elapsed = (last_ts - self.first_ts) as f64 / 1_000_000.0;
                }
            }
        } else {
            self.virtual_elapsed += delta_real * speed;
            let current_sim_ts = self.first_ts + (self.virtual_elapsed * 1_000_000.0) as u64;

            loop {
                if let Some(next_ts) = self.event_source.peek_next_ts() {
                    if next_ts > current_sim_ts {
                        break;
                    }
                    if let Some(event) = self.event_source.get_next_event() {
                        self.process_event(&event);
                        processed_events = true;
                    } else {
                        break;
                    }
                } else {
                    break;
                }
            }
        }

        // Object detection
        let (closest_wall, open_space, object_pos) = (
            self.event_source.get_logic().unwrap().latest_closest_wall,
            self.event_source.get_logic().unwrap().latest_open_space,
            self.event_source.get_logic().unwrap().latest_object_pos,
        );

        if self.debug && processed_events {
            println!("Simulator: ts={} theta={:.4} rpm={:.2} cw=({:.1},{:.1}) os=({:.1},{:.1}) op=({:.1},{:.1})",
                self.event_source.get_logic().unwrap().detector.last_ts.unwrap_or(0),
                self.event_source.get_logic().unwrap().detector.theta,
                self.event_source.get_logic().unwrap().detector.rpm,
                closest_wall.0, closest_wall.1,
                open_space.0, open_space.1,
                object_pos.0, object_pos.1);
        }

        // RPM values from event log
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

        let robot_opt = self.event_source.get_robot();
        let world_opt = self.event_source.get_world();

        egui::CentralPanel::default().show(ctx, |ui| {
            let heading_text = format!(
                "Real-Time Robot LiDAR Simulation\nEvents: {}, TS: {} ms, Simulator RPM: {:.2}, Measured RPM: {:.2}, Target RPM: {:.2}, flipped: {} (simulated) {} (measured)",
                self.current_event_idx,
                self.event_source.get_logic().unwrap().detector.last_ts.unwrap_or(0) / 1000,
                self.event_source.get_logic().unwrap().detector.rpm,
                measured_rpm,
                target_rpm,
                self.event_source.get_robot_flipped(),
                self.event_source.get_logic().unwrap().detector.flipped,
            );
            ui.heading(heading_text);

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
                    let d = self.event_source.get_logic().unwrap().detector.bins_dist[i];
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
                        .color(egui::Color32::LIGHT_BLUE)
                        .radius(6.0),
                );

                let heading_len: f64 = 200.0;
                let hx = heading_len * self.event_source.get_logic().unwrap().detector.theta.cos() as f64;
                let hy = heading_len * self.event_source.get_logic().unwrap().detector.theta.sin() as f64;
                plot_ui.line(
                    Line::new(PlotPoints::new(vec![[0.0, 0.0], [hx, hy]]))
                        .color(egui::Color32::LIGHT_BLUE)
                        .width(2.0),
                );

                if closest_wall != (0.0, 0.0) {
                    plot_ui.line(
                        Line::new(PlotPoints::new(vec![
                            [0.0, 0.0],
                            [closest_wall.0 as f64, closest_wall.1 as f64],
                        ]))
                        .color(egui::Color32::BLUE)
                        .width(2.0),
                    );
                }

                if open_space != (0.0, 0.0) {
                    plot_ui.line(
                        Line::new(PlotPoints::new(vec![
                            [0.0, 0.0],
                            [open_space.0 as f64, open_space.1 as f64],
                        ]))
                        .color(egui::Color32::GREEN)
                        .width(2.0),
                    );
                }

                if object_pos != (100.0, 100.0) {
                    plot_ui.line(
                        Line::new(PlotPoints::new(vec![
                            [0.0, 0.0],
                            [object_pos.0 as f64, object_pos.1 as f64],
                        ]))
                        .color(egui::Color32::RED)
                        .width(2.0),
                    );
                }

                // Visualize simulated MotorControlPlan movement vector
                if let Some(plan) = &self.event_source.get_logic().unwrap().motor_control_plan {
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

                // The motor part is currently commented out for clarity
                // Modulation is known to work so we don't need to see this
                /*let motor_radius: f64 = 40.0;
                let speed_scale: f64 = 2.0 / 30.0;
                let left_phi: f32 = PI / 2.0 + self.event_source.get_logic().unwrap().detector.theta;
                let right_phi: f32 = -PI / 2.0 + self.event_source.get_logic().unwrap().detector.theta;
                let left_pos: [f64; 2] = [motor_radius * left_phi.cos() as f64, motor_radius * left_phi.sin() as f64];
                let right_pos: [f64; 2] = [motor_radius * right_phi.cos() as f64, motor_radius * right_phi.sin() as f64];
                let left_dir: [f64; 2] = [-left_phi.sin() as f64, left_phi.cos() as f64];
                let right_dir: [f64; 2] = [-right_phi.sin() as f64, right_phi.cos() as f64];
                let left_vec: [f64; 2] = [left_dir[0] * left_rpm as f64 * speed_scale, left_dir[1] * left_rpm as f64 * speed_scale];
                let right_vec: [f64; 2] = [right_dir[0] * right_rpm as f64 * speed_scale, right_dir[1] * right_rpm as f64 * speed_scale];

                // Motor positions
                plot_ui.points(
                    egui_plot::Points::new(vec![left_pos])
                        .color(egui::Color32::WHITE)
                        .radius(3.0),
                );
                plot_ui.points(
                    egui_plot::Points::new(vec![right_pos])
                        .color(egui::Color32::WHITE)
                        .radius(3.0),
                );

                // Target speed vectors
                plot_ui.line(
                    Line::new(PlotPoints::new(vec![left_pos, [left_pos[0] + left_vec[0], left_pos[1] + left_vec[1]]]))
                        .color(egui::Color32::from_rgb(255, 0, 0))
                        .width(2.0),
                );
                plot_ui.line(
                    Line::new(PlotPoints::new(vec![right_pos, [right_pos[0] + right_vec[0], right_pos[1] + right_vec[1]]]))
                        .color(egui::Color32::from_rgb(0, 255, 0))
                        .width(2.0),
                );*/

                if let Some(event) = &self.latest_planner {
                    if let InputEvent::Planner(ts, plan, cw, os, op, theta, rpm) = event {
                        let offset = self.theta_offset;

                        // Draw detection and movement vectors as large dots after rotation
                        let dot_radius = 10.0;
                        if plan.movement_x != 0.0 || plan.movement_y != 0.0 {
                            let (rot_x, rot_y) = self.rotate_point(plan.movement_x, plan.movement_y, offset);
                            plot_ui.points(
                                egui_plot::Points::new(vec![[rot_x * 200.0, rot_y * 200.0]])
                                    .color(egui::Color32::from_rgb(255, 165, 0)) // Orange
                                    .radius(dot_radius),
                            );
                        }
                        if cw.0 != 0.0 || cw.1 != 0.0 {
                            let (rot_x, rot_y) = self.rotate_point(cw.0, cw.1, offset);
                            plot_ui.points(
                                egui_plot::Points::new(vec![[rot_x, rot_y]])
                                    .color(egui::Color32::BLUE)
                                    .radius(dot_radius),
                            );
                        }
                        if os.0 != 0.0 || os.1 != 0.0 {
                            let (rot_x, rot_y) = self.rotate_point(os.0, os.1, offset);
                            plot_ui.points(
                                egui_plot::Points::new(vec![[rot_x, rot_y]])
                                    .color(egui::Color32::GREEN)
                                    .radius(dot_radius),
                            );
                        }
                        if op.0 != 100.0 || op.1 != 100.0 {
                            let (rot_x, rot_y) = self.rotate_point(op.0, op.1, offset);
                            plot_ui.points(
                                egui_plot::Points::new(vec![[rot_x, rot_y]])
                                    .color(egui::Color32::RED)
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

                if let (Some(robot), Some(world)) = (robot_opt, world_opt) {
                    let mut rects: Vec<Rect> = vec![world.arena];
                    rects.extend(world.objects.clone());
                    let robot_rect = Rect {
                        min_x: robot.pos_x - 70.0,
                        min_y: robot.pos_y - 70.0,
                        max_x: robot.pos_x + 70.0,
                        max_y: robot.pos_y + 70.0,
                    };
                    rects.push(robot_rect);

                    // This aligns the rectangles to the lidar plot
                    let world_theta = self.event_source.get_logic().unwrap().detector.theta - robot.theta;

                    for r in rects {
                        let corners = vec![
                            world_to_local(r.min_x, r.min_y, robot.pos_x, robot.pos_y, world_theta),
                            world_to_local(r.max_x, r.min_y, robot.pos_x, robot.pos_y, world_theta),
                            world_to_local(r.max_x, r.max_y, robot.pos_x, robot.pos_y, world_theta),
                            world_to_local(r.min_x, r.max_y, robot.pos_x, robot.pos_y, world_theta),
                            world_to_local(r.min_x, r.min_y, robot.pos_x, robot.pos_y, world_theta), // close the loop
                        ];

                        for i in 0..4 {
                            let p1 = corners[i];
                            let p2 = corners[i + 1];
                            plot_ui.line(
                                Line::new(PlotPoints::new(vec![[p1.0, p1.1], [p2.0, p2.1]]))
                                    .color(egui::Color32::DARK_GRAY)
                                    .width(1.0),
                            );
                        }
                    }
                }
            });
        });

        if ctx.input(|i| i.key_pressed(egui::Key::V)) {
            self.mode = "play".to_string();
        }
        if ctx.input(|i| i.key_pressed(egui::Key::C)) {
            self.mode = "semislow".to_string();
        }
        if ctx.input(|i| i.key_pressed(egui::Key::X)) {
            self.mode = "slow".to_string();
        }
        if ctx.input(|i| i.key_pressed(egui::Key::Z)) {
            self.mode = "pause".to_string();
        }
        if ctx.input(|i| i.key_pressed(egui::Key::E) || i.key_pressed(egui::Key::Space)) {
            if self.mode != "step" {
                self.mode = "step".to_string();
                self.current_lidar_points.clear();
            } else {
                self.step_requested = true;
            }
        }
        if ctx.input(|i| i.key_pressed(egui::Key::F)) {
            self.event_source
                .set_robot_flipped(!self.event_source.get_robot_flipped());
        }
        if ctx.input(|i| i.key_pressed(egui::Key::Q) || i.key_pressed(egui::Key::Escape)) {
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }

        ctx.request_repaint();
    }
}
