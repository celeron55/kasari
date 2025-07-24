use clap::Parser;
use eframe::{self, egui};
use std::error::Error;
use std::fs::File;
use std::io::{self, BufRead, BufReader, Error as IoError};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

mod app;
mod events;
mod physics;
mod sources;

use app::MyApp;
use sources::{EventSource, FileEventSource, SimEventSource};
use events::get_ts;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Log file path or '-' for stdin
    #[arg(required_unless_present = "sim")]
    source: Option<String>,

    /// Enable debug prints
    #[arg(long, short)]
    debug: bool,

    /// Remove all WifiControl events and inject new ones with mode=2 every 100ms starting after first Lidar event
    #[arg(long, short)]
    inject_autonomous: bool,

    /// Offset LIDAR's distance values (mm)
    #[arg(long, short, default_value_t = 0.0)]
    lidar_distance_offset: f32,

    /// Run in true simulation mode (ignores source)
    #[arg(long, short)]
    sim: bool,

    /// Arena width in mm (simulation only)
    #[arg(long, default_value_t = 1200.0)]
    arena_width: f32,

    /// Arena height in mm (simulation only)
    #[arg(long, default_value_t = 1200.0)]
    arena_height: f32,

    /// Whether to include the object in the arena (simulation only)
    #[arg(long)]
    no_object: bool,

    /// Whether to set the robot to rotate in reverse
    #[arg(long)]
    reverse_rotation: bool,

    /// Whether to run the robot in the flipped state
    #[arg(long)]
    robot_flipped: bool,

    /// Headless mode for profiling
    #[arg(long)]
    headless: bool,
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();

    let lines: Box<dyn Iterator<Item = Result<String, IoError>>> = if args.sim {
        Box::new(std::iter::empty())
    } else {
        match args.source {
            Some(source) => {
                if source == "-" {
                    Box::new(io::stdin().lines())
                } else {
                    let file = File::open(&source)?;
                    let reader = BufReader::new(file);
                    Box::new(reader.lines())
                }
            }
            None => {
                return Err("Source argument required when not in simulation mode".into());
            }
        }
    };

    let mut app = MyApp::new(
        lines,
        args.debug,
        args.inject_autonomous,
        args.lidar_distance_offset,
        args.sim,
        args.arena_width,
        args.arena_height,
        args.no_object,
        args.reverse_rotation,
        args.robot_flipped,
    );

    if args.headless {
        let running = Arc::new(AtomicBool::new(true));
        let r = running.clone();
        ctrlc::set_handler(move || {
            r.store(false, Ordering::SeqCst);
            println!("Received SIGINT, shutting down gracefully...");
        })?;

        let mut processed_events = 0;
        let mut latest_ts = 0u64;
        let mut last_print = Instant::now();
        let print_interval = Duration::from_secs(1);

        while running.load(Ordering::SeqCst) {
            if let Some(event) = app.event_source.get_next_event() {
                app.process_event(&event); // Optional: process as in GUI mode
                processed_events += 1;
                latest_ts = get_ts(&event);
            } else {
                // No more events; could break or sleep
                std::thread::sleep(Duration::from_millis(100));
            }

            let now = Instant::now();
            if now - last_print >= print_interval {
                println!(
                    "Processed events: {}, Latest timestamp: {} ms",
                    processed_events,
                    latest_ts / 1000
                );
                last_print = now;
            }
        }

        // Final print on exit
        println!(
            "Final: Processed events: {}, Latest timestamp: {} ms",
            processed_events,
            latest_ts / 1000
        );
    } else {
        let options = eframe::NativeOptions {
            viewport: egui::ViewportBuilder::default().with_inner_size([2000.0, 2000.0]),
            ..Default::default()
        };
        eframe::run_native(
            "Robot Simulator",
            options,
            Box::new(move |_cc| Ok(Box::new(app))),
        )?;
    }

    Ok(())
}
