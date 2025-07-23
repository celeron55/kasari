use clap::Parser;
use eframe::{self, egui};
use std::error::Error;
use std::fs::File;
use std::io::{self, BufRead, BufReader, Error as IoError};

mod app;
mod events;
mod physics;
mod sources;

use app::MyApp;
use sources::{EventSource, FileEventSource, SimEventSource};

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
                args.lidar_distance_offset,
                args.sim,
                args.arena_width,
                args.arena_height,
                args.no_object,
                args.reverse_rotation,
                args.robot_flipped,
            )))
        }),
    )?;

    Ok(())
}
