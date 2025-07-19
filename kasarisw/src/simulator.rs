use std::io;

mod shared;
use shared::kasari::{InputEvent, MainLogic};

fn main() {
    println!("Starting simulator...");
    let mut logic = MainLogic::new();

    loop {
        // TODO: Read events from file and feed them to logic
        logic.feed_event(InputEvent::Vbat(0, 11.0));

        logic.step();

        // TODO: Plot lidar points
        // TODO: Plot detected vectors
    }
}
