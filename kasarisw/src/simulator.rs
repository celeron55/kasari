use std::io;

mod shared;
use shared::kasari::{InputEvent, MainLogic};

fn main() {
    println!("Starting simulator...");
    let mut logic = MainLogic::new();

    // Example: Simulate feeding events (expand this with your simulator logic)
    logic.feed_event(InputEvent::Vbat(0, 11.0));

    // Example loop for user input or simulation
    loop {
        let mut input = String::new();
        io::stdin().read_line(&mut input).expect("Failed to read line");
        if input.trim() == "quit" {
            break;
        }
        // Parse input and feed events to logic, etc.
    }
}
