kasarisw
========

Overview
--------

This project implements a spinning robot using a repurposed LDS02RR LIDAR for
navigation. The robot spins its entire body (500-2000 RPM) to scan the
environment, measures rotation via accelerometer, and moves directionally by
modulating motor RPMs differentially. It supports autonomous object
following/wall avoidance.

Key components:
- Embedded firmware: Runs on ESP32, handles sensors, motion planning, WiFi event
  streaming.
- PC simulator: Replays logs or simulates a virtual world to visualize LIDAR
  points, detections, and behavior.
- Shared logic: Processes LIDAR/accel data for binning and detection.
- Python GUI (event_monitor.py): Monitors sensor data, logs events, sends
  controls over TCP.

Goals: Compare real robot behavior to simulation; maximize LIDAR sampling in
non-standard setup.

## Prerequisites

- Rust: Stable (1.88)
- For embedded (ESP32):
  - espup for toolchain
  - Python 3 (with tkinter) for tools/event_monitor.py
- Optional: bacon for compilation error monitoring
- Hardware (embedded): ESP32, LDS02RR LIDAR, ADXL373 accel, motors/ESCs, battery

PC target
---------

The PC target builds a GUI simulator (bin: simulator) for log replay or virtual
simulation.

Build:
$ cargo +stable build-pc  # Or cargo build --bin simulator --features pc

Monitor compiler errors:
$ bacon -j pc

Run executable:
$ ./target/debug/simulator  [OPTIONS] [SOURCE]

Options (from clap):
- SOURCE: Event log file or '-' for stdin (required unless --sim).
- --debug (-d): Enable debug prints.
- --inject-autonomous (-i): Inject autonomous mode (WifiControl mode=2) every 100ms.
- --lidar-distance-offset (-o) <MM>: Adjust LIDAR distances (default: 0.0).
- --sim (-s): Simulate virtual world (ignores SOURCE).

Example: Replay log with debug:
$ ./target/debug/simulator --debug events.log

Example: Simulate:
$ ./target/debug/simulator --sim

Embedded target
---------------

The embedded target (bin: kasarisw) is ESP32 firmware for the physical robot.

Install toolchain:
$ rm -rf ~/.rustup/toolchains/esp  # For a 100% clean install
$ rustup override unset  # An override can confuse everything up
$ cargo install espup
$ espup install
$ cp $HOME/export-esp.sh export-esp.sh

Before any build commands:
$ . export-esp.sh
$ # Configure wifi network to connect to (in addition to hosting one):
$ export SSID=foo
$ export PASSWORD=bar

Build:
$ cargo build-esp  # Or cargo build --bin kasarisw --features esp

Monitor compiler errors:
$ bacon

Flash and run:
$ cargo run-esp

Monitor target:
$ ./monitor.sh  # Serial logs via USB

## Monitoring and Control Tool (Python)

Use tools/event_monitor.py (Python 3 GUI) to stream events over WiFi/TCP, view
sensors, log JSON, and send controls.

Run:
$ python tools/event_monitor.py  # Launches Tkinter GUI

Features:
- Connect to robot (default: 192.168.1.248:8080; editable. For AP, enter
  192.168.2.1:8080).
- Displays: Lidar distances, Accelerometer (Y/Z G), Receiver pulse, Vbat
  voltage, Planner (speeds, positions, theta, RPM).
- Controls: Toggle "Control Target" (mode=1; sliders for
  rotation/movement/turning), "Autonomous Mode" (mode=2).
- Logs: Auto-saves JSON events to ./log/kasarisw_YYYY-MM-DD_HHMMSS.log.
- Parses binary events (via parse_events.py; tags XOR 0x5555).

CLI Alternative (parse binary from stdin):
$ python tools/parse_events.py < binary_stream  # Prints parsed events

Example: Pipe TCP stream to parse_events.py:
$ nc 192.168.1.248 8080 | python tools/parse_events.py > events.log

Description of the hardware and software
----------------------------------------

This project implements a spinning robot with a fixed LIDAR mounted onto it. The
rotation speed is measured using an accelerometer measuring the normal
acceleration of a fixed point in the robot body.

The sensor type is LDS02RR triangulation LIDAR which are commonly used in robot
vacuum cleaners, but it is ran in a non-standard way removed from its regular
mount, encoder and motor, maxing out the sampling rate by generating a "fake"
encoder signal which does not depend on the actual RPM of the robot, with the
only goal of getting as many samples from it as possible. There is no LIDAR
motor.  Instead, the entire robot spins. There is no TX line to the LIDAR, only
RX. The LIDAR encodes 4 evenly spaced measurements per event.

Known LIDAR quirk: LDS02RR has an internal ~11° angle offset (data not aligned
to physical "front"). Code applies distance offset (36mm mounting adjustment);
angle mismatches may need manual theta offsets in sim rendering.

When operating, the robot produces an event log which is captured over wifi
using tools/event_monitor.py.

This event log can be replayed by the simulator. The simulator displays the
robot's Planner events which indicate what the robot was doing. The simulator
also runs its own instance of the algorithm and displays the output of that.
This way, the robot's behavior can be compared to the simulator's behavior.

The robot achieves directional movement while spinning by differentially
modulating the RPM of its left and right motors, based on a target vector from
the motion planner in world coordinates. This creates net propulsion in the
desired direction, synchronized with real-time orientation from LIDAR and
accelerometer data. Rotation speeds between 500 to 2000 RPM allow LIDAR
operation and linear movement of the robot.

The robot has two rotations, one is its physical rotation which corresponds to
where the LIDAR is pointing. The other is its virtual rotation which it aims to
keep still relative to the world, of course limited by its sensor information
(it only knows its rotational speed within some tolerance, and the offset is
essentially random as the angle is integrated from RPM).

In the simulator, the robot is shown with its virtual rotation pointing to a
static direction (is it towards +X? I'm not sure). This makes the world shown
mostly still, but rotated by the offset and measurement error of the robot's
rotational speed measurement.

## Hardware Pinout (ESP32)

From embedded.rs:
- Motors: GPIO26 (right ESC PWM), GPIO27 (left ESC PWM).
- LIDAR: GPIO13 (fake encoder PWM out), GPIO16 (UART RX), GPIO17 (UART TX,
  unused).
- Accelerometer (ADXL373 SPI): GPIO5 (CS), GPIO18 (SCLK), GPIO19 (MISO), GPIO23
  (MOSI).
- RC Receiver: GPIO34 (PWM input via RMT).
- Battery Monitor: GPIO39 (ADC).

## Dependencies (from Cargo.toml)

- Shared: embassy-sync, ringbuffer, static_cell, arrayvec, num-traits, libm.
- Embedded (esp feature): esp-hal, esp-wifi, esp-alloc, etc.
- Simulator (pc feature): clap, eframe/egui_plot, serde_json.

## Event Format and Protocol

Events are JSON (logs) or binary (TCP stream on port 8080).
- Types: Lidar (distances), Accelerometer (G values), Receiver (pulse), Vbat
  (voltage), WifiControl (mode/speeds), Planner (plan/detections).
- Binary: Tag (u16 XOR 0x5555) + payload (struct-packed, e.g., Qffff for Lidar).
- WiFi: Hosts AP "kasarisw" (192.168.2.1); connects to env SSID. Stream events
  out; receive WifiControl in.

Example JSON event:
[ "Lidar", 123456789, 100.0, 200.0, 300.0, 400.0 ]

