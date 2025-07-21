kasarisw
========

PC target
---------

Build:
$ cargo +stable build-pc

Monitor compiler errors:
$ bacon -j pc

Run executable:
$ ./target/debug/simulator

Embedded target
---------------

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
$ cargo build-esp

Monitor compiler errors:
$ bacon

Flash and run:
$ cargo run-esp

Monitor target:
$ ./monitor.sh

Receiving events from the robot via wifi:
$ python tools/event_monitor.py


Description of the hardware and software
----------------------------------------

This project implements a spinning robot with a fixed LIDAR mounted onto it. The
rotation speed is measured using an accelerometer measuring the normal
acceleration of a fixed point in the robot body.

The sensor type is XV-11 triangulation LIDAR which are commonly used in robot
vacuum cleaners, but it is ran in a non-standard way removed from its regular
mount, encoder and motor, maxing out the sampling rate by generating a "fake"
encoder signal which does not depend on the actual RPM of the robot, with the
only goal of getting as many samples from it as possible. There is no LIDAR
motor.  Instead, the entire robot spins. There is no TX line to the LIDAR, only
RX. The LIDAR encodes 4 evenly spaced measurements per event.

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
accelerometer data.

