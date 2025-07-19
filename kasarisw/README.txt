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


Description of the hardware set-up
----------------------------------

This project implements a spinning robot with a fixed LIDAR mounted onto it. The
rotation speed is measured using an accelerometer measuring the normal
acceleration of a fixed point in the robot body.

The sensor type is a triangulation LIDAR which are commonly used in robot vacuum
cleaners, but it is ran in a non-standard way removed from its regular mount,
maxing out the sampling rate by generating a "fake" encoder signal which does
not depend on the actual RPM of the robot, with the only goal of getting as
many samples from it as possible. There is no LIDAR motor. Instead, the entire
robot spins. There is no TX line to the LIDAR, only RX. The LIDAR encodes 4
evenly spaced measurements per event.

