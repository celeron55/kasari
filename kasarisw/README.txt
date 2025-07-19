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

