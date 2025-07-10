kasarisw
========

Before any build commands:
$ . export-esp.sh
$ # Configure wifi network to connect to (in addition to hosting one):
$ export SSID=foo
$ export PASSWORD=bar

Build:
$ cargo build

Monitor compiler errors:
$ bacon

Flash:
$ cargo run

Monitor target:
$ ./monitor.sh

Receiving events from the robot:
$ python tools/event_monitor.py
or:
$ curl 192.168.1.248:8080 | python tools/parse_events.py

