Building:
$ sh export-esp.sh
$ cargo build

Flashing:
$ cargo run

Bluetooth support:
$ sudo dnf remove platformio python3-platformio
$ mv ~/.platformio ~/.platformio_old
$ curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
$ python3 get-platformio.py
$ cargo install cargo-pio --git https://github.com/ivmarkov/cargo-pio
$ cargo pio espidf menuconfig
- Enable bluetooth, save and quit

