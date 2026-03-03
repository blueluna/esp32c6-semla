# esp32c6-semla

A `no_std` Rust firmware for the ESP32-C6 that drives a Pimoroni Inky pHat
e-paper display and counts down the days to **Fettisdag**.

![esp32c6-semla](resources/esp32c6-semla.jpg)

## How it works

1. **Wi-Fi** — connects to a configured access point on boot. Credentials are
   stored in a dedicated flash partition (`cfg_0`) using
   [`sequential-storage`](https://crates.io/crates/sequential-storage), falling
   back to compile-time `SSID`/`PASSWORD` env vars on first boot.

2. **NTP** — synchronises the clock via `pool.ntp.org` using
   [`sntpc`](https://crates.io/crates/sntpc). The display is refreshed once per
   day, timed to trigger at midnight in the `Europe/Stockholm` timezone (via
   [`jiff`](https://crates.io/crates/jiff)).

3. **Fettisdag calculation** — Fettisdag is Shrove Tuesday, which falls 47 days
   before Easter Sunday. Easter is computed using the Meeus/Jones/Butcher
   algorithm (pure integer arithmetic, no external crate needed). If Fettisdag
   has already passed for the current year, the countdown rolls over to the
   following year.

4. **Display** — rendered onto the 212×104 three-colour (black/white/red)
   e-paper panel via SPI using the
   [`ssd1675`](https://crates.io/crates/ssd1675) driver and
   [`u8g2-fonts`](https://crates.io/crates/u8g2-fonts) for text.

## Hardware

| Display pin | ESP32-C6 GPIO | Description        |
|-------------|---------------|--------------------|
| VCC         | 3V3           | Power supply       |
| GND         | GND           | Ground             |
| DIN         | 2             | SPI MOSI           |
| CLK         | 3             | SPI clock          |
| CS          | 4             | SPI chip select    |
| DC          | 5             | Data / Command     |
| RST         | 19            | Display reset      |
| BUSY        | 18            | Display busy       |

## Building and flashing

Set Wi-Fi credentials (or store them in flash after the first flash):

```sh
export SSID="your-network"
export PASSWORD="your-password"
cargo run --release
```

Flashing uses [`probe-rs`](https://probe.rs) with an ESP32-C6 USB JTAG probe.
The target is `riscv32imac-unknown-none-elf`.
