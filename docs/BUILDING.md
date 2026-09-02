# Building

This project is an Arduino sketch that targets the Arduino MKR WiFi 1010 with the MKR CAN Shield. The build is driven by the [Arduino CLI](https://arduino.github.io/arduino-cli/).

## Prerequisites

- [Arduino CLI](https://arduino.github.io/arduino-cli/latest/installation/) installed and available on your `PATH`.
- The `arduino:samd` boards platform (see below).

## Install the boards platform

```sh
arduino-cli core update-index
arduino-cli core install arduino:samd
```

## Compile

From the repository root:

```sh
arduino-cli compile --fqbn arduino:samd:mkrwifi1010 can_bridge.ino
```

## Upload (optional)

Connect the board, then upload to its serial port (e.g. `/dev/cu.usbmodem*` on macOS):

```sh
arduino-cli upload -p <PORT> --fqbn arduino:samd:mkrwifi1010 can_bridge.ino
```

## Continuous integration

The same board (`arduino:samd:mkrwifi1010`) is compiled in CI by the `compile` job in `.github/workflows/arduino.yml`.
