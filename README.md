# CAN Bridge

Stream CAN bus frames from an Arduino MKR WiFi 1010 + MKR CAN Shield to a TCP stream.

## Hardware

- [Arduino MKR WiFi 1010](https://docs.arduino.cc/hardware/mkr-wifi-1010/)
- [Arduino MKR CAN Shield](https://docs.arduino.cc/hardware/mkr-can-shield/)

## Features

- Reads CAN 2.0 frames from the bus.
- Encapsulates frames into a simple binary format.
- Streams frames over TCP to a configurable host and port.
