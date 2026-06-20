# ESP Media Station

ESP32-S3 based embedded multimedia station with live video streaming, two-way audio, TinyML human detection, and touchscreen UI.

---

## Overview

The ESP Media Station is a self-contained multimedia embedded system built on the ESP32-S3 N16R8. It combines a camera pipeline, video streaming, two-way audio, a touchscreen interface, and on device human presence detection: all running concurrently and locally on the S3.

The project targets intermediate ESP-IDF development and serves as a platform for exploring the limits of a single high-performance microcontroller handling a full multimedia workload.

---

## Hardware

| Component | Details |
| --- | --- |
| MCU | ESP32-S3 N16R8 (16MB Flash, 8MB PSRAM) |
| Camera | OV3660 via FPC connector (DVP interface) |
| Display | ILI9341 320x240 TFT with resistive touch |
| Audio Input | INMP441 MEMS microphone |
| Audio Output | MAX98357 audio amplifier and standard 4-ohm speaker |

---

## Features

- **Live video streaming:** MJPEG over a custom RTP/UDP implementation (RFC 2435)
- **Two way audio:** Full duplex I2S mic input and speaker output, streamed over a separate RTP session
- **Human presence detection:** TensorFlow Lite for Microcontrollers (INT8 quantized), used for human present/not-present trigger
- **Touchscreen UI:** LVGL based menu with on screen controls, rendered on ILI9341
- **Flash encryption:** AES-256-XTS, key generated and burned to eFuse on device
- **Secure boot:** Secure Boot V2 with ECDSA signed bootloader and application partition

---

## Security

Both flash encryption and secure boot are configured via eFuse and are irreversible in production mode. Development mode is used during active development to allow re-flashing.

- **Flash Encryption:** AES-256-XTS. Key generated on device, never exposed.
- **Secure Boot:** ECDSA. Bootloader and application partition signed. Public key digest burned to eFuse.

---

## Build

Requires ESP-IDF v6.0.1

```bash
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

To build tests

```bash
cd tests
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

