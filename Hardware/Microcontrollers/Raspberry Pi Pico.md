---
title: Raspberry Pi Pico
aliases: [RP2040, Raspberry Pi Pico Board, Pico Microcontroller]
tags: [microcontroller, raspberry-pi, hardware, embedded, rp2040]
---

# 🟩 Raspberry Pi Pico

The **Raspberry Pi Pico** is a low-cost, high-performance microcontroller board based on the **RP2040** chip, designed by the Raspberry Pi Foundation. Unlike other Raspberry Pi boards, the Pico is not a full single-board computer but a microcontroller platform ideal for embedded systems, IoT, robotics, and education.

---

## 🧭 Overview

- **RP2040 Microcontroller**: Dual-core ARM Cortex-M0+ @ 133 MHz
- **RAM**: 264 KB SRAM
- **Flash**: 2 MB onboard QSPI flash (external, up to 16 MB supported)
- **GPIO**: 26 multi-function pins (3.3V logic)
- **Peripherals**: 2x UART, 2x SPI, 2x I2C, 16x PWM channels, 3x ADC (12-bit), USB 1.1 device/host
- **Programmable I/O (PIO)**: 2 PIO blocks for custom peripherals and protocols
- **Power**: 1.8–5.5V input, low power modes supported
- **Form Factor**: 21 × 51 mm, breadboard-friendly

---

## 📦 Common Use Cases

- Embedded systems and IoT devices
- Robotics and motor control
- Sensor interfacing and data logging
- Educational projects and prototyping
- Custom protocol implementation using PIO

---

## 🛠️ Programming Options

- **C/C++ SDK**: Official low-level SDK for maximum performance and control
- **MicroPython**: Easy scripting and rapid prototyping
- **CircuitPython**: Adafruit’s Python variant for microcontrollers
- **Arduino Core**: Arduino-style programming (community-supported)

---

## 🔌 Peripherals & Features

- 26 GPIO pins (including 3 ADC inputs)
- 2x UART, 2x SPI, 2x I2C
- 16x PWM channels
- 3x 12-bit ADC channels
- USB 1.1 device/host support
- 2x Programmable I/O (PIO) blocks for custom digital interfaces

---

## 🆚 Raspberry Pi Pico vs. Other Microcontrollers

| Feature                | Raspberry Pi Pico | ESP32                  | ESP8266                | ATmega328P (Arduino Uno) |
|------------------------|------------------|------------------------|------------------------|--------------------------|
| **CPU**                | 2x Cortex-M0+ @ 133 MHz | 2x Xtensa LX6 @ 240 MHz | 1x Tensilica @ 80/160 MHz | 1x AVR @ 16 MHz          |
| **RAM**                | 264 KB           | ~520 KB                | ~50 KB                 | 2 KB                     |
| **Flash**              | 2 MB (up to 16 MB) | 4–16 MB (external)     | 512 KB–4 MB (external) | 32 KB                    |
| **Wi-Fi**              | ❌ No            | ✅ Yes                 | ✅ Yes                 | ❌ No                    |
| **Bluetooth**          | ❌ No            | ✅ Yes                 | ❌ No                  | ❌ No                    |
| **ADC**                | 3x 12-bit        | 18x 12-bit             | 1x 10-bit              | 6x 10-bit                |
| **PIO/Custom IO**      | ✅ Yes           | ❌ No                  | ❌ No                  | ❌ No                    |
| **USB**                | ✅ Yes (1.1)     | ✅ Yes (OTG)           | ❌ No                  | ❌ No                    |
| **Best Use Cases**     | Embedded, PIO, Education | IoT, wireless, BLE | Simple Wi-Fi IoT       | Basic embedded, Arduino  |

---

## ✅ Pros and ❌ Cons

### ✅ Advantages
- **Low Cost**: Affordable for education and prototyping.
- **Powerful PIO**: Enables custom digital interfaces and protocols.
- **Dual-Core**: Parallel processing for advanced applications.
- **Flexible Programming**: Supports C/C++, MicroPython, CircuitPython, and Arduino.
- **Strong Community**: Backed by Raspberry Pi Foundation and open-source ecosystem.

### ❌ Disadvantages
- **No Built-in Wi-Fi/Bluetooth**: Requires external modules for wireless connectivity.
- **Limited Flash/RAM**: Less than some higher-end MCUs.
- **No DAC**: Lacks built-in digital-to-analog converter.
- **Not Linux-capable**: Unlike other Raspberry Pi boards, cannot run Linux.

---

## 📦 Common Boards & Variants

- **Raspberry Pi Pico**: Standard board
- **Raspberry Pi Pico W**: Adds Wi-Fi (Infineon CYW43439 chip)
- **Raspberry Pi Pico H**: Pre-soldered headers
- **Third-party boards**: Pimoroni Tiny 2040, Adafruit Feather RP2040, etc.

---

## 🔗 See Also

- [[ESP32]]
- [[ESP8266]]
- [[ATmega328P]]
- [[Microcontrollers]]
- [[Raspberry Pi]]
- [[MicroPython]]
- [[CircuitPython]]

---

## 📚 Resources

- [Raspberry Pi Pico Documentation](https://www.raspberrypi.com/documentation/microcontrollers/)
- [RP2040 Datasheet](https://www.raspberrypi.com/documentation/microcontrollers/rp2040.html)
- [MicroPython Pico Guide](https://docs.micropython.org/en/latest/rp2/)
- [CircuitPython Pico Guide](https://learn.adafruit.com/getting-started-with-raspberry-pi-pico-circuitpython)
- [Pico Arduino Core](https://github.com/earlephilhower/arduino-pico)
- [Raspberry Pi Pico Projects](https://projects.raspberrypi.org/en/projects?software%5B%5D=raspberry-pi-pico)

---

## 🧠 Summary

The Raspberry Pi Pico is a versatile, low-cost microcontroller board ideal for embedded systems, education, and rapid prototyping. Its unique PIO feature, dual-core processor, and flexible programming options make it a strong choice for both beginners and advanced users, though it lacks built-in wireless connectivity found in some competitors.
