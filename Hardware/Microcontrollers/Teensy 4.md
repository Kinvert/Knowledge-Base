---
title: Teensy 4.x (Teensy 4.0 / 4.1)
aliases: [Teensy 4, Teensy 4.0, Teensy 4.1, PJRC Teensy]
tags: [microcontroller, hardware, teensy, pjrc, embedded, arm, cortex-m7]
---

# ⚡ Teensy 4.x (Teensy 4.0 / 4.1)

The **Teensy 4.x** series (including **Teensy 4.0** and **Teensy 4.1**) are high-performance microcontroller development boards designed by **PJRC**. They are based on the NXP i.MX RT1062 chip, featuring a 32-bit ARM Cortex-M7 core running at up to 600 MHz, making them some of the fastest microcontroller boards available for hobbyists, makers, and professionals.

---

## 🧭 Overview

- **Processor**: ARM Cortex-M7 @ 600 MHz (NXP i.MX RT1062)
- **RAM**: 1024 KB (1 MB) SRAM
- **Flash**: 2 MB (on-chip, with QSPI expansion on 4.1)
- **GPIO**: 
  - Teensy 4.0: 40 digital I/O pins (18 PWM, 14 analog inputs)
  - Teensy 4.1: 55 digital I/O pins (35 PWM, 18 analog inputs)
- **Peripherals**: UART, SPI, I2C, CAN, I2S, SDIO, USB Host/Device, Ethernet (4.1 only)
- **Special Features**: 
  - High-speed USB (480 Mbps)
  - MicroSD slot (4.1)
  - Ethernet PHY (4.1)
  - QSPI memory expansion (4.1)
- **Form Factor**: Small, breadboard-friendly (4.0), larger with more I/O (4.1)
- **Power**: 3.3V logic, 5V tolerant inputs

---

## 📦 Common Use Cases

- High-speed data acquisition and signal processing
- Audio synthesis and DSP (Teensy Audio Library)
- Robotics and motor control
- Real-time control systems
- USB/MIDI devices and custom HID controllers
- Advanced LED and lighting projects
- Prototyping for embedded systems

---

## 🛠️ Programming Options

- **Arduino IDE** (with Teensyduino add-on): Easy access to Arduino ecosystem and libraries
- **PlatformIO**: Advanced development and project management
- **C/C++**: Direct programming for maximum performance
- **Audio Library**: Powerful real-time audio processing and synthesis

---

## 🔌 Peripherals & Features

- Multiple UART, SPI, I2C buses
- CAN bus support (with external transceiver)
- I2S audio interface
- SD card interface (4.1)
- Ethernet (4.1, requires external PHY)
- USB Host/Device/OTG
- PWM, analog input/output, timers
- QSPI memory expansion (4.1)

---

## 🆚 Teensy 4.x vs. Other Microcontrollers

| Feature                | Teensy 4.1           | Teensy 4.0         | ESP32                  | Raspberry Pi Pico      | ATmega328P (Arduino Uno) |
|------------------------|----------------------|--------------------|------------------------|------------------------|--------------------------|
| **CPU**                | Cortex-M7 @ 600 MHz  | Cortex-M7 @ 600 MHz| 2x Xtensa LX6 @ 240 MHz| 2x Cortex-M0+ @ 133 MHz| 1x AVR @ 16 MHz          |
| **RAM**                | 1 MB                 | 1 MB               | ~520 KB                | 264 KB                 | 2 KB                     |
| **Flash**              | 2 MB + QSPI          | 2 MB               | 4–16 MB (external)     | 2 MB (up to 16 MB)     | 32 KB                    |
| **Wi-Fi/Bluetooth**    | ❌ No                 | ❌ No              | ✅ Yes                 | ❌ No                  | ❌ No                    |
| **USB**                | Host/Device/OTG      | Host/Device/OTG    | Host/Device/OTG        | Device (1.1)           | ❌ No                    |
| **Ethernet**           | ✅ Yes (4.1 only)     | ❌ No              | ❌ No (some variants)  | ❌ No                  | ❌ No                    |
| **GPIO**               | 55                   | 40                 | 34+                    | 26                     | 23                       |
| **Best Use Cases**     | High-speed, audio, DSP| High-speed, compact| IoT, wireless, BLE     | Embedded, PIO, education| Basic embedded, Arduino  |

---

## ✅ Pros and ❌ Cons

### ✅ Advantages
- **Extreme Performance**: 600 MHz Cortex-M7, ideal for DSP, audio, and real-time tasks
- **Rich Peripherals**: Multiple buses, audio, CAN, Ethernet (4.1), SD, USB Host/Device
- **Arduino Compatible**: Easy for beginners, powerful for experts
- **Strong Community**: Extensive documentation, libraries, and forum support
- **Compact Form Factor**: Breadboard-friendly (4.0), more I/O (4.1)

### ❌ Disadvantages
- **No Built-in Wireless**: Lacks Wi-Fi/Bluetooth (requires external modules)
- **3.3V Logic**: Not all shields and sensors are 3.3V compatible
- **Higher Cost**: More expensive than basic MCUs like ATmega328P or Pico
- **Advanced Features Require Learning Curve**: Some features (DMA, QSPI, audio) are advanced

---

## 📦 Common Boards & Variants

- **Teensy 4.0**: Smallest, breadboard-friendly, 40 I/O pins
- **Teensy 4.1**: More I/O, SD slot, Ethernet, QSPI expansion, larger form factor

---

## 🔗 See Also

- [[ESP32]]
- [[Raspberry Pi Pico]]
- [[ATmega328P]]
- [[Microcontrollers]]
- [[Arduino]]
- [[Audio DSP]]

---

## 📚 Resources

- [PJRC Teensy 4.0 Product Page](https://www.pjrc.com/store/teensy40.html)
- [PJRC Teensy 4.1 Product Page](https://www.pjrc.com/store/teensy41.html)
- [Teensyduino (Arduino Add-on)](https://www.pjrc.com/teensy/td_download.html)
- [Teensy Forum](https://forum.pjrc.com/)
- [Teensy Audio Library](https://www.pjrc.com/teensy/td_libs_Audio.html)
- [PlatformIO Teensy Support](https://docs.platformio.org/en/latest/boards/teensy.html)

---

## 🧠 Summary

Teensy 4.x boards are among the fastest and most capable microcontroller platforms available to makers and professionals. Their combination of high performance, rich peripherals, and Arduino compatibility make them ideal for demanding embedded, audio, and real-time applications.
