---
title: ESP32
aliases: [Espressif ESP32, ESP32 Microcontroller]
tags: [Microcontrollers, WiFi, Bluetooth, Espressif]
---

# ESP32

The **ESP32** is a powerful, low-cost microcontroller with built-in **Wi-Fi** and **Bluetooth** capabilities. Designed by **Espressif Systems**, it is widely used in IoT, robotics, embedded systems, and hobbyist projects.

---

## 🔧 Key Features

- Dual-core Tensilica Xtensa LX6 (or LX7 in newer chips)
- Clock speed: up to 240 MHz
- RAM: ~520 KB SRAM
- Flash: External SPI flash (typically 4MB or 8MB)
- Connectivity:
  - **Wi-Fi**: 802.11 b/g/n
  - **Bluetooth**: v4.2 BR/EDR and BLE
- GPIO: 34+ programmable pins
- ADC, DAC, I2C, SPI, UART, CAN, PWM
- Touch sensor support
- Deep sleep mode for low power applications
- Available in many modules: ESP32-WROOM-32, ESP32-WROVER, etc.

---

## 📦 Common Development Boards

| Board Name            | Key Traits                          |
|-----------------------|-------------------------------------|
| **ESP32 DevKitC**     | Basic, widely supported dev board   |
| **ESP32-WROVER**      | With PSRAM and external antenna     |
| **TTGO T-Display**    | With onboard display                |
| **LilyGO T-Beam**     | With GPS and LoRa                   |

---

## 🛠️ Programming Options

### 🔹 ESP-IDF (Espressif IoT Dev Framework)
- Official development framework.
- C/C++ based, low-level access.
- Full control over all peripherals and RTOS features.

### 🔹 Arduino Core for ESP32
- Arduino-style programming.
- Easier for hobbyists and fast prototyping.
- Supports most Arduino libraries.

### 🔹 MicroPython
- Python interpreter running directly on the ESP32.
- Great for rapid development and scripting.
- Limited performance compared to native C/C++.

---

## 🔌 Peripherals & Protocols

The ESP32 supports a huge number of protocols:

- **UART** (up to 3)
- **I2C** (2)
- **SPI** (4)
- **CAN**
- **PWM**
- **ADC/DAC**
- **Hall effect sensor**
- **Capacitive touch sensors**

It also has built-in **Timers**, **Watchdogs**, and **DMA** support.

---

## 🧠 Use Cases

- Home automation (e.g. ESPHome, Tasmota)
- IoT devices with MQTT
- Robot controllers with wireless telemetry
- DIY smart displays
- LoRaWAN nodes (with external LoRa modules)
- Bluetooth Low Energy (BLE) sensors or gateways

---

## 📚 Resources

- [Espressif Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
- [ESP32 Arduino Core GitHub](https://github.com/espressif/arduino-esp32)
- [MicroPython ESP32 Docs](https://docs.micropython.org/en/latest/esp32/)
- [Random Nerd Tutorials](https://randomnerdtutorials.com/projects-esp32/) - Great beginner projects
- [Hackaday Projects](https://hackaday.io/projects/tag/ESP32)

---

## 🔗 See Also

- [[Microcontrollers]]
- [[I2C]]
- [[WiFi]]
- [[Bluetooth]]
- [[ESP8266]]
- [[MQTT]]
- [[LoRa]]
- [[MicroPython]]
- [[Arduino]]
- [[ESPHome]]

---
