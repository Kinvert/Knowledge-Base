# MicroPython

MicroPython is a lean and efficient implementation of the Python 3 programming language optimized to run on microcontrollers and embedded systems. It brings the ease and flexibility of Python scripting to resource-constrained hardware, making rapid prototyping and embedded development more accessible.

---

## 🧠 Overview

Designed to run with limited RAM and flash, MicroPython enables developers to write clean, high-level code for devices like the STM32 series, ESP32, RP2040, and more. It includes a subset of the Python standard library and hardware-specific modules for GPIO, I2C, SPI, PWM, and others.

---

## 🧰 Key Features

- Python 3.4+ syntax and semantics (subset)
- Interactive REPL via serial or USB
- Hardware abstraction libraries for peripherals (GPIO, UART, ADC, DAC, PWM, I2C, SPI)
- Garbage collection and dynamic memory management
- Native code compilation support via `@micropython.native` decorator
- Support for asyncio-style concurrency (uasyncio)
- Built-in filesystem support on flash and external storage (e.g., SD card)
- Extensible via native modules written in C

---

## 📊 Comparison Chart

| Feature                | MicroPython       | [[CircuitPython]] | Arduino C/C++    | [[Zephyr RTOS]]   | [[FreeRTOS]]       |
|------------------------|-------------------|-------------------|------------------|-------------------|--------------------|
| Language               | Python 3 subset   | Python 3 subset   | C/C++            | C                 | C                  |
| Target Platforms       | Wide (STM32, ESP32, RP2040, etc.) | Similar, focus on Adafruit boards | Wide MCU support | Wide MCU & SoC    | Wide MCU support    |
| REPL Support           | Yes               | Yes               | No               | No                | No                 |
| Ease of Use            | High (Python)     | High              | Moderate         | Moderate          | Low (RTOS config)  |
| Real-Time Capabilities | Limited           | Limited           | Full             | Full              | Full               |
| Library Ecosystem      | Growing           | Growing           | Large            | Large             | Large              |
| Flash/RAM Footprint    | Small/Small       | Small/Small       | Very small       | Moderate          | Moderate           |

---

## 🏗️ Use Cases

- Rapid prototyping of embedded applications
- Educational robotics platforms
- Sensor data acquisition and control loops
- IoT devices with network connectivity
- Wearables and low-power devices
- Devices requiring quick iteration and dynamic scripting

---

## ✅ Strengths

- Easy to learn and write compared to low-level languages
- Interactive development via REPL accelerates debugging
- Portable across many microcontroller platforms
- Supports advanced Python features like generators and async
- Large and active community with growing third-party modules

---

## ❌ Weaknesses

- Limited real-time deterministic behavior compared to native code or RTOS
- Higher memory footprint than bare-metal C
- Some Python libraries not supported due to resource constraints
- Performance slower than compiled C/C++

---

## 🧠 Core Concepts

- [[REPL]] (Interactive prompt for live coding/debugging)
- [[uasyncio]] (MicroPython async concurrency library)
- [[Native Modules]] (C extensions for performance-critical tasks)
- [[Garbage Collection]] (Automatic memory management)
- [[Filesystem]] (Flash or SD-based storage)

---

## 🧩 Compatible Items

- STM32 MCU boards (e.g., STM32F4, STM32F7 series)
- ESP8266 and ESP32
- Raspberry Pi Pico (RP2040)
- Pyboard series (native MicroPython boards)
- Sensors, displays, and communication peripherals supporting MicroPython drivers

---

## 🛠️ Developer Tools

- Official MicroPython firmware builds and UF2 files
- `ampy` and `rshell` command-line tools for file transfer and REPL
- Thonny IDE with built-in MicroPython support
- Visual Studio Code extensions (Pymakr, PyMakr)
- uPyCraft IDE and WebREPL for ESP boards

---

## 📚 Documentation and Support

- [MicroPython Official Documentation](https://docs.micropython.org/)
- [MicroPython GitHub Repository](https://github.com/micropython/micropython)
- [MicroPython Forum](https://forum.micropython.org/)
- [MicroPython Tutorials](https://docs.micropython.org/en/latest/tutorial/index.html)

---

## 🧩 Related Notes

- [[Python]] (General programming language)
- [[CircuitPython]] (Adafruit's fork of MicroPython)
- [[RTOS]] (Real-time Operating Systems)
- [[ESP32]] (Popular MicroPython target MCU)
- [[RP2040]] (Raspberry Pi microcontroller with MicroPython support)
- [[REPL]] (Interactive console)
- [[uasyncio]] (Async programming in MicroPython)

---

## 🔗 External Resources

- [MicroPython on YouTube](https://www.youtube.com/results?search_query=micropython)
- [Awesome MicroPython - curated list](https://github.com/mcauser/awesome-micropython)
- [MicroPython Wiki](https://en.wikipedia.org/wiki/MicroPython)
- [Pyboard D series](https://shop.pycom.io/collections/boards/products/pyboard-d-series)

---
