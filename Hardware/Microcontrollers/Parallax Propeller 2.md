# Parallax Propeller 2

The Parallax Propeller 2 is a unique 32-bit multicore microcontroller designed for parallel processing, real-time I/O, and deterministic execution. With 8 symmetrical cores ("cogs"), it allows developers to run independent tasks concurrently, making it well-suited for complex timing-critical applications in robotics, signal processing, and control systems.

---

## 🧠 Overview

Unlike traditional microcontrollers that rely on a central core and interrupts, the Propeller 2 executes multiple threads independently in hardware, each with its own scheduler and resources. This design enables efficient, deterministic handling of time-sensitive tasks without complex real-time operating systems or interrupt management.

---

## 🧰 Key Features

- 8 symmetrical 32-bit cores (cogs), each with its own ALU, registers, and scheduler
- 512 KB shared HUB RAM + 8× 2 KB local LUT/COG RAM
- Deterministic instruction timing
- Smart pins with configurable logic for UART, SPI, PWM, ADC, DAC, etc.
- 64 smart I/O pins with individually programmable behaviors
- Clock speeds up to 320 MHz
- Native video generation (VGA, HDMI, NTSC/PAL)
- Optional external SPI flash (boot from SD also supported)
- Debugging over serial or USB with Propeller Tool or FlexProp IDE

---

## 📊 Comparison Chart

| Feature                 | Propeller 2       | Teensy 4.1       | STM32F756 Nucleo     | RP2040 (Pi Pico)     | ESP32                |
|-------------------------|-------------------|------------------|-----------------------|----------------------|----------------------|
| Cores                   | 8 (symmetric)     | 1 (Cortex-M7)    | 1 (Cortex-M7)         | 2 (Cortex-M0+)       | 2 (Xtensa LX6)       |
| Clock Speed             | Up to 320 MHz     | 600 MHz          | 216 MHz               | 133 MHz              | 240 MHz              |
| RAM                     | 512 KB HUB + local| 1 MB + ext PSRAM | 320 KB                | 264 KB               | ~520 KB              |
| FPU                     | Yes (per core)    | Yes              | Yes                   | No                   | Yes                  |
| I/O Pins                | 64 smart pins     | 55               | 114                   | 26                   | ~34                  |
| Parallelism             | Hardware multicore| Single-core      | Single-core           | Dual-core            | Dual-core            |
| Video Out               | Yes               | No               | No                    | No                   | No                   |

---

## 🏗️ Use Cases

- Robotics with deterministic, low-latency motor control
- Sensor fusion or parallel sensor polling
- Protocol emulation (custom UART/SPI/I2C/1-Wire)
- Video generation or signal modulation/demodulation
- Embedded systems requiring multiple concurrent real-time tasks
- Audio synthesis or real-time DSP

---

## ✅ Strengths

- True parallel processing with independent cores
- Deterministic timing makes real-time behavior predictable
- Smart pins abstract away low-level peripheral code
- Native video signal generation
- Great for educational and experimental designs

---

## ❌ Weaknesses

- Smaller developer ecosystem compared to STM32/ESP32
- Limited third-party library support
- Lower clock speeds and memory than some competitors
- Steeper learning curve for conventional MCU users

---

## 🧠 Core Concepts

- [[Deterministic Execution]] (Instruction-level predictability)
- [[Parallel Processing]] (Multicore concurrency model)
- [[Smart Pins]] (Configurable I/O logic on each pin)
- [[Forth]] (Language occasionally used on Propeller)
- [[Assembly Language]] (Low-level coding for performance-critical routines)

---

## 🧩 Compatible Items

- Propeller 2 Evaluation Board
- P2 Edge Module + P2 Edge Mini Breakout
- External SPI Flash for code storage
- HDMI/VGA connectors for video out
- Passive components for audio/video demos

---

## 🛠️ Developer Tools

- Propeller Tool (Windows)
- FlexProp IDE (cross-platform, open source)
- pnut (official assembler)
- PropGCC (for C/C++)
- Debug via serial over USB or PropPlug

---

## 📚 Documentation and Support

- [Official Parallax Propeller 2 Page](https://www.parallax.com/propeller-2/)
- [Propeller 2 Forum](https://forums.parallax.com/categories/propeller-2)
- [Parallax Learn Site](https://learn.parallax.com/)
- [FlexProp GitHub](https://github.com/totalspectrum/flexprop)

---

## 🧩 Related Notes

- [[Microcontrollers]]
- [[Parallel Processing]] (Multiple concurrent execution units)
- [[Smart Pins]] (Programmable I/O logic)
- [[Teensy 4.1]] (High-speed single-core alternative)
- [[STM32F756 Nucleo-144]] (Mainstream high-performance MCU)
- [[Forth]] (Programming language occasionally used on Propeller)
- [[Assembly Language]] (Often used for Propeller core routines)

---

## 🔗 External Resources

- [Propeller 2 Documentation Hub](https://docs.google.com/document/d/1SE3g0j6JtaDb7tMy0oZm4uKz4jOh2lU1-IfCEKH3Zmo)
- [YouTube: Propeller 2 Demos](https://www.youtube.com/results?search_query=Parallax+Propeller+2)
- [GitHub: FlexProp IDE](https://github.com/totalspectrum/flexprop)
- [Parallax GitHub](https://github.com/parallaxinc)

---
