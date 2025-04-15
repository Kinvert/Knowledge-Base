---
title: Parallax Propeller
aliases: [Propeller, Propeller Chip, Parallax Propeller P1, Parallax P1]
tags: [microcontroller, hardware, parallax]
---
# Parallax Propeller (P1)

The **Parallax Propeller** is a unique multi-core microcontroller released in 2006 by **Parallax Inc.** Unlike most microcontrollers, the Propeller features **8 symmetric 32-bit cores (called COGs)** with **deterministic execution**, allowing for software-based peripherals, real-time responsiveness, and clean parallelism without interrupts.

---

## 🌟 Key Features

- **8 COGs (32-bit RISC cores)**  
  Each COG runs independently and has access to shared **hub memory**.
  
- **Deterministic, interrupt-free concurrency**  
  Ideal for signal processing, motor control, and real-time systems.

- **32 KB Shared HUB RAM**  
  Shared memory accessed in a round-robin schedule.

- **512 Bytes per COG of local RAM**  
  For private instruction and data space.

- **Clock speed:** Up to 80 MHz system clock (can be overclocked with care).

- **Built-in peripherals (implemented in software):**  
  - UART  
  - I2C  
  - SPI  
  - VGA / NTSC / PAL video output  
  - PS/2 keyboard/mouse support  
  - PWM / Servo control  
  - Quadrature encoder reading

- **Programming Languages:**
  - **[[Spin]]** (custom language by Parallax)  
  - **Assembly** (specific to the P1)  
  - [[C]] (via SimpleIDE / GCC-based tools)  
  - Community support for **PropForth**, **Tachyon Forth**, and **[[Micropython]]**

---

## 🧠 Programming Model

- No hardware interrupts; instead, tasks run on separate COGs.
- COGs communicate via **mailbox structures** in shared HUB memory.
- Each COG is programmed independently and loaded at runtime.

### Example: Starting a UART driver on a COG

```spin
CON
  _clkmode = xtal1 + pll16x
  _xinfreq = 5_000_000

OBJ
  serial : "FullDuplexSerial"

VAR
  long uartCog

PUB Start
  uartCog := cognew(serial.start(31, 30, 0, 115200), @stack)

DAT
  stack  long  32[16]
```

## ⚙️ Development Tools

| Tool                    | Description                                                     |
| ----------------------- | --------------------------------------------------------------- |
| **Propeller Tool**      | Official IDE for Windows (Spin & Assembly)                      |
| **PNut**                | Internal Parallax tool for lower-level work                     |
| **SimpleIDE**           | GCC-based toolchain with C support                              |
| **FastSpin / FlexProp** | Community-made Spin2/C compiler by Eric Smith                   |
| **PropGCC**             | GCC port for P1 (less maintained today)                         |
| **Loaders**             | Propeller-load, pnut, and third-party loaders for flashing code |

## 🧰 Common Use Cases

- Educational projects (used in schools and by hobbyists)
    
- Robotics
    
- Retro gaming and homebrew consoles
    
- Motor control
    
- Signal processing (software-based PWM, PPM, ADC)
    
- Custom peripherals (you write your own drivers!)


## 📦 Hardware & Modules

|Module|Description|
|---|---|
|**Propeller DIP-40**|Original through-hole package|
|**Propeller Proto Board**|Breakout board for development|
|**Propeller Activity Board**|Includes power regulation, audio, sensors, breadboard|
|**QuickStart Board**|Low-cost dev board with USB, EEPROM, LEDs|
|**Propeller Mini**|Compact module for embedded use|