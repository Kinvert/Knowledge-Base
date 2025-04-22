---
title: Small Linux-Capable Computers
tags: [hardware, embedded, linux, raspberry pi, beaglebone, gumstix, sbc]
---

# 🧠 Small Linux-Capable Computers

This page is a reference for **Single Board Computers (SBCs)** and other compact platforms that are capable of running a full Linux distribution. These are widely used in **robotics**, **IoT**, **prototyping**, and **edge computing**.

---

## 🔹 Why Use These?

- Small size & low power consumption
- Runs **Linux distros** like Debian, Ubuntu, Arch, or Yocto
- GPIO and peripheral support (I2C, SPI, UART, etc.)
- Often used in **robotics**, **custom hardware projects**, or **network appliances**

---

## 🧩 Common Boards & Modules

### 🔸 Raspberry Pi Series
- The most popular SBC line.
- Excellent community support.
- Models range from low-power Pi Zero to powerful Raspberry Pi 5.

### 🔸 BeagleBone
- Known for real-time capabilities and built-in PRUs (Programmable Real-time Units).
- Strong for industrial and robotics projects.

### 🔸 Gumstix Overo
- Tiny, modular computer-on-module (COM).
- Ideal for custom embedded hardware with expansion boards.
- Less beginner-friendly, better for custom PCBs.

### 🔸 NVIDIA Jetson Nano
- Designed for AI/ML at the edge.
- Hardware-accelerated GPU (CUDA support).
- More power-hungry but powerful.

### 🔸 Orange Pi
- Cheaper Raspberry Pi alternative.
- Decent community, some software quirks.
- Useful for cost-sensitive applications.

### 🔸 Odroid
- Performance-oriented boards.
- Offers more RAM and CPU power than typical Raspberry Pi.
- Often used for Android or Linux development.

### 🔸 Pine64
- Community-driven SBCs.
- Some support ARM64 Linux and even open-source phones.

### 🔸 UDOO
- Hybrid SBC + Arduino.
- Useful for combining microcontroller & Linux workflows.

---

## 📊 Comparison Table

| Board/Module      | CPU Architecture | RAM      | GPU             | Wireless | GPIO | Notable Features                        |
|------------------|------------------|----------|-----------------|----------|------|-----------------------------------------|
| Raspberry Pi 5   | ARM Cortex-A76   | 4GB/8GB  | Broadcom VideoCore VII | Wi-Fi, BT | ✔️   | Mainstream, great support               |
| Raspberry Pi Zero 2 W | ARM Cortex-A53 | 512MB   | VideoCore IV    | Wi-Fi, BT | ✔️   | Tiny & very low power                   |
| BeagleBone Black | ARM Cortex-A8    | 512MB    | SGX530           | Some     | ✔️   | PRUs, industrial use                    |
| Gumstix Overo    | ARM Cortex-A8    | 256–1GB  | PowerVR SGX530   | Optional | ✔️   | Modular, COM format                     |
| Jetson Nano      | ARM Cortex-A57   | 4GB      | 128-core Maxwell | Optional | ✔️   | AI/ML dev, CUDA support                 |
| Orange Pi Zero2  | ARM Cortex-A53   | 1GB      | Mali 450         | Wi-Fi    | ✔️   | Inexpensive, small                     |
| Odroid XU4       | ARM Cortex-A15/A7 | 2GB     | Mali-T628        | USB only | ✔️   | High performance SBC                    |
| Pine64 RockPro64 | ARM Cortex-A72/A53 | 2GB–4GB | Mali-T860       | Optional | ✔️   | Community-driven, open hardware        |
| UDOO x86 Ultra   | x86 Intel Quad   | 4–8GB    | Intel HD         | Wi-Fi, BT | ✔️   | x86 compatible, Arduino built-in       |

---

## Videos
- https://www.youtube.com/watch?v=X4blR5Ua3S0

---

## 🔗 See Also

- [[Raspberry Pi]]
- [[BeagleBone]]
- [[Embedded Linux]]
- [[Yocto Project]]
- [[Single Board Computers]]
- [[Gumstix Overo Notes]]
- [[Jetson Nano Projects]]

---
