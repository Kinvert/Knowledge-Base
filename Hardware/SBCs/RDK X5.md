# 🤖 RDK X5 Robot Development Kit

The **RDK X5** is a compact yet powerful ROS-ready edge AI development board intended for robotics, autonomous systems, and vision-heavy embedded applications. Developed by D-Robotics, it's built around the Sunrise 5 SoC with an integrated 10-TOPS AI accelerator and supports dual MIPI CSI-2 camera inputs, PoE, and Ubuntu 22.04.

---

## 🧠 Overview

- **Processor**: Sunrise 5 SoC (8× ARM Cortex-A55 @ 1.5GHz)
- **AI Accelerator**: 10 TOPS BPU (dedicated for vision & deep learning)
- **RAM**: 4GB or 8GB LPDDR4
- **Storage**: microSD + 1Gb NAND Flash
- **Camera Support**: Dual 4-lane MIPI CSI-2 ports (suitable for stereo vision)
- **I/O**:
  - USB 3.0 × 4
  - USB-C (for power, flash, UART, and debugging)
  - Gigabit Ethernet (supports PoE via optional HAT)
  - WiFi 6 & Bluetooth 5.4
  - HDMI, MIPI-DSI (display)
  - 28 GPIOs, CAN FD
- **Power Supply**: 5V/5A USB-C or PoE
- **OS Support**: Ubuntu 22.04 with full ROS2 compatibility

---

## 🧰 Key Features

- **Stereo Vision Ready**: Dual CSI ports allow real-time stereo/depth processing using modules like SC230AI.
- **AI Acceleration**: Optimized BPU suitable for running object detection, segmentation, SLAM, etc.
- **ROS2 Friendly**: Prebuilt images include ROS2 setup for robotics development.
- **Compact Design**: Ideal for embedded platforms and mobile robots.
- **"Flash Connect"**: A single USB-C port for flashing, debugging, and communication.

---

## 📊 RDK X5 vs Similar Edge AI Boards

| Board                 | CPU/Memory            | AI Accelerator     | Camera Support        | Networking        | ROS2 Support | Price Tier |
|----------------------|------------------------|---------------------|------------------------|-------------------|--------------|------------|
| **RDK X5**           | 8×A55 / 4–8GB LPDDR4   | 10 TOPS BPU         | 2× 4-lane MIPI CSI-2   | GbE + PoE, WiFi6   | Yes          | Mid        |
| Jetson Xavier NX     | 6×A57 / 8–16GB LPDDR4  | 21 TOPS (GPU+DLAs)  | 2× CSI-2               | GbE, WiFi         | Yes          | High       |
| Jetson Orin Nano     | 4–6×A78 / 4–8GB LPDDR5 | 20–40 TOPS (GPU)    | 1–2× CSI               | GbE, WiFi         | Yes          | Mid-High   |
| Raspberry Pi + StereoPi | 4×A76 / 4GB           | None                | 2× CSI via CM4 IO Board | GbE via HAT       | Yes          | Low-Mid    |
| Luxonis OAK-D Lite   | ARM microcontroller    | Myriad X VPU        | Integrated stereo (USB) | USB-C only        | Limited      | Mid        |

---

## ✅ Pros

- Rich I/O and AI features for its size
- Full ROS2 stack and Ubuntu support out of the box
- Modern connectivity: WiFi 6, BT 5.4, CAN FD
- Stereo vision-ready (dual MIPI)
- Optional PoE and custom HATs available

## ⚠️ Cons

- Smaller ecosystem than Jetson or Raspberry Pi
- Fewer third-party accessories
- Software stack is newer, so community support is still growing
- Limited onboard storage (mostly reliant on SD)

---

## 🔧 Example Use Cases

- Real-time stereo SLAM
- Mobile robot with onboard AI inference (YOLOv8, SuperPoint, etc.)
- Autonomous drone with WiFi streaming
- Vision-guided manipulation systems

---

## 🔗 Related Notes

- [[Jetson Family]]
- [[Stereo Cameras]]
- [[MIPI CSI-2 Protocol]]
- [[Edge Computing]]
- [[ROS2]]
- [[Depth Estimation]]

---
