# 🍓 Raspberry Pi 5

The **Raspberry Pi 5** is the most powerful Raspberry Pi to date, offering significant improvements in CPU, GPU, RAM, and I/O over its predecessors. It’s designed to bridge the gap between hobbyist SBCs and professional edge computing applications, and is now a viable option for vision-based robotics, light AI workloads, and even limited edge compute use cases.

---

## 🧠 Overview

| Feature            | Spec                                                                 |
|--------------------|----------------------------------------------------------------------|
| CPU                | Quad-core 64-bit Arm Cortex-A76 @ 2.4GHz                             |
| GPU                | VideoCore VII                                                        |
| RAM                | 4GB or 8GB LPDDR4X-4267 SDRAM                                         |
| Storage            | microSD (UHS-I) + PCIe Gen 2 ×1 via FFC (for NVMe, etc.)             |
| Camera Support     | **Dual 4-lane MIPI CSI-2** connectors (independent pipelines)        |
| Display            | Dual 4-lane MIPI DSI                                                 |
| USB                | 2× USB 3.0, 2× USB 2.0                                                |
| Networking         | Gigabit Ethernet (w/ PoE HAT support), WiFi 5, Bluetooth 5.0         |
| Expansion          | 40-pin GPIO, PCIe (via FFC adapter), UART, I2C, SPI, PWM             |
| Power              | USB-C (PD/QC), 5V/5A recommended                                     |
| OS Support         | Raspberry Pi OS, Ubuntu 22.04/24.04, others                          |

---

## 📷 CSI Camera Support

- **Two separate 4-lane MIPI CSI-2 connectors**
- Allows for **independent operation** of two camera modules (ideal for stereo vision!)
- Compatible with:
  - Raspberry Pi Camera V3
  - HQ Camera
  - Arducam IMX series (including global shutter variants)
  - Stereo camera setups via the [[StereoPi]] HAT or Arducam stereo adapters

---

## 🧰 Key Highlights

- PCIe expansion enables NVMe SSDs or custom peripherals
- Faster USB 3.0 and true Gigabit Ethernet (unlike RPi 4's shared USB/Ethernet lane)
- Great thermals with optional active cooler
- Camera support now competitive with many robotics-grade SBCs
- Full Ubuntu 22.04/24.04 support with ROS2 compatibility

---

## 🔍 Comparison with Other Popular SBCs

| Feature               | Raspberry Pi 5        | Jetson Orin Nano 8GB | RDK X5             | BeagleV Ahead       |
|----------------------|------------------------|------------------------|---------------------|----------------------|
| CPU                  | 4× Cortex-A76 @ 2.4GHz | 6× Cortex-A78AE        | 8× Cortex-A55       | 4× Cortex-A55        |
| GPU / AI             | VideoCore VII          | 40 TOPS GPU            | 10 TOPS BPU         | NPU (3.2 TOPS)       |
| RAM                  | 4–8GB LPDDR4X           | 8GB LPDDR5              | 4–8GB LPDDR4         | 4GB LPDDR4            |
| CSI Camera Ports     | **2× 4-lane MIPI CSI-2**| 1–2 CSI-2 (via adapter) | 2× 4-lane CSI-2      | 1× MIPI CSI           |
| PCIe                 | Gen 2 x1 (via FFC)      | Gen 3 x4                | Yes (expansion HATs) | Yes (M.2 slot)        |
| Storage              | microSD + PCIe          | microSD + eMMC         | microSD + NAND       | microSD + eMMC        |
| USB                  | 2× USB 3.0, 2× USB 2.0   | USB 3.0                 | USB 3.0 + Type-C     | USB 3.0 + Type-C      |
| Networking           | GbE, WiFi 5, BT 5.0     | GbE, WiFi 6             | GbE + PoE, WiFi 6    | GbE, WiFi 6, BT 5.2   |
| Price Tier           | Low–Mid                 | Mid–High                | Mid                  | Mid                   |
| ROS2 Support         | ✅                       | ✅                      | ✅                   | ✅                    |

---

## ✅ Pros

- Very affordable and well-documented
- Dual camera support is finally standard
- Strong general-purpose performance
- Expanding support for PCIe NVMe storage
- Active community, ecosystem, and accessories

---

## ⚠️ Cons

- No onboard AI accelerator (requires Coral TPU, Movidius, etc.)
- Limited RAM options (only up to 8GB)
- PCIe requires breakout board, not native M.2
- Still no built-in eMMC storage option

---

## 🔧 Example Use Cases

- Stereo vision or depth estimation
- SLAM prototyping (with added Coral or Movidius)
- ROS2 robotics projects (lidar, cameras, motors)
- Educational and affordable autonomous vehicles
- Mobile video streaming or security surveillance

---

## 🔗 Related Notes

- [[Stereo Cameras]]
- [[MIPI CSI-2 Protocol]]
- [[Arducam Multi Camera Adapter Module V2.2]]
- [[Edge Computing]]
- [[Jetson Family]]
- [[RDK X5]]
- [[ROS2]]
- [[Raspberry Pi Camera]]

---
