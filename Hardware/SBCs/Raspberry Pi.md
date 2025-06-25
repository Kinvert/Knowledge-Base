# 🟣 Raspberry Pi

**Raspberry Pi** is a family of low-cost, compact single-board computers (SBCs) originally designed to promote computer science education. Over the years, Raspberry Pi boards have become widely used in robotics, IoT, home automation, media centers, and industrial applications.

---

## 🧠 Summary

- Developed by the **Raspberry Pi Foundation** (UK).
- Known for affordability, broad community support, and versatility.
- Runs Linux-based operating systems (typically **Raspberry Pi OS**, Ubuntu, etc.).
- Huge ecosystem of add-ons (HATs), accessories, and software.
- Popular in hobbyist, educational, and industrial settings.

---

## ⚙️ Common Raspberry Pi Versions

| Model             | CPU                          | RAM Options     | Connectivity                  | Notable Features                       |
|-------------------|-----------------------------|----------------|--------------------------------|----------------------------------------|
| **Raspberry Pi 1 B** | 700 MHz ARM11                | 256MB / 512MB   | 100Mbps Ethernet, 2x USB      | The original Pi                        |
| **Raspberry Pi 2 B** | 900 MHz quad-core Cortex-A7  | 1GB             | 100Mbps Ethernet, 4x USB      | First multi-core Pi                    |
| **Raspberry Pi 3 B** | 1.2 GHz quad-core Cortex-A53 | 1GB             | Wi-Fi 2.4GHz, BT 4.1, 100Mbps Ethernet | First with onboard Wi-Fi + Bluetooth |
| **Raspberry Pi 3 B+** | 1.4 GHz quad-core Cortex-A53 | 1GB             | Wi-Fi dual-band, BT 4.2, Gigabit Ethernet (via USB 2.0) | Faster network                        |
| **Raspberry Pi 4 B** | Quad-core Cortex-A72 1.5GHz  | 2/4/8GB         | True Gigabit Ethernet, USB 3, dual micro HDMI | Major upgrade in power                |
| **Raspberry Pi Zero** | 1 GHz single-core ARM11     | 512MB           | No network onboard, 1x micro USB | Ultra-compact and low-cost            |
| **Raspberry Pi Zero W** | Same as Zero + Wi-Fi + BT  | 512MB           | Wi-Fi, BT                      | Tiny wireless Pi                       |
| **Raspberry Pi 400** | Quad-core Cortex-A72 1.8GHz  | 4GB             | Same as Pi 4 B                 | Built into a keyboard                  |
| **Raspberry Pi 5** (2023) | Quad-core Cortex-A76 2.4GHz | 4/8GB           | PCIe, true Gigabit, USB 3, dual HDMI | Latest gen, significant I/O bump     |

---

## 🚀 Typical Use Cases

| Use Case                     | Suitable Model(s)                    |
|------------------------------|---------------------------------------|
| **Learning to code**          | Pi 3 B+, Pi 4, Pi 5                  |
| **Media center (Kodi)**       | Pi 4, Pi 400                         |
| **Retro gaming**              | Pi 3 B+, Pi 4                        |
| **IoT / sensor hubs**         | Pi Zero W, Pi 3 B+                   |
| **Robotics / small embedded** | Pi Zero W, Pi 3 B+                   |
| **Edge AI / vision projects** | Pi 4, Pi 5                           |

---

## 📊 Comparison to Other SBCs

| Board                 | CPU / RAM                        | Price (est) | Strengths                           | Weaknesses                             |
|-----------------------|----------------------------------|-------------|-------------------------------------|----------------------------------------|
| **Raspberry Pi 4**     | Quad-core Cortex-A72 / up to 8GB | $35-75      | Huge community, broad support       | Not real-time OS by default            |
| **BeagleBone Black**   | Single-core Cortex-A8 / 512MB    | $60         | Built-in PRUs (real-time units)     | Weaker CPU, smaller ecosystem          |
| **Odroid XU4**         | Octa-core / 2GB                  | $50-70      | More powerful CPU, eMMC support     | Smaller community, higher power usage  |
| **Jetson Nano**        | Quad-core Cortex-A57 + GPU / 4GB | $99         | CUDA GPU for AI                     | Higher cost, higher power draw         |
| **Pine64 RockPro64**   | Dual Cortex-A72 + Quad A53 / up to 4GB | $60     | PCIe slot, powerful CPU             | Less beginner-friendly software setup  |

---

## 🏆 Strengths

- 🟢 **Affordable** — Even the Pi 5 remains cost-effective.
- 🟢 **Massive community** — Countless tutorials, forums, and projects.
- 🟢 **Wide range of accessories** — HATs, cases, displays, camera modules.
- 🟢 **Good Linux support** — Raspberry Pi OS, Ubuntu, and others.

---

## ⚠️ Weaknesses

- 🔴 **Not designed for hard real-time applications** (e.g. motor control without help from a coprocessor).
- 🔴 **USB / SD card bottlenecks** on older models.
- 🔴 **Limited I/O speed compared to industrial boards**.

---

## 🔗 Related Notes

- [[SBC]]
- [[BeagleBone Black]]
- [[Jetson Nano]]
- [[Odroid XU4]]
- [[Pine64 RockPro64]]
- [[Raspberry Pi Zero W]]

---

## 🌐 External References

- [Raspberry Pi official site](https://www.raspberrypi.org/)
- [Pi 5 announcement](https://www.raspberrypi.com/news/introducing-raspberry-pi-5/)
- [Raspberry Pi PCIe Database Jeff Geerling](https://pipci.jeffgeerling.com/)

---
