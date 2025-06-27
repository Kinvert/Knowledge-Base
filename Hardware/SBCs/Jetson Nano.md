# 🟢 Jetson Nano (and Orin Nano Family)

The **Jetson Nano** family by NVIDIA is a line of compact, low-power AI development boards targeted at hobbyists, educators, and edge AI developers. Over time, it expanded to include much more powerful versions like the **Jetson Orin Nano** family.

---

## 📦 Variants

| Name | RAM | eMMC | GPU | CPU | Release |
|------|-----|------|-----|-----|---------|
| Jetson Nano Developer Kit A02 | 4GB LPDDR4 | No | 128-core Maxwell | Quad-core ARM Cortex-A57 @ 1.43 GHz | 2019 |
| Jetson Nano Developer Kit B01 | 4GB LPDDR4 | No | 128-core Maxwell | Quad-core ARM Cortex-A57 @ 1.43 GHz | 2020 |
| Jetson Nano 2GB Developer Kit | 2GB LPDDR4 | No | 128-core Maxwell | Quad-core ARM Cortex-A57 @ 1.43 GHz | 2020 |
| Jetson Nano Production Module | 4GB LPDDR4 | 16GB eMMC | 128-core Maxwell | Quad-core ARM Cortex-A57 @ 1.43 GHz | 2020 |
| Jetson Orin Nano 4GB | 4GB LPDDR5 | No | 512-core Ampere w/ 16 Tensor Cores | Quad-core Cortex-A78AE | 2023 |
| Jetson Orin Nano 8GB | 8GB LPDDR5 | No | 1024-core Ampere w/ 32 Tensor Cores | Six-core Cortex-A78AE | 2023 |
| Jetson Orin Nano Dev Kit (Super Dev Kit) | 8GB LPDDR5 | 64GB eMMC | 1024-core Ampere w/ 32 Tensor Cores | Six-core Cortex-A78AE | 2023 |

---

## 🧠 Summary

- Original Nano boards were based on the **Maxwell GPU** and are entry-level AI boards.
- The **Orin Nano** boards are based on the **Ampere GPU architecture** and are significantly more powerful — bridging the gap to mid-tier Jetson boards like Xavier NX.
- The "Super Dev Kit" term is often used informally to describe the **Jetson Orin Nano Developer Kit**, which is a more complete package with better cooling and more ports.

---

## 🧩 Interfaces & Expansion

| Feature | Nano Dev Kit (A02/B01) | Orin Nano Dev Kit |
|--------|--------------------------|-------------------|
| GPIO | 40-pin header (Raspberry Pi compatible) | 40-pin |
| CSI Camera | 1–2x MIPI CSI-2 | 2x CSI (22-pin & 15-pin FFC) |
| Display | HDMI, DP | HDMI, DP |
| USB | 4x USB 3.0, micro-USB | USB 3.1 + USB-C |
| Storage | microSD | microSD + eMMC (Dev Kit only) |
| M.2 / PCIe | No (B01: PCIe x1 exposed) | M.2 Key M slot (NVMe SSD support) |
| Networking | Gigabit Ethernet | Gigabit Ethernet |

---

## 🛠️ Accessories & Add-ons

- [[Raspberry Pi Camera]] (CSI camera support)
- M.2 NVMe SSDs (for Orin Nano Dev Kit)
- USB Wi-Fi dongles (original Nano doesn't include wireless)
- Heatsinks & active fans
- GPIO expansion boards
- IMUs, depth cameras, sensors via I2C, SPI, UART

---

## 💻 Supported OS & Software

- **Jetson Nano (Maxwell)**:
  - JetPack 4.x
  - Ubuntu 18.04 based
  - Requires legacy support to install newer packages (like ROS2)
- **Jetson Orin Nano (Ampere)**:
  - JetPack 5.x and 6.x
  - Ubuntu 20.04/22.04 based
  - Supports latest [[ROS2]], [[CUDA]], [[cuDNN]], [[TensorRT]], etc.

---

## ⚖️ Comparison Summary

| Feature | Nano 2GB | Nano 4GB | Orin Nano 4GB | Orin Nano 8GB |
|--------|-----------|-----------|----------------|----------------|
| RAM | 2GB | 4GB | 4GB LPDDR5 | 8GB LPDDR5 |
| GPU | 128-core Maxwell | 128-core Maxwell | 512-core Ampere | 1024-core Ampere |
| CPU | 4x A57 | 4x A57 | 4x A78AE | 6x A78AE |
| AI Perf (TOPS) | 0.5 | 0.5 | 20 | 40 |
| Cost (USD) | ~$59 | ~$99 | ~$199 | ~$299 |

---

## 🏆 Strengths

- Very cost-effective way to get started with AI at the edge
- Strong community and tutorials (especially for Nano)
- Plug-and-play with many Raspberry Pi-compatible accessories
- Orin Nano delivers major performance gains while retaining small form factor

---

## ⚠️ Limitations

- Original Nano boards are becoming outdated (Ubuntu 18.04, JetPack 4.x)
- No built-in Wi-Fi on older boards
- USB booting and NVMe support are limited (especially on older models)
- JetPack installation and flashing can be finicky

---

## 🔗 Related Notes

- [[Jetson Family]]
- [[CUDA]]
- [[TensorRT]]
- [[ROS2]]
- [[Raspberry Pi]]
- [[AI Edge Computing]]
- [[SBCs]]

---

## 🌐 External Resources

- [Official Jetson Nano](https://developer.nvidia.com/embedded/jetson-nano)
- [Jetson Download Center](https://developer.nvidia.com/embedded/downloads)
- [JetsonHacks GitHub](https://github.com/JetsonHacks)
- [Jetson Linux SDK](https://docs.nvidia.com/jetson/)

---
