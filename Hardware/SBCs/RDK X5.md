# 🤖 RDK X5 Robot Development Kit

The **RDK X5** (Robot Development Kit X5) by D‑Robotics / Waveshare / Horizon is a powerful ROS‑ready edge AI computer designed for robotics, autonomy, and perception applications. It features dual‑MIPI inputs for stereo vision, 10‑TOPS AI acceleration, rich I/O interfaces, and PoE support — all in a compact Ubuntu 22.04 kit.

---

## 🧠 Summary

- **SoC**: Sunrise 5 – 8× ARM Cortex‑A55 @ 1.5 GHz, 10 TOPS BPU, 32 GFLOPS GPU :contentReference[oaicite:0]{index=0}  
- **Memory**: 4 GB or 8 GB LPDDR4; storage via micro‑SD or onboard 1 Gbit NAND :contentReference[oaicite:1]{index=1}  
- **Cameras**: Dual MIPI CSI‑2 4‑lane ports — supports stereo + AI modules :contentReference[oaicite:2]{index=2}  
- **Connectivity**: USB 3.0×4, USB2.0 Type‑C, HDMI, MIPI‑DSI, Gigabit Ethernet with PoE, Wi‑Fi 6, Bluetooth 5.4, CAN FD, 28 GPIOs :contentReference[oaicite:3]{index=3}  
- **Power**: USB‑C 5 V/5 A; PoE support via HAT :contentReference[oaicite:4]{index=4}  
- **OS**: Ubuntu 22.04 (preloaded image); ROS‑friendly :contentReference[oaicite:5]{index=5}

---

## 🛠️ Key Features

- **Stereo Vision + Depth**: Dual 4‑lane MIPI CSI ports support plugin camera modules like the SC230AI 2MP stereo depth cameras :contentReference[oaicite:6]{index=6}  
- **AI Acceleration**: Onboard BPU offers 10 TOPS for inference on vision and robotics algorithms :contentReference[oaicite:7]{index=7}  
- **“Flash Connect”**: Single USB‑C port for flashing, debugging, power, and display output :contentReference[oaicite:8]{index=8}  
- **PoE Module**: Optional HAT provides PoE IEEE 802.3af/at power and active cooling :contentReference[oaicite:9]{index=9}

---

## 📊 RDK X5 vs Similar Edge AI Boards

| Device             | CPU / RAM       | AI Accel        | CSI Inputs        | USB / Ethernet / PoE | GPU   | OS Support             | Price Tier |
|--------------------|------------------|------------------|--------------------|------------------------|--------|------------------------|------------|
| **RDK X5**         | 8×A55, 4–8 GB LPDDR4 | 10 TOPS BPU       | 2×4‑lane MIPI CSI | USB 3 x4, GbE+PoE, Wi‑Fi6, BT5.4 | 32 GFLOPS | Ubuntu 22.04 + ROS    | Mid-range  |
| Jetson Xavier NX   | 6×A57, 8–16 GB LPDDR4 | 21 TOPS | 2×4‑lane MIPI CSI | USB 3 x2, GbE       | 384-core Volta + 48TC | JetPack 5.x + ROS2 | High-end |
| Jetson Orin Nano   | 4–6×A78AE, 4–8 GB LPDDR5 | 20–40 TOPS | 2×CSI (varies) | USB 3.1, GbE        | 512–1024-core Ampere | JetPack 6.x + ROS2 | Mid-high |
| Raspberry Pi 5 CM4 + StereoPi | 4×Cortex‑A76  | None (CPU only)   | 2×CSI via CM4        | USB 3, GbE (hat)      | none   | Raspberry Pi OS, Ubuntu | Low-mid    |
| Luxonis OAK-D Lite | Quad‑core ARM, 1–2 GB   | Myriad X (FPGA)   | None (USB Stereo)   | USB‑C only            | Myriad X NCS2 | Ubuntu/ROS              | Mid       |

---

## ✅ Pros & ⚠️ Cons

### ✅ Pros
- Rich hardware: stereo vision, PoE, AI accelerators, CAN FD, display and audio
- Plug-and-play Ubuntu + ROS environment
- “Flash Connect” simplifies development
- Suitable for robotics, industrial edge, stereo perception

### ⚠️ Cons
- No embedded NVMe / PCIe (storage via micro-SD/NAND)
- Linux-only (no RTOS or bare-metal)
- Developer ecosystem is smaller than NVIDIA’s Jetson platform
- BPU APIs and community tools are less mature than CUDA/TensorRT ecosystems

---

## 🧰 Ideal Use Cases

- **Stereo visual SLAM** and depth processing with dual CSI inputs
- ROS‑based robotics: manipulation, navigation, perception
- On-device AI inference (transformers, YOLO, optical flow, stereo depth)
- Autonomous drones or mobile robots needing Wi‑Fi 6, PoE, CAN FD

---

## 🔗 Related Notes

- [[Jetson Family]]
- [[Stereo Cameras]]
- [[MIPI CSI-2 Protocol]]
- [[Edge Computing]]
- [[ROS2]]
- [[Depth Estimation]]
- [[SBCs]]

---

## 🌐 External References
- RDK X5 official site – CPU, BPU, features :contentReference[oaicite:10]{index=10}  
- DFRobot / RDK X5 specs – connectivity, Ubuntu 22.04 :contentReference[oaicite:11]{index=11}  
- Waveshare PoE HAT details :contentReference[oaicite:12]{index=12}  
- SC230AI stereo camera module specs :contentReference[oaicite:13]{index=13}  

---
