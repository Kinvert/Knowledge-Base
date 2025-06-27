# 🟢 Jetson Family Overview

The **Jetson** family from [[NVIDIA]] is a powerful lineup of embedded computing modules designed for edge AI, robotics, autonomous systems, and industrial applications. Each Jetson board integrates a GPU (with CUDA and Tensor Cores), CPU, and memory — making them highly suitable for AI inferencing at the edge.

This document summarizes the full Jetson lineup, including the **Nano**, **TX**, **Xavier**, and **Orin** families.

---

## 🧬 Jetson Lineup

### 🟢 Jetson Nano Series

- Entry-level, cost-effective boards
- Ideal for education, hobby robotics, and small AI applications
- GPU: 128-core Maxwell
- CPU: Quad-core ARM Cortex-A57
- Includes Jetson Nano 2GB, 4GB, Orin Nano 4GB/8GB, Orin Nano Developer Kit

→ See [[Jetson Nano]] for full breakdown.

---

### 🔵 Jetson TX Series (TX1, TX2, TX2 NX)

- Mid-level performance
- Ruggedized, used in drones, robotics, and industrial systems
- GPU: 256-core Maxwell (TX1) or Pascal (TX2)
- CPU: Dual Denver 2 + Quad-core ARM Cortex-A57
- Form factor: SO-DIMM, requires carrier board

---

### 🟣 Jetson Xavier Series (Xavier NX, AGX Xavier)

- High-performance models
- Ideal for robotics, smart cities, industrial inspection
- GPU: 384-core Volta with 48 Tensor Cores
- CPU: 6-core or 8-core Carmel ARMv8.2 64-bit
- AI Performance: up to 32 TOPS (NX) or 32–64 TOPS (AGX)

---

### 🔴 Jetson Orin Series (Orin NX, Orin Nano, AGX Orin)

- Flagship performance for edge AI
- Suitable for full autonomous driving stacks, advanced robotics, and ML training at the edge
- GPU: Up to 2048-core Ampere with up to 64 Tensor Cores
- CPU: Up to 12-core ARM Cortex-A78AE
- Up to 275 TOPS (AGX Orin)

---

## 🧩 Jetson Family Comparison

| Module | RAM | GPU | CPU | AI Perf (TOPS) | JetPack Support | Form Factor |
|--------|-----|-----|-----|----------------|------------------|-------------|
| Jetson Nano 4GB | 4GB LPDDR4 | 128-core Maxwell | 4x Cortex-A57 | 0.5 | 4.x | Dev Kit / Module |
| Jetson Nano 2GB | 2GB LPDDR4 | 128-core Maxwell | 4x Cortex-A57 | 0.5 | 4.x | Dev Kit |
| Jetson Orin Nano 4GB | 4GB LPDDR5 | 512-core Ampere + 16 TC | 4x A78AE | 20 | 5.x / 6.x | Module / Dev Kit |
| Jetson Orin Nano 8GB | 8GB LPDDR5 | 1024-core Ampere + 32 TC | 6x A78AE | 40 | 5.x / 6.x | Module / Dev Kit |
| Jetson TX2 | 8GB LPDDR4 | 256-core Pascal | Dual Denver2 + 4x A57 | 1.3 | 4.x | SO-DIMM |
| Jetson Xavier NX | 8GB / 16GB | 384-core Volta + 48 TC | 6x Carmel | 21 | 4.x / 5.x | SODIMM |
| Jetson AGX Xavier | 16GB / 32GB | 512-core Volta + 64 TC | 8x Carmel | 32 | 4.x / 5.x | BGA |
| Jetson Orin NX | 8GB / 16GB LPDDR5 | 1024-core Ampere + 32 TC | 6x A78AE | 70 | 5.x / 6.x | SODIMM |
| Jetson AGX Orin | 32–64GB | 2048-core Ampere + 64 TC | 12x A78AE | 200–275 | 5.x / 6.x | BGA |

---

## 🔧 Developer Tools

- [[CUDA]] and [[cuDNN]] for GPU computing
- [[TensorRT]] for inference optimization
- [[DeepStream]] SDK for video analytics
- [[ROS2]] support (especially in Orin series)
- NVIDIA Nsight tools for profiling and debugging

---

## 📚 Documentation and Support

- [Jetson Download Center](https://developer.nvidia.com/embedded/downloads)
- [Jetson Forums](https://forums.developer.nvidia.com/c/jetson-embedded-systems/)
- [JetsonHacks GitHub](https://github.com/JetsonHacks)
- [Jetson Linux SDK](https://docs.nvidia.com/jetson/)

---

## 🏆 Use Cases

- Robotics (SLAM, visual odometry, path planning)
- Smart cameras and edge vision systems
- Autonomous mobile robots (AMRs)
- Traffic monitoring and smart infrastructure
- Medical imaging devices
- AI education platforms

---

- [[SBCs]]
