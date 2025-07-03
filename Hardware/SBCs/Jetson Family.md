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

## 🎥 Jetson Family Camera Interface Comparison

This section focuses on the Jetson modules' camera capabilities, particularly **CSI-2 lane count**, **ribbon cable support**, and **compatibility with common camera modules** like IMX219, IMX477, Arducam, and others. Camera support is a crucial factor in vision-based AI applications.

| Module               | CSI-2 Interfaces | Max CSI Lanes | Ribbon Connectors on Dev Kit | Supported Cameras         | Notes                                                                 |
|----------------------|------------------|----------------|------------------------------|----------------------------|------------------------------------------------------------------------|
| Jetson Nano 4GB      | 1                | 2              | 1 (15-pin)                   | IMX219, Arducam Mini, HQ   | Limited to one 2-lane camera; camera multiplexers (e.g., Arducam) required for more |
| Jetson Nano 2GB      | 1                | 2              | 1 (15-pin)                   | IMX219                     | Similar to 4GB, but lower memory limits processing capabilities       |
| Jetson Orin Nano 4GB | 1                | 4              | 1 (22-pin)                   | IMX477, HQ, Arducam 4-lane | Higher CSI bandwidth allows high-res or dual-lane cameras             |
| Jetson Orin Nano 8GB | 1                | 4              | 1 (22-pin)                   | IMX477, HQ, others         | Same as 4GB with more RAM and performance                             |
| Jetson TX2           | 3                | 6              | Varies (carrier board)       | Multiple via custom boards | Advanced with multiple CSI ports if supported by carrier              |
| Jetson Xavier NX     | 3                | 6              | 1–2 (depending on kit)       | IMX219, IMX477, Arducam    | Supports stereo and multi-cam with proper breakout                    |
| Jetson AGX Xavier    | 6                | 16             | 2+ (varies by carrier)       | Many industrial cameras    | Industrial-level multi-camera support; often used with custom carrier boards |
| Jetson Orin NX       | 5                | 8              | Depends on carrier           | Up to 4× IMX477, IMX708    | Excellent multi-cam performance for smaller footprint                 |
| Jetson AGX Orin      | 8                | 16–32          | 2+ (via dev board/carrier)   | Multiple high-end cameras  | Best for full sensor suites (LiDAR, stereo, thermal, etc.)            |

---

### 🧠 Notes on Interface

- **CSI Lanes**: Each camera sensor may use 2 or 4 lanes depending on resolution and FPS.
- **15-pin vs 22-pin**: 15-pin is common on older Pis and Jetsons; newer Jetsons and cameras are switching to 22-pin for 4-lane support.
- **Camera Support**: JetPack versions affect supported cameras—some newer sensors require JetPack 5+.

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
