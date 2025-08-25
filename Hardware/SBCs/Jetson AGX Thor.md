# Jetson AGX Thor

Jetson AGX Thor is NVIDIA’s newest (Aug 2025) high-end Jetson platform for “physical AI” and advanced robotics, built around the Blackwell architecture with massive FP4/FP8 throughput, fast LPDDR5X memory, PCIe Gen5, and high-speed sensor I/O (including [[QSFP28]] for 4×25 GbE). It targets humanoids, AMRs, manipulators, and real-time multimodal perception + planning stacks.

---

## 🧭 Overview

- **What it is:** A developer kit + production modules (Jetson **T5000** / **T4000**) delivering datacenter-class generative + multimodal AI at the edge.  
- **Why it matters:** ~order-of-magnitude jump over [[Jetson AGX Orin]] in both compute and memory bandwidth; adds MIG, QSFP28, and a modern SBSA/Ubuntu/Kernel stack via JetPack 7.  
- **Where it fits:** Top of the [[Jetson Family]] above Orin/NX/Nano; adjacent to [[IGX]]/Holoscan for sensor fusion and medical/industrial edge.

---

## 🧠 Core Concepts

- **Blackwell GPU + 5th-gen Tensor Cores:** Native `FP4/FP8` with a new Transformer Engine; very high tokens/sec for LLM/VLM and low-latency vision pipelines.  
- **MIG (Multi-Instance GPU):** Partition the GPU to isolate real-time tasks (e.g., control, perception) from best-effort workloads (e.g., mapping, LLM tools).  
- **High-speed I/O for sensors:** QSFP28 (4×25 GbE), 5 GbE RJ45, PCIe Gen5, and [[HSB]] (Holoscan Sensor Bridge) camera path via the [[QSFP]] slot for dense multi-camera rigs.  
- **JetPack 7 / SBSA:** Ubuntu 24.04 + Linux 6.8, CUDA/TensorRT/Isaac/Metropolis/Holoscan with server-style firmware interfaces → easier maintenance and portability.  
- **Module variants:** T5000 (max perf) and T4000 (mid-tier), same form factor + pin-compatible.

---

## 🛠️ How It Works (at a glance)

- **Compute path:** Blackwell GPU runs perception (DNNs, VLMs), planning, and LLM/VLA policies in FP4/FP8; CPU (Arm Neoverse-V3AE) handles ROS 2 nodes, middleware, and real-time control.  
- **Concurrency:** Use `MIG` + `nvpmodel`/`power modes` to pin critical graphs (e.g., visual servoing) while background tasks (mapping, loggers, agents) run concurrently.  
- **Sensor ingest:** Cameras via `HSB through QSFP` or MIPI CSI; high-bandwidth sensors via `QSFP28` (25/50/100 GbE topologies), storage via `M.2 NVMe (PCIe Gen5 x4)`.

---

## 🧩 Key Features

- Up to **~2070 TFLOPS (FP4 sparse)** on T5000; **~1200 TFLOPS (FP4 sparse)** on T4000.  
- **128 GB LPDDR5X**, 256-bit, ~273 GB/s (T5000); **64 GB** on T4000.  
- **MIG**, PVA v3, optical-flow, dual HW enc/dec; dense 4K/8K video pipelines.  
- **[[QSFP28]] (4×25 GbE)** + **5 GbE RJ45**, **PCIe Gen5**, USB-C/USB-A, CAN-FD.  
- **Power envelopes:** ~40–130 W (T5000); ~40–70 W (T4000).

---

## 📊 Comparison Chart (Jetson family + Jetson Thor)

> Metrics differ (FP4 TFLOPS vs INT8 TOPS). Treat values as **vendor reference** for relative positioning, not apples-to-apples.

| Platform | AI Compute (vendor metric) | Power (module) | Memory | GPU uArch | CPU | Notable I/O |
|---|---:|---:|---|---|---|---|
| **[[Jetson AGX Thor Dev Kit]] ([[T5000]])** | **~2070 TFLOPS FP4 (sparse)** | 40–130 W | 128 GB LPDDR5X / 256-bit / ~273 GB/s | **Blackwell** | 14-core Neoverse-V3AE | QSFP28 4×25 GbE, 5 GbE RJ45, PCIe Gen5, HSB via QSFP |
| **[[Jetson T4000]] (module)** | ~1200 TFLOPS FP4 (sparse) | 40–70 W | 64 GB LPDDR5X / 256-bit / ~273 GB/s | Blackwell | 12-core Neoverse-V3AE | Similar I/O via carrier |
| **[[Jetson AGX Orin]] (64 GB)** | up to **275 INT8 TOPS** | 15–60 W | 64 GB LPDDR5 | Ampere | 12× Cortex-A78AE | PCIe Gen4, up to 10 GbE dev-kit |
| **Jetson Orin NX (16 GB, Super)** | up to **157 INT8 TOPS** | 10–40 W | 16 GB LPDDR5 | Ampere | 8× Cortex-A78AE | PCIe Gen4, GbE |
| **Jetson Orin Nano (8 GB, Super)** | up to **67 INT8 TOPS** | 7–25 W | 8 GB LPDDR5 | Ampere | 6× Cortex-A78AE | PCIe Gen3/4 (varies), GbE |
| **[[Jetson AGX Xavier]] (64 GB)** | up to **32 INT8 TOPS** | 10–30 W | 64 GB LPDDR4x | Volta | 8× Carmel | PCIe Gen4, GbE |

---

## 🧪 Typical Robotics Use Cases

- **Humanoids & mobile manipulators:** VLA/LLM planners + multi-cam perception + whole-body control.  
- **AMRs/cobots:** Real-time sensor fusion (multi-cam + lidar/radar), 3D perception, policy inference, on-device mapping.  
- **High-bandwidth vision:** Dozens of cameras via HSB/CSI; edge analytics over 25–100 GbE.  
- **Low-latency inference:** Visual servoing, grasping, reactive navigation with MIG isolation for hard real-time loops.

---

## ✅ Strengths

- Massive **FP4/FP8** throughput for gen-AI + vision; **MIG** for mixed-criticality.  
- **[[QSFP28]]** + **PCIe Gen5** → simpler scaling to multi-sensor rigs and fast storage.  
- **JetPack 7** (Ubuntu 24.04, Linux 6.8) with SBSA → better OS/tooling support.  
- **Large unified memory** (up to 128 GB) reduces CPU↔GPU bottlenecks.

---

## ⚠️ Limitations / Gotchas

- **Power/Thermals:** 40–130 W demands robust cooling and power design.  
- **Metric mismatch:** FP4 TFLOPS vs INT8 TOPS can mislead cross-gen comparisons.  
- **Cost:** Dev kit is premium; plan BOM accordingly.  
- **Ecosystem maturity:** Some third-party carriers/sensors may lag initial releases.

---

## 🧰 Developer Tools & Software Stack

- **JetPack 7** (Ubuntu 24.04, Linux 6.8), CUDA, cuDNN, TensorRT / TensorRT-LLM, Nsight.  
- **Robotics:** [[Isaac ROS]], [[Isaac Sim]], [[Isaac Lab]], [[GR00T]] (vision-language-action).  
- **Vision/IoT:** [[DeepStream]], [[Metropolis]], [[Holoscan]] for sensor pipelines.  
- **ROS 2:** Build with `colcon`, hardware-accelerate with NITROS nodes; isolate hard-RT tasks with MIG + RT kernels.

---

## 🔌 Hardware & I/O Highlights (Dev Kit)

- **Networking:** `QSFP28 (4×25 GbE)`, `RJ45 5 GbE`; Wi-Fi 6E (M.2 Key-E).  
- **Storage:** `M.2 Key-M (PCIe Gen5 x4)` populated with `1 TB NVMe`.  
- **Displays:** `HDMI 2.0b`, `DP 1.4a`.  
- **Cameras:** `HSB camera via QSFP slot`, plus MIPI CSI on the module/carrier.  
- **Other:** `USB-C/USB-A`, `CAN-FD`, debug UART/JTAG, fan + power headers.

---

## 🧬 Variants

- **Jetson [[T5000]]** (max compute, 128 GB LPDDR5X, ~2070 FP4 TFLOPS sparse).  
- **Jetson [[T4000]]** (mid-tier, 64 GB LPDDR5X, ~1200 FP4 TFLOPS sparse).  
- **AGX Thor Dev Kit** (carrier + thermal + T5000 module + 1 TB NVMe).

---

## 🧩 Compatible Items

- **Cameras:** GMSL2/FPD-Link (via [[HSB]] or CSI partners), USB/UVC, Ethernet cameras (5–25–100 GbE ecosystems).  
- **Networking:** QSFP28 DACs/AOCs; QSFP28↔SFP28 breakouts; 25/100 GbE switches.  
- **Storage:** NVMe Gen4/5 SSDs; external USB SSDs.  
- **Robotics:** CAN-FD motor controllers, time-sync (PTP/IEEE-1588) NICs/switches, GPS/IMU over UART/CAN/Ethernet.  
- **Carriers/Enclosures:** Partner carrier boards, ruggedized chassis, PoE add-ons (as ecosystem matures).

---

## 🔍 Related Notes / Concepts

- [[Jetson Family]]  
- [[Jetson AGX Orin]]  
- [[Jetson Xavier AGX]]
- [[ROS2]] (Robot Operating System)  
- [[Isaac ROS]] (Hardware-accelerated ROS 2 packages)  
- [[Isaac Sim]] (Simulation with Omniverse)  
- [[Holoscan]] (Sensor processing pipelines)  
- [[CUDA]] (Parallel programming model)  
- [[TensorRT]] (Inference SDK)  
- [[DeepStream]] (Vision analytics)  
- [[DDS]] (Data distribution for ROS 2)  

---

## 📚 External Resources

- NVIDIA “Jetson Thor Series” overview  
- Jetson AGX Thor Developer Kit product page + datasheet  
- Jetson T5000/T4000 module datasheet  
- Jetson Orin family overview + specs  
- Jetson AGX Xavier specs  
- NVIDIA Technical Blog: “Introducing NVIDIA Jetson Thor…”

---

## 🧾 Summary

Jetson AGX Thor brings Blackwell-class FP4/FP8 performance, MIG isolation, and serious sensor I/O (QSFP28, PCIe Gen5) to robotics. If you’ve outgrown [[Jetson AGX Orin]] or need multi-camera + generative reasoning on-device, Thor is the new flagship in the Jetson stack.
