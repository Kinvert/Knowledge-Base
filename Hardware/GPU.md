# 🔷 GPU (Graphics Processing Unit)

A **GPU (Graphics Processing Unit)** is a specialized electronic circuit originally designed to accelerate **image rendering** for display systems. Over time, GPUs have evolved into powerful parallel processors widely used in **high-performance computing (HPC)**, **deep learning**, **scientific simulations**, **cryptocurrency mining**, and **robotics**.

---

## 🧠 Summary

- Optimized for **parallel processing** of large data sets.
- Originally designed for rendering **3D graphics**.
- Now widely used for **general-purpose computing** via GPGPU.
- Common brands: **NVIDIA**, **AMD**, **Intel Arc**.

---

## 🛠️ Key Features

- Thousands of small cores for massive parallelism.
- High memory bandwidth (GDDR6, HBM2e, etc.).
- Support for compute APIs like [[CUDA]], [[OpenCL]], and [[Vulkan]].
- Suitable for training and inference in ML frameworks like [[TensorFlow]] and [[PyTorch]].
- Available in embedded, desktop, server, and cloud form factors.

---

## 🎯 Use Cases

- **Graphics rendering** in gaming, VR/AR, and video processing.
- **Machine learning / AI training & inference**.
- **Simulations** (e.g. physics, weather, robotics).
- **Scientific computing** (matrix algebra, FFTs).
- **Parallelized tasks** in cryptography and finance.
- **Edge AI** using devices like [[NVIDIA Jetson]] or [[AMD Ryzen Embedded]].

---

## 🧪 GPU Comparison Table

| GPU Model            | Cores         | VRAM           | Architecture   | Use Case                  | Notable Feature                |
|----------------------|---------------|----------------|----------------|----------------------------|--------------------------------|
| NVIDIA RTX 4090      | 16384 CUDA    | 24GB GDDR6X    | Ada Lovelace   | DL training, 3D rendering  | DLSS 3, Ray Tracing            |
| NVIDIA A100          | 6912 CUDA     | 40/80GB HBM2e  | Ampere         | Data centers, AI/ML        | Tensor Cores, NVLink          |
| NVIDIA Jetson Orin   | 2048 CUDA     | Up to 64GB     | Ampere         | Edge AI, robotics          | Embedded form factor          |
| AMD RX 7900 XTX      | 6144 CUs      | 24GB GDDR6     | RDNA 3         | Gaming, compute            | AV1 encoding, Ray Tracing     |
| Intel Arc A770       | 4096 Xe       | 16GB GDDR6     | Xe-HPG         | Consumer compute, graphics | Open-source driver ecosystem  |

---

## ✅ Strengths

- Massive parallelism ideal for matrix operations.
- Great support across major ML and simulation frameworks.
- Strong ecosystem and documentation (esp. NVIDIA).
- Widely available across price/performance tiers.

---

## ⚠️ Weaknesses

- Higher power consumption than CPUs or [[TPU]]s.
- Can be expensive at the high end.
- Latency-sensitive tasks may underperform compared to dedicated ASICs.
- Requires knowledge of specific APIs (e.g., [[CUDA]]).

---

## 🔧 Programming Models

- [[CUDA]] (NVIDIA only)
- [[OpenCL]] (vendor-neutral, less common now)
- [[Vulkan]] Compute
- [[DirectCompute]]
- [[Metal]] (Apple devices)
- [[ROCm]] (AMD's open platform)

---

## 🧩 Related Technologies

- [[TPU]]
- [[FPGA]]
- [[ASIC]]
- [[CUDA]]
- [[TensorFlow]]
- [[PyTorch]]
- [[Jetson Family]]
- [[Compute APIs]]

---

## 🌐 External Links

- [NVIDIA Developer Portal](https://developer.nvidia.com/)
- [AMD ROCm Platform](https://rocm.docs.amd.com/)
- [Intel Arc GPUs](https://www.intel.com/content/www/us/en/products/docs/arc-discrete-graphics.html)

---
