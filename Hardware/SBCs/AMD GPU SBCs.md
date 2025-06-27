# 🔶 AMD SBCs with Integrated or Dedicated GPUs

While AMD is more commonly known for its CPUs and GPUs in desktops and laptops, **Single Board Computers (SBCs)** powered by AMD processors **with integrated or discrete GPUs** have emerged in recent years as compelling alternatives to ARM-based boards like the [[Jetson Nano]] or [[Raspberry Pi]] — especially when **x86 compatibility and GPU horsepower** are required.

This document focuses on AMD-powered SBCs that feature **capable GPU acceleration**, either through **integrated Radeon graphics** (APUs) or discrete GPUs, and lists their **VRAM (Video RAM)** specs where available.

---

## 🧠 Why Choose an AMD SBC?

- Full x86_64 support — runs Windows or full Linux distributions
- Open driver stack (esp. AMDGPU under Linux)
- Some models offer **Radeon Vega or RDNA2 graphics**
- More RAM and GPU VRAM than typical ARM boards
- Excellent for AI, vision, and embedded workstation applications

---

## 🧩 AMD SBC Comparison Table

| Board Name | SoC / APU | GPU Model | GPU VRAM | CPU Cores / Threads | RAM | Notable Features |
|------------|------------|------------|-----------|----------------------|------|------------------|
| UDOO BOLT V8 | AMD Ryzen V1605B | Radeon Vega 8 | Shares system RAM (up to 2GB) | 4c / 8t @ 3.6GHz | Up to 32GB DDR4 | Dual HDMI, USB-C, PCIe |
| UDOO BOLT V3 | AMD Ryzen V1202B | Radeon Vega 3 | Shares system RAM (~1GB) | 2c / 4t @ 3.2GHz | Up to 32GB DDR4 | Lower power, small footprint |
| ASRock 4X4-V1000M | AMD Ryzen V1605B | Radeon Vega 8 | Shared (configurable in BIOS) | 4c / 8t @ 3.6GHz | Up to 32GB | Commercial-grade SBC |
| LattePanda 3 Delta | Intel CPU (Not AMD) | — | — | — | — | ❌ *Not AMD* but often confused |
| Simply NUC Sequoia V8 | Ryzen Embedded V1807B | Radeon Vega 11 | Shared (up to 2GB) | 4c / 8t @ 3.8GHz | Up to 32GB | Industrial-grade, dual LAN, PCIe |
| OnLogic ML100G-40 | AMD Ryzen Embedded R1505G | Radeon Vega 3 | Shared | 2c / 4t @ 3.3GHz | Up to 32GB | Fanless, industrial rugged SBC |
| Sapphire BP-FP5 | AMD Ryzen Embedded R1000/V1000 | Vega 3–11 | Shared | 2–4c / 4–8t | Up to 32GB | Compact embedded board |

---

## 🔧 Notes on VRAM

- **Integrated GPUs** (like Vega 8, Vega 11) **do not have dedicated VRAM**. Instead, they use **shared system RAM**. Most BIOS setups allow you to **allocate up to 2GB** of RAM for GPU use.
- Discrete GPUs on SBCs are uncommon with AMD — most use **Ryzen Embedded APUs**.

---

## 🚀 Use Cases

- AI inference (PyTorch, ONNX, TensorFlow Lite)
- GPU-accelerated vision processing (OpenCV, GStreamer)
- Embedded workstation platforms
- Industrial machine vision
- Media servers or signage

---

## 🔄 Compared to Jetson Family

| Feature | AMD SBCs | Jetson Nano/Orin |
|--------|----------|------------------|
| Architecture | x86_64 | ARM |
| GPU Type | Radeon Vega / RDNA2 (APU) | NVIDIA Maxwell/Ampere |
| VRAM | Shared (0.5–2GB) | Dedicated |
| CUDA Support | ❌ | ✅ |
| OpenCL / Vulkan | ✅ | Partially |
| AI SDK Support | Moderate | Strong (TensorRT, DeepStream) |
| OS Support | Full Linux/Windows | Ubuntu-based Linux only |
| Expansion | Often better (PCIe, SATA, NVMe) | Limited |
| Power Usage | Higher (15W–35W) | Lower (5W–15W) |

---

## 🧠 Summary of Strengths

- x86 compatibility opens up broader software/hardware ecosystem
- Radeon GPUs are decent for OpenCL and Vulkan workloads
- More RAM (and usually better I/O) than ARM SBCs
- Ideal for developers needing desktop-class support in embedded

---

## ⚠️ Limitations

- No CUDA (so some AI/ML tools won't work out of the box)
- Higher power consumption than ARM boards
- Smaller community than Jetson/Raspberry Pi
- Often pricier

---

## 🔗 Related Notes

- [[Jetson Family]]
- [[CUDA]]
- [[TensorFlow]]
- [[OpenCV]]
- [[ONNX]]
- [[SBCs]]

---
