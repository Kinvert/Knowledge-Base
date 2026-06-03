# Ryzen AI MAX+ 395

The **AMD Ryzen AI MAX+ 395** is a high-end Zen 5 APU, formerly known as **Strix Halo**, built for premium mobile workstations, compact desktops, and AI PCs. It combines a 16-core CPU, a large integrated Radeon GPU, an XDNA 2 NPU, and up to 128 GB of unified LPDDR5x memory.

Unlike a normal desktop CPU such as the [[Ryzen 9 9950X]] or [[Ryzen 9 9950X3D]], the Ryzen AI MAX+ 395 is interesting because the CPU, GPU, and NPU sit inside one power-managed platform with shared system memory. That makes it closer in spirit to Apple Max/Ultra-style workstation SoCs than to a socketed AM5 desktop build.

---

## 🧠 Overview

- **Architecture:** 16x Zen 5 CPU cores
- **Cores / Threads:** 16C / 32T
- **Base / Boost Clock:** 3.0 GHz / up to 5.1 GHz
- **Cache:** 16 MB L2 + 64 MB L3
- **Default TDP:** 55W
- **Configurable TDP:** 45-120W
- **Socket / Package:** FP11, soldered platform
- **Memory:** 256-bit LPDDR5x-8000
- **Max Memory:** 128 GB
- **Integrated GPU:** Radeon 8060S Graphics
- **GPU Cores:** 40 RDNA 3.5 CUs
- **GPU Frequency:** Up to 2900 MHz
- **NPU:** Ryzen AI / XDNA 2
- **AI Performance:** Up to 126 total platform TOPS, up to 50 NPU TOPS
- **PCIe:** PCIe 4.0, 16 usable native lanes
- **USB:** 2x native USB4 40Gbps

---

## 🧩 What Makes It Different

Most x86 systems split memory into CPU system RAM and separate GPU VRAM. The Ryzen AI MAX+ 395 uses high-bandwidth LPDDR5x as shared memory for the platform. AMD also supports **Variable Graphics Memory**, allowing a large portion of system memory to be assigned as VRAM on supported systems.

This matters for local AI because many models are limited more by available memory than by raw TOPS. A 128 GB Ryzen AI MAX+ 395 machine can allocate far more memory to its integrated GPU than typical consumer mobile GPUs, though software support and raw GPU throughput still matter.

---

## 🧪 Use Cases

| Use Case | Why It Fits |
|---|---|
| Local LLM inference | Large unified memory can hold bigger quantized models than many laptop GPUs |
| Vision-language models | Radeon 8060S + large memory helps with local multimodal workloads |
| Mobile workstation | Strong 16-core CPU in a power-managed platform |
| Compact AI desktop | Useful where a discrete GPU tower is too large or power hungry |
| Robotics dev workstation | Good for ROS2 builds, local perception, simulation prep, and edge AI prototyping |
| Light-to-medium content creation | CPU, iGPU, media engine, and memory bandwidth are well balanced |
| Portable game/dev machine | 40-CU integrated GPU is unusually strong for an APU |

---

## 📊 Comparison Table

| Processor / Platform | Type | CPU | GPU / Accelerator | Memory Model | Cache | Power | Best For |
|---|---|---|---|---|---|---|---|
| **Ryzen AI MAX+ 395** | APU / SoC | 16C / 32T Zen 5 | Radeon 8060S, 40 RDNA 3.5 CUs + 50 TOPS NPU | Up to 128 GB unified LPDDR5x-8000 | 80 MB total CPU cache | 55W default, 45-120W cTDP | Compact AI workstations, local LLMs, mobile dev |
| [[Ryzen 9 9950X3D]] | Desktop CPU | 16C / 32T Zen 5 | Small iGPU only; needs discrete GPU | DDR5 system RAM + discrete GPU VRAM | 144 MB total cache | 170W TDP | Gaming, creation, dev workstation with dGPU |
| [[Ryzen 9 9950X3D2]] | Desktop CPU | 16C / 32T Zen 5 | Small iGPU only; needs discrete GPU | DDR5 system RAM + discrete GPU VRAM | 208 MB total cache | 200W TDP | Cache-heavy builds, simulation, creation, gaming |
| [[Ryzen 9 9950X]] | Desktop CPU | 16C / 32T Zen 5 | Small iGPU only; needs discrete GPU | DDR5 system RAM + discrete GPU VRAM | 80 MB total cache | 170W TDP | General high-end desktop compute |
| [[Threadripper 7960X]] | HEDT CPU | 24C / 48T Zen 4 | No meaningful integrated GPU | DDR5 RDIMM + discrete GPU VRAM | 152 MB total cache | 350W TDP | Heavy compiling, simulation, multi-GPU workstations |
| Apple M4 Max | SoC | Up to 16-core CPU | Integrated Apple GPU + Neural Engine | Unified memory | Platform-specific | Laptop/workstation power envelope | macOS mobile AI/media workstation |
| NVIDIA [[DGX Spark]] | AI mini-workstation | Grace/Arm-class CPU complex | Blackwell-class NVIDIA AI accelerator | Large unified AI memory | Platform-specific | Appliance-class | CUDA-first local AI prototyping |

---

## ✅ Pros

- Strong 16-core Zen 5 CPU in a compact/mobile class platform.
- Large integrated Radeon 8060S GPU compared with ordinary laptop APUs.
- Unified memory can help local AI workloads that are VRAM-limited.
- NPU is available for supported Windows AI / Ryzen AI workloads.
- Much better iGPU capability than AM5 desktop Ryzen CPUs.
- Strong performance-per-watt when compared with socketed desktop CPUs.

---

## ❌ Cons

- Soldered platform: no AM5 socket upgrade path.
- LPDDR5x memory is not user-upgradeable in most systems.
- PCIe 4.0 and 16 usable lanes are more limited than desktop AM5 platforms.
- ROCm / Radeon AI software support can be more workload-dependent than CUDA.
- NPU acceleration is only useful when the software stack targets it.
- A desktop 9950X3D / 9950X3D2 plus a high-end discrete GPU is still stronger for many GPU-heavy workloads.

---

## 🧠 Local AI Notes

The Ryzen AI MAX+ 395 is attractive for local AI because it can ship with up to 128 GB of unified memory, and supported systems can expose a large portion of that memory as graphics memory. This is useful for quantized LLM inference, long-context experimentation, and local multimodal models.

The tradeoff is software maturity. CUDA-first tools generally favor NVIDIA GPUs. AMD's advantage here is not that the Radeon 8060S beats high-end discrete GPUs. The advantage is that the platform can offer a lot of usable memory in a compact, relatively efficient machine.

---

## 🤖 Robotics and RL Relevance

For robotics, the Ryzen AI MAX+ 395 is not a replacement for a big discrete-GPU workstation, but it is a strong compact development node:

- [[ROS2]] builds and local development
- perception prototyping with cameras and embeddings
- local LLM/RAG assistant workflows near the robot
- simulation setup and light testing
- edge AI experiments where power and size matter
- compact mobile workstation for field robotics

For heavy [[Isaac Lab]], [[MuJoCo]], RL training, large CUDA models, or multi-GPU work, a 9950X3D/9950X3D2 desktop with a discrete NVIDIA GPU remains the more conventional path.

---

## 🔧 Compatible Items

- [[LPDDR5]]
- [[AVX-512]]
- [[ROCm]]
- [[OpenCL]]
- [[ONNX]]
- [[Ryzen AI]]
- [[USB4]]
- [[PCIe]]

---

## 🔗 Related Concepts

- [[CPUs]]
- [[Ryzen 9 9950X]]
- [[Ryzen 9 9950X3D]]
- [[Ryzen 9 9950X3D2]]
- [[DGX Spark]]
- [[GPU]]
- [[Tensor Cores]]
- [[Local AI]]

---

## 📚 Further Reading

- AMD Ryzen AI MAX+ 395 product page: https://www.amd.com/en/products/processors/laptop/ryzen/ai-300-series/amd-ryzen-ai-max-plus-395.html
- AMD Ryzen AI MAX+ 395 blog: https://www.amd.com/en/blogs/2025/amd-ryzen-ai-max-395-processor-breakthrough-ai-.html
- AMD Ryzen 9 9950X3D2 announcement: https://www.amd.com/en/newsroom/press-releases/2026-4-22-amd-launches-ryzen-9-9950x3d2-dual-edition-processor.html
- AMD Ryzen 9 9950X3D product page: https://www.amd.com/en/products/processors/desktops/ryzen/9000-series/amd-ryzen-9-9950x3d.html
- Phoronix Ryzen AI MAX+ 395 Linux comparison: https://www.phoronix.com/review/ryzen-ai-max-395-9950x-9950x3d

---

## 📝 Summary

The Ryzen AI MAX+ 395 is best understood as a high-end x86 workstation APU: 16 Zen 5 cores, an unusually large integrated Radeon GPU, an AI NPU, and up to 128 GB unified memory. It is compelling for compact AI PCs, local LLM inference, portable development, and field robotics workflows. It is less compelling when the workload clearly wants a socketed desktop CPU, more PCIe lanes, upgradeable memory, or a CUDA-class discrete GPU.
