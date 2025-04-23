# 🧮 Compute APIs

Compute APIs provide low-level access to parallel computing resources like GPUs, enabling massive data processing, simulation, rendering, machine learning, and more. These APIs abstract the complexities of direct hardware interaction, letting developers focus on performance-critical code.

This document summarizes major compute APIs, compares them, and helps determine the best fit for your project.

---

## 🧵 Major Compute APIs

### 🔹 [[CUDA]]
- **Developer**: NVIDIA
- **Language Support**: C, C++, Python (via bindings), Fortran
- **Target Hardware**: NVIDIA GPUs
- **Use Cases**: Deep learning, simulations, scientific computing, video processing
- **Notes**: Dominant in ML frameworks like TensorFlow and PyTorch.

### 🔹 OpenCL
- **Developer**: Khronos Group
- **Language Support**: C-based kernels; wrappers for Python, C++, Java
- **Target Hardware**: Cross-vendor: AMD, Intel, NVIDIA, FPGAs, ARM, etc.
- **Use Cases**: Cross-platform compute, heterogeneous computing
- **Notes**: More portable than CUDA, but can be harder to optimize.

### 🔹 Vulkan Compute
- **Developer**: Khronos Group
- **Language Support**: C/C++
- **Target Hardware**: Cross-platform, including desktop, mobile, embedded GPUs
- **Use Cases**: Graphics + compute together; real-time rendering; gaming
- **Notes**: Low-level API; complex but powerful. Better GPU control.

### 🔹 SYCL
- **Developer**: Khronos Group
- **Language Support**: Modern C++
- **Target Hardware**: Cross-platform (supports CUDA, OpenCL backends)
- **Use Cases**: HPC, machine learning, simulations
- **Notes**: Easier than OpenCL; part of Intel oneAPI; supports data-parallel C++.

### 🔹 Metal Performance Shaders
- **Developer**: Apple
- **Language Support**: Objective-C, Swift, C++
- **Target Hardware**: Apple GPUs
- **Use Cases**: iOS/macOS compute tasks, image processing, ML
- **Notes**: Highly optimized for Apple silicon, limited to Apple devices.

### 🔹 DirectCompute
- **Developer**: Microsoft
- **Language Support**: HLSL, C++
- **Target Hardware**: DirectX-compatible GPUs (Windows only)
- **Use Cases**: Compute shaders within DirectX, GPGPU on Windows
- **Notes**: Often used in game engines alongside graphics.

### 🔹 ROCm (HIP)
- **Developer**: AMD
- **Language Support**: C++, Python (via PyTorch, TensorFlow support)
- **Target Hardware**: AMD GPUs
- **Use Cases**: Deep learning, HPC
- **Notes**: AMD’s CUDA-equivalent, supports HIP as a portable CUDA-like language.

---

## 📊 Comparison Table

| API         | Vendor     | Cross-Platform | Language(s)      | GPU Support     | Good For                          | Abstraction Level |
|-------------|------------|----------------|------------------|------------------|------------------------------------|--------------------|
| CUDA        | NVIDIA     | ❌              | C, C++, Python   | NVIDIA only      | ML, Simulations, HPC               | Low-level          |
| OpenCL      | Khronos    | ✅              | C, C++, wrappers | Most GPUs        | General-purpose, heterogeneous     | Low to mid         |
| Vulkan      | Khronos    | ✅              | C/C++            | Most GPUs        | Graphics + compute, games          | Low-level          |
| SYCL        | Khronos    | ✅              | Modern C++       | Many (via backends) | HPC, easier OpenCL alternative     | Mid-level          |
| Metal       | Apple      | ❌ (Apple-only) | Swift, Obj-C     | Apple GPUs       | ML on iOS/macOS                    | Mid to high        |
| DirectCompute| Microsoft | ❌ (Windows)    | HLSL, C++         | DirectX GPUs     | Windows gaming, shaders            | Mid to low         |
| ROCm (HIP)  | AMD        | ✅              | C++, Python       | AMD GPUs         | Deep learning, scientific compute  | Low-level          |

---

## ✅ Strengths & Weaknesses

| API         | Strengths                                                                 | Weaknesses                                                        |
|-------------|---------------------------------------------------------------------------|--------------------------------------------------------------------|
| CUDA        | Massive ecosystem, great tooling, best performance on NVIDIA              | Vendor lock-in, not cross-platform                                |
| OpenCL      | Broad hardware support, portable                                          | Verbose, slower dev workflow, less tooling support                |
| Vulkan      | Unified graphics + compute, low overhead                                  | Very complex, steeper learning curve                              |
| SYCL        | Modern C++ style, portable, easier OpenCL                                 | Still maturing, backend-dependent optimizations                   |
| Metal       | Optimized for Apple devices, simple API                                   | Apple-only                                                         |
| DirectCompute| Easy Windows integration, good for shader tasks                          | Windows-only, outdated vs newer APIs                              |
| ROCm        | Open-source, supports HIP (CUDA-like portability)                         | Limited GPU support outside AMD, ecosystem less mature than CUDA  |

---

## 🛠 Common Use Cases

- **Machine Learning / AI**: CUDA, ROCm, SYCL
- **Graphics + Compute**: Vulkan, Metal
- **Scientific Computing**: OpenCL, CUDA, ROCm
- **Embedded / Mobile**: Metal (Apple), Vulkan Compute, OpenCL
- **Cross-Platform Needs**: OpenCL, SYCL, Vulkan

---

## 🤔 When to Use What?

- **Stuck on NVIDIA?** Use **CUDA**
- **Need portability?** Try **OpenCL** or **SYCL**
- **Apple-only app?** Use **Metal**
- **Building a game?** Consider **Vulkan Compute** or **DirectCompute**
- **Working on AMD hardware?** Explore **ROCm**

---


