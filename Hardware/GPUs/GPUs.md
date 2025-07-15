# GPUs

**GPUs** (Graphics Processing Units) are highly parallel processors originally designed for graphics rendering but now widely used for general-purpose computation, especially in fields requiring high-throughput numerical processing. In robotics, AI/ML, scientific computing, simulation, and embedded systems, GPUs enable real-time processing and acceleration of tasks that would otherwise be infeasible on CPUs.

This file serves as an index for topics and tools related to GPUs, their architectures, APIs, and use in robotics and engineering workflows.

---

## 🧠 Overview

Modern GPUs can contain thousands of cores optimized for SIMD (Single Instruction, Multiple Data) execution, allowing them to execute massive numbers of operations in parallel. While traditional CPUs are optimized for single-threaded performance, GPUs shine when performing repeated operations across large data arrays (e.g., matrix multiplication, image processing, neural network inference).

GPU computation is leveraged using APIs and toolkits like [[CUDA Toolkit]], [[OpenCL]], [[Vulkan Compute]], and more.

---

## 🧩 Major Vendors and Architectures

| Vendor   | Architecture Families                  | Notes                                 |
|----------|-----------------------------------------|----------------------------------------|
| NVIDIA   | Tesla, Quadro, GeForce, Jetson, A100    | CUDA support, Tensor Cores, cuDNN      |
| AMD      | Radeon, Instinct, Ryzen APUs            | ROCm/HIP ecosystem, OpenCL             |
| Intel    | Iris, Arc, Xe, Integrated GPUs          | oneAPI, SYCL support                   |
| Apple    | M1/M2 GPU                               | Metal compute API                      |
| Google   | TPU (not a GPU, but related)            | Matrix-focused AI acceleration         |

---

## 🧠 Common GPU APIs and Toolkits

- [[CUDA Toolkit]] — NVIDIA's GPU compute platform  
- [[cuBLAS]] — Linear algebra on GPUs  
- [[cuDNN]] — Deep learning acceleration  
- [[TensorRT]] — Optimized inference runtime  
- [[OpenCL]] — Open standard for cross-platform GPGPU  
- [[ROCm]] — AMD’s GPU compute stack  
- [[SYCL]] — OpenCL-based C++ abstraction (used by Intel)  
- [[Vulkan Compute]] — Low-level compute on GPU via Vulkan  
- [[DirectML]] — Windows-native ML API  
- [[Metal Performance Shaders]] — Apple’s GPU compute API  

---

## 🧮 Use Cases

- Training deep neural networks  
- Inference acceleration in edge/robotics systems  
- Visual SLAM and real-time image processing  
- Simulation environments like [[Isaac Gym]], [[Gazebo]] (w/ GPU support)  
- Finite Element simulations and [[CFD]]  
- Physics-based rendering (e.g. ray tracing)

---

## 📊 Comparison Table: GPUs for Engineering Workloads

| GPU Model         | Vendor  | Tensor Cores | CUDA Cores | Target Use            | Notes                         |
|-------------------|---------|---------------|------------|------------------------|-------------------------------|
| A100              | NVIDIA  | ✅             | 6912       | AI/Scientific HPC      | Used in datacenters           |
| RTX 4090          | NVIDIA  | ✅             | 16384      | ML training + gaming   | Consumer-grade beast          |
| Jetson Orin NX    | NVIDIA  | ✅             | 1024       | Embedded AI/robotics   | Compact edge GPU              |
| Radeon Instinct MI100 | AMD  | ❌             | 7680       | HPC + ML (via ROCm)    | AMD’s CUDA rival              |
| Intel Arc A770    | Intel   | ❌             | 512 EU     | Entry-level compute    | oneAPI, limited adoption      |
| Apple M2 GPU      | Apple   | ❌             | ~10-core   | Mobile AI + Metal      | ML acceleration on macOS      |

---

## 🔧 Related GPU Concepts

- [[CUDA]]  
- [[Tensor Cores]]  
- [[cuBLAS]]  
- [[cuDNN]]  
- [[nvcc]]  
- [[GPU Acceleration]]  
- [[OpenCL]]  
- [[ROCm]]  
- [[PTX]]  
- [[Mixed Precision Training]]  
- [[Kernel Launch]]  
- [[Warp (GPU)]]  
- [[TensorFlow]]  
- [[PyTorch]]  
- [[Isaac Gym]]  
- [[Deep Learning]]  
- [[CFD]]  
- [[Finite Element]]
- [[Hardware]]

---

## 📚 Further Reading

- [NVIDIA Developer Zone](https://developer.nvidia.com)  
- [AMD ROCm Documentation](https://rocmdocs.amd.com)  
- [Intel oneAPI](https://www.intel.com/oneapi)  
- [GPU Benchmarks for AI/ML](https://paperswithcode.com)  
- Books: *Programming Massively Parallel Processors*, *CUDA by Example*

---
