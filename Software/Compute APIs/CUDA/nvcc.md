# nvcc

**`nvcc`** (NVIDIA CUDA Compiler) is the compiler driver used to compile CUDA programs—source files that include both host (CPU) and device (GPU) code. It wraps around standard host compilers (e.g., `gcc`, `cl`) and the NVIDIA `ptxas` assembler to generate executables or object files that include GPU kernels.

---

## 🧠 Overview

`nvcc` handles files that mix C++/C with CUDA-specific extensions (e.g., `__global__`, `__device__`, `__host__`). It's a key part of the CUDA development toolchain, often used to build high-performance GPU-accelerated applications.

Common input file extensions:
- `.cu` — CUDA source file
- `.cuh` — CUDA headers

It is part of the [[CUDA Toolkit]] and typically targets NVIDIA GPUs using the [[PTX]] intermediate representation.

---

## 🛠️ Common Commands

- `nvcc file.cu -o my_program` — Compile a single CUDA file  
- `nvcc -arch=sm_75 file.cu` — Compile for specific GPU architecture  
- `nvcc -ptx file.cu` — Compile to PTX only  
- `nvcc -c file.cu` — Compile to object file  
- `nvcc -I./include -L./lib -lmylib file.cu` — Add include and lib paths  
- `nvcc --help` — Show help options

---

## 📊 Comparison Table: GPU Compilers

| Compiler | Vendor  | CUDA Support | OpenCL | Notes                          |
|----------|---------|--------------|--------|--------------------------------|
| nvcc     | NVIDIA  | ✅ Native     | ❌ No   | Primary CUDA compiler          |
| clang    | LLVM    | ✅ via CUDA   | ✅ Yes | Often used with HIP or SYCL    |
| hipcc    | AMD     | 🟡 HIP only   | ✅ Yes | Converts CUDA to HIP for AMD   |
| dpcpp    | Intel   | ❌            | ✅ Yes | For SYCL targets               |
| gcc      | FSF     | ❌            | ❌     | Can be used as host compiler   |

---

## 🚀 Use Cases

- GPU-accelerated machine learning libraries  
- Numerical simulation (CFD, FEM)  
- Image and signal processing  
- Reinforcement learning environments (e.g., [[Isaac Gym]])  
- Robotics workloads offloaded to GPU  
- High-performance computing and scientific simulations

---

## ✅ Pros

- Deep integration with NVIDIA GPU hardware  
- Compiles mixed CPU/GPU code seamlessly  
- Supports inline PTX and fat binaries  
- Can target multiple compute capabilities

---

## ❌ Cons

- Only works with NVIDIA GPUs  
- Compilation times can be long  
- Less flexible than newer multi-vendor GPU toolchains (e.g., SYCL)

---

## 🔗 Related Concepts

- [[CUDA]]  
- [[GPU Acceleration]]  
- [[PTX]] (Parallel Thread Execution)  
- [[Isaac Gym]]  
- [[NVIDIA Driver]]  
- [[cuBLAS]]  
- [[Tensor Cores]]  
- [[CUDA Streams]]  
- [[Thrust]]

---

## 📚 Further Reading

- NVIDIA CUDA Toolkit documentation  
- Books like *CUDA by Example* or *Programming Massively Parallel Processors*  
- Online courses on Udacity or NVIDIA's own developer portal  
- CUDA Sample projects (installed with toolkit)

---
