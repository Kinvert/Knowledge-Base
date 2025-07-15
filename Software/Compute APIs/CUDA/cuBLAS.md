# cuBLAS

**cuBLAS** is NVIDIA’s GPU-accelerated version of the Basic Linear Algebra Subprograms (BLAS) library. It enables high-performance dense linear algebra computations on NVIDIA GPUs, making it a fundamental building block for many scientific computing and machine learning applications.

cuBLAS is part of the [[CUDA Toolkit]] and is used extensively in areas such as deep learning, simulations, and numerical optimization.

---

## 🧠 Overview

cuBLAS provides GPU-accelerated implementations of all three levels of BLAS:
- **Level 1:** Vector operations (e.g., dot product)
- **Level 2:** Matrix-vector operations
- **Level 3:** Matrix-matrix operations (e.g., GEMM — General Matrix Multiply)

It is accessible via a C API and supports both single and double precision, complex numbers, and batched operations.

---

## 🔑 Key Features

- Optimized matrix operations using GPU parallelism  
- Supports multi-stream execution and batched GEMMs  
- Works with CUDA streams and device memory  
- Compatible with other CUDA libraries (e.g., [[cuSolver]], [[cuDNN]])  
- Often used under the hood in ML frameworks like [[TensorFlow]] and [[PyTorch]]

---

## ⚙️ Example Usage

- `cublasSgemm()` — Single-precision matrix multiplication  
- `cublasDgemv()` — Double-precision matrix-vector multiply  
- `cublasCreate()` / `cublasDestroy()` — Handle initialization and cleanup  
- `cublasSetStream()` — Assign computation to a specific CUDA stream

---

## 📊 Comparison Table

| Library        | Vendor   | Backend       | GPU Support | Notes                                 |
|----------------|----------|----------------|--------------|----------------------------------------|
| cuBLAS         | NVIDIA   | CUDA           | ✅ NVIDIA    | Optimized GPU BLAS                    |
| [[BLIS]]       | Open     | CPU            | ❌           | Modular and fast on CPUs              |
| [[OpenBLAS]]   | Open     | CPU            | ❌           | High-performance CPU BLAS             |
| [[MKL]]        | Intel    | CPU (some GPU) | 🟡 Partial   | Excellent CPU support, GPU via DPC++  |
| [[hipBLAS]]    | AMD      | HIP            | ✅ AMD       | cuBLAS equivalent for AMD GPUs        |
| [[OneDNN]]     | Intel    | CPU/GPU        | 🟡           | DNN optimized, also includes BLAS ops |

---

## 🚀 Use Cases

- Matrix operations in deep learning (GEMM in [[cuDNN]])  
- Solving systems of linear equations with [[cuSolver]]  
- Large-scale simulations (e.g., physics engines, FEM)  
- Robotics: sensor fusion, optimization, state estimation  
- Scientific and numerical computing (e.g., CFD, quantum chemistry)

---

## ✅ Pros

- Extremely fast on NVIDIA GPUs  
- Reliable and mature  
- Widely supported across scientific and ML frameworks  
- Batched and strided APIs for deep learning inference/training

---

## ❌ Cons

- NVIDIA-only hardware support  
- Requires familiarity with CUDA memory management  
- Lower-level API compared to higher abstraction libraries

---

## 🔗 Related Concepts

- [[CUDA Toolkit]]  
- [[cuSolver]]  
- [[cuDNN]]  
- [[GEMM]]  
- [[PTX]]  
- [[Tensor Cores]]  
- [[PyTorch]]  
- [[TensorFlow]]  
- [[BLAS]]  
- [[Matrix Multiplication]]  
- [[GPU Acceleration]]  
- [[Linear Algebra]]  
- [[CUDA Streams]]

---

## 📚 Further Reading

- [cuBLAS Documentation](https://docs.nvidia.com/cublas/)  
- CUDA Samples using cuBLAS  
- cuBLAS vs CPU BLAS benchmarks  
- NVIDIA Dev Blog: Efficient matrix operations with cuBLAS  
- cuBLASLt for tensor core optimized operations

---
