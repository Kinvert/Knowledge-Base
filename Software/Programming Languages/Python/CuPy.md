# CuPy

**CuPy** is a GPU-accelerated array library that has an API nearly identical to **NumPy**, enabling drop-in acceleration of numerical and scientific computing tasks using NVIDIA CUDA-enabled GPUs. It is widely used for high-performance computing, deep learning preprocessing, and GPU-accelerated robotics workflows.

---

## 📚 Overview

CuPy lets you run NumPy-style code on GPUs with minimal code changes. It supports a wide range of functions: linear algebra, FFT, random number generation, indexing, broadcasting, and more. Under the hood, CuPy translates array operations into CUDA kernels for execution on the GPU.

For robotics and AI applications involving large datasets, real-time processing, or deep learning, CuPy can provide major performance improvements without requiring low-level CUDA programming.

---

## 🧠 Core Concepts

- **cupy.ndarray**: GPU array object analogous to NumPy's `ndarray`  
- **Device Contexts**: Manual control over which GPU to use  
- **Memory Pinned Transfers**: Faster host-to-device memory copy  
- **CUDA Integration**: Can directly run custom CUDA kernels  
- **Drop-in Replacement**: Many `numpy` functions work by replacing `import numpy as np` with `import cupy as cp`  

---

## 🧰 Use Cases

- GPU-accelerated SLAM preprocessing  
- Point cloud filtering and transformation at scale  
- Real-time sensor fusion and filtering  
- Preprocessing for deep learning inference  
- Physics simulation workloads  
- Accelerated robotics control loops and planning  

---

## ✅ Pros

- Near drop-in replacement for NumPy  
- Massive speed-up on GPU-supported operations  
- Supports many common scientific computing tasks  
- Seamless integration with CUDA  
- Works well with deep learning frameworks like Chainer and PyTorch  

---

## ❌ Cons

- Requires NVIDIA GPU and CUDA drivers  
- Not all NumPy functions are supported (esp. niche APIs)  
- Debugging GPU code can be harder  
- Memory management needs attention for large workloads  
- Doesn't support all CPUs or operating systems  

---

## 📊 Comparison Table

| Feature               | NumPy        | CuPy         | PyTorch      | TensorFlow   | JAX          |
|-----------------------|--------------|--------------|--------------|--------------|--------------|
| Runs on GPU           | No           | Yes          | Yes          | Yes          | Yes          |
| NumPy-Compatible API  | Yes          | Yes          | Partial      | Partial      | Yes          |
| FFT Support           | Yes          | Yes          | No           | Yes          | Yes          |
| SciPy Compatibility   | Partial      | Limited      | No           | No           | Limited      |
| Robotics Usage        | High         | Growing      | High         | High         | Low          |

---

## 🤖 In a Robotics Context

| Application              | Role of CuPy                             |
|--------------------------|------------------------------------------|
| SLAM                     | Speeding up feature extraction and filtering  
| Sensor Fusion            | Fast Kalman filter or EKF updates on GPU  
| Point Cloud Processing   | Accelerated voxel filtering or clustering  
| Control Systems          | Matrix computations for MPC/LQR in real-time  
| AI Preprocessing         | Normalize and transform inputs before inference  

---

## 🔧 Common Functions

- `cp.array()` – Create a GPU array  
- `cp.asnumpy()` / `cp.asarray()` – Transfer data between CPU and GPU  
- `cp.dot()` / `cp.matmul()` – Matrix multiplication  
- `cp.linalg.*` – Linear algebra functions  
- `cp.fft.*` – GPU-based FFT  
- `cp.random.*` – Random number generation  

---

## 🔧 Compatible Items

- [[NumPy]] – Drop-in API compatibility  
- [[CUDA]] – Required backend for GPU execution  
- [[PyTorch]] / [[TensorFlow]] – Can interoperate via `DLPack`  
- [[Cython]] – Can integrate with custom GPU kernels for performance  
- [[Point Cloud Segmentation]] – Ideal for GPU-intensive segmentation  

---

## 🔗 Related Concepts

- [[NumPy]] (CuPy mimics its API for GPU execution)  
- [[CUDA]] (Required for CuPy backend)  
- [[Cython]] (For custom CUDA kernel integration)  
- [[Point Cloud Processing]] (Heavy GPU acceleration possible)  
- [[TensorFlow]] / [[PyTorch]] (Interoperable with CuPy arrays)  

---

## 📚 Further Reading

- [CuPy Official Website](https://cupy.dev/)  
- [CuPy GitHub Repository](https://github.com/cupy/cupy)  
- [CuPy vs NumPy Benchmarks](https://docs.cupy.dev/en/stable/benchmark.html)  
- [Using CuPy with DLPack](https://docs.cupy.dev/en/stable/user_guide/interoperability.html)  
- [Real Python Intro to CuPy](https://realpython.com/cupy-python-gpu/)  

---
