# CUDA Streams

**CUDA Streams** are sequences of operations (kernels, memory transfers, etc.) that execute in order on an NVIDIA GPU. They enable **concurrent execution** by allowing multiple streams to run asynchronously and overlap computation and data transfer, improving GPU utilization and performance.

Streams are a fundamental CUDA feature for optimizing parallel workflows in robotics, machine learning, and scientific simulations.

---

## 🧠 Overview

A CUDA stream is essentially a queue of commands executed in order. By default, all operations run in the **default stream** (also called the **null stream**) and are serialized. Using multiple streams allows:

- Overlapping kernel execution and memory transfers  
- Concurrent kernel execution (if hardware supports it)  
- Fine-grained control of GPU task scheduling  

Synchronization between streams can be controlled with events or explicit synchronization calls.

---

## 🧪 Use Cases

- Overlapping data transfer (host ↔ device) with kernel execution  
- Running multiple independent kernels concurrently  
- Pipelines where input/output tasks execute in parallel  
- Robotics sensor data processing where latency matters  
- Real-time simulations requiring high GPU throughput  
- Machine learning training with asynchronous data loading

---

## ⚙️ Capabilities

- Create and destroy streams dynamically  
- Enqueue kernels, memory copies, and other operations into streams  
- Synchronize streams with events or explicit waits  
- Stream priorities (on supported GPUs)  
- Supports CUDA APIs and runtime APIs  
- Compatible with CUDA libraries like [[cuBLAS]], [[cuFFT]], [[cuSolver]]  

---

## 📊 Comparison Table

| Feature                | Default Stream | User-Defined Stream  | Notes                             |
|------------------------|----------------|---------------------|----------------------------------|
| Order of execution     | Sequential      | Sequential per stream | Streams run concurrently          |
| Concurrency            | No              | Yes (if hardware allows) | Requires multiple streams        |
| Overlapping operations | No              | Yes                  | Useful for copy-kernel overlap   |
| Synchronization        | Implicit        | Explicit             | Use events or sync calls          |

---

## ✅ Pros

- Increased GPU utilization and throughput  
- Allows overlapping compute and data transfer  
- Enables complex concurrent workflows  
- Fine control over GPU task scheduling  
- Supported by all recent NVIDIA GPUs and CUDA versions

---

## ❌ Cons

- More complex programming model  
- Requires careful synchronization to avoid race conditions  
- Hardware limits max concurrent streams/kernels  
- Debugging concurrency issues can be challenging

---

## 🔗 Related Concepts

- [[CUDA Toolkit]]  
- [[CUDA Kernels]]  
- [[cuBLAS]]  
- [[cuFFT]]  
- [[cuSolver]]  
- [[PyCUDA]]  
- [[GPU Computing]]  
- [[Asynchronous Programming]]  

---

## 📚 Further Reading

- [CUDA Streams Documentation (NVIDIA)](https://docs.nvidia.com/cuda/cuda-c-programming-guide/index.html#streams)  
- [CUDA C++ Programming Guide](https://docs.nvidia.com/cuda/cuda-c-programming-guide/index.html)  
- [CUDA Stream Example](https://developer.nvidia.com/blog/how-overlap-data-transfers-cuda-cc/)  
- [PyCUDA Streams](https://documen.tician.de/pycuda/driver.html#pycuda.driver.Stream)  

---
