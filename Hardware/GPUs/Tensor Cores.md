# Tensor Cores

**Tensor Cores** are specialized hardware units integrated into modern NVIDIA GPUs (starting with the Volta architecture) designed to accelerate tensor (matrix-matrix) operations. They are optimized for deep learning workloads, particularly for operations like matrix multiplication and accumulation, which are fundamental to neural networks.

Tensor Cores dramatically improve performance and throughput for both training and inference by allowing mixed-precision computing and parallelizing large-scale tensor computations.

---

## 🧠 Overview

Tensor Cores perform **fused multiply-add (FMA)** operations on small matrices (e.g., 4×4 or 8×8). For example, in FP16 mode, a Tensor Core can compute:

`D = A × B + C`

with matrix A and B in half precision (FP16) and accumulation in single precision (FP32).

They are leveraged automatically in high-level libraries like [[cuDNN]], [[cuBLASLt]], and frameworks like [[TensorFlow]] and [[PyTorch]] when mixed precision training is enabled.

---

## 🔍 Architectures with Tensor Cores

| GPU Architecture | Available in GPUs       | Precision Support         |
|------------------|--------------------------|----------------------------|
| Volta            | Tesla V100              | FP16                      |
| Turing           | RTX 20 Series, T4       | FP16, INT8, INT4          |
| Ampere           | RTX 30 Series, A100     | FP16, BF16, TF32, INT8    |
| Hopper           | H100                    | FP8, BF16, TF32, FP16     |

---

## ⚙️ Capabilities

- Fused matrix multiply-accumulate (FMA)  
- Parallel execution of thousands of matrix ops  
- Mixed-precision computation (e.g., FP16 inputs, FP32 accumulation)  
- Accelerates training and inference in deep learning  
- Used internally by libraries such as [[cuDNN]], [[TensorRT]], [[cuBLASLt]]  
- Works with [[CUDA Toolkit]] for low-level access

---

## 🚀 Use Cases

- Training large neural networks (CNNs, RNNs, Transformers)  
- Inference on edge and datacenter hardware  
- Robotics and real-time perception tasks  
- Reinforcement learning (e.g., [[Isaac Gym]])  
- Scientific computing (linear algebra, simulations)

---

## 📊 Comparison Table: GPU Compute Units

| Unit Type      | Target Use Case        | Precision Support     | Optimized For             |
|----------------|------------------------|------------------------|---------------------------|
| CUDA Core      | General-purpose        | FP32/FP64              | All workloads             |
| Tensor Core    | Matrix operations      | FP16, BF16, TF32, FP8  | DL training and inference |
| RT Core        | Ray tracing            | N/A                    | Graphics rendering        |
| Tensor Unit (TPU)| Google AI workloads | BF16, INT8             | Neural network inference  |

---

## ✅ Pros

- Massive speedups for matrix-heavy computations  
- Supported natively in major ML frameworks  
- Enables real-time inference and large-scale model training  
- Hardware support for modern low-precision formats (e.g., FP8, TF32)

---

## ❌ Cons

- Requires careful memory alignment and data formatting  
- Underutilized without proper kernel design or library support  
- Only available on newer NVIDIA GPUs  
- Benefits depend on workload structure (not all models benefit equally)

---

## 🔗 Related Concepts

- [[CUDA Toolkit]]  
- [[cuBLAS]]  
- [[cuDNN]]  
- [[TensorRT]]  
- [[Mixed Precision Training]]  
- [[GPU Acceleration]]  
- [[Matrix Multiplication]]  
- [[Parallel Programming]]  
- [[PyTorch]]  
- [[TensorFlow]]  
- [[Isaac Gym]]  
- [[AMP (Automatic Mixed Precision)]]

---

## 📚 Further Reading

- [NVIDIA Tensor Cores Overview](https://developer.nvidia.com/tensor-cores)  
- Mixed precision training with Tensor Cores in PyTorch and TensorFlow  
- NVIDIA Whitepaper: Volta, Turing, Ampere architectures  
- Benchmarks comparing Tensor Core vs non-Tensor Core performance

---
