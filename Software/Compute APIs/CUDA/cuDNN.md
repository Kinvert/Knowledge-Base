# cuDNN

**cuDNN** (CUDA Deep Neural Network library) is a GPU-accelerated library from NVIDIA that provides highly optimized implementations of deep learning primitives. It is a key component in the deep learning ecosystem, used by frameworks such as [[TensorFlow]], [[PyTorch]], and [[MXNet]] to leverage the parallel computing power of NVIDIA GPUs.

cuDNN is part of the [[CUDA Toolkit]] but also available as a standalone install.

---

## 🧠 Overview

cuDNN is designed for high-throughput training and inference of deep neural networks. It implements efficient operations for:
- Convolution (forward, backward, gradients)
- Activation functions (ReLU, Sigmoid, Tanh)
- Normalization (BatchNorm, LayerNorm)
- Pooling (Max, Average)
- RNNs, LSTMs, GRUs (recurrent networks)
- Softmax
- Tensor transforms and layout management

It acts as a low-level backend to simplify and accelerate DNNs on NVIDIA GPUs, utilizing [[Tensor Cores]] when available.

---

## 🔑 Key Features

- High performance on NVIDIA GPUs  
- Support for mixed precision (FP32, FP16, INT8)  
- Seamless integration with popular ML frameworks  
- Automatically uses optimized algorithms per operation  
- Supports stream-based execution for pipelining  
- Handles tensor formats (NCHW, NHWC, etc.)

---

## ⚙️ Common APIs

- `cudnnConvolutionForward()` — Forward convolution pass  
- `cudnnActivationForward()` — Applies activation  
- `cudnnBatchNormalizationForwardTraining()` — BN during training  
- `cudnnRNNForwardInference()` — RNN inference pass  
- `cudnnSoftmaxForward()` — Softmax layer for classification  
- `cudnnSetTensor4dDescriptor()` — Define tensor layout

These are all C APIs and require CUDA device memory management (via `cudaMalloc`, etc.).

---

## 📊 Comparison Table

| Library         | GPU Support | Precision Support | Key Use Cases         | Vendor      |
|------------------|--------------|-------------------|------------------------|--------------|
| cuDNN            | ✅ NVIDIA    | FP32/FP16/INT8    | CNNs, RNNs, activations| NVIDIA        |
| [[OneDNN]]       | 🟡 (some GPUs)| FP32/INT8         | CPU-optimized DNN ops  | Intel         |
| [[TensorRT]]     | ✅ NVIDIA    | FP16/INT8         | Inference optimization | NVIDIA        |
| [[hipDNN]]       | ✅ AMD       | FP32              | AMD cuDNN alternative  | AMD (via ROCm)|
| [[Keras Backend]]| CPU/GPU      | Varies            | High-level framework   | TensorFlow    |

---

## 🚀 Use Cases

- Training and inference in [[TensorFlow]] and [[PyTorch]]  
- Reinforcement learning environments (e.g., [[Isaac Gym]])  
- CNNs for image classification or detection  
- RNNs and LSTMs for time-series or language models  
- Accelerated robotics perception pipelines  
- Embedded inference on Jetson platforms

---

## ✅ Pros

- State-of-the-art GPU performance for DNN workloads  
- Used in nearly all mainstream DL frameworks  
- Optimized for all NVIDIA GPU generations  
- Reduces need to write custom CUDA kernels

---

## ❌ Cons

- Closed-source binary (not open source)  
- Limited to NVIDIA hardware  
- C-style API has a steep learning curve  
- Requires manual memory management

---

## 🔗 Related Concepts

- [[CUDA Toolkit]]  
- [[Tensor Cores]]  
- [[cuBLAS]]  
- [[TensorRT]]  
- [[PyTorch]]  
- [[TensorFlow]]  
- [[Jetson Nano]]  
- [[Isaac Gym]]  
- [[Mixed Precision Training]]  
- [[CNN]]  
- [[RNN]]  
- [[Reinforcement Learning]]  
- [[GPU Acceleration]]

---

## 📚 Further Reading

- [cuDNN Developer Guide](https://docs.nvidia.com/deeplearning/cudnn)  
- CUDA Samples demonstrating cuDNN usage  
- Deep Learning framework benchmarks (cuDNN vs custom ops)  
- NVIDIA blog: cuDNN optimizations for training and inference

---
