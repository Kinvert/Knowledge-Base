# 🧠 tinygrad

**tinygrad** is a minimalist deep learning framework that showcases how deep learning libraries like [[PyTorch]] or [[TensorFlow]] can be implemented with only a few hundred lines of code. Created by **George Hotz (geohot)**, tinygrad started as an educational project and evolved into a surprisingly capable framework used for real workloads — including inference on custom hardware like Apple's [[M1 GPU]], GPUs via [[Metal]], [[OpenCL]], and more recently, its own kernels and backends.

---

## ✨ Overview

- **Language**: Python
- **Size**: ~1000 lines of core logic
- **Philosophy**: Understandable > fast
- **Purpose**: Learning, research, and real experimentation
- **License**: MIT

---

## 🔧 Key Features

- Tiny neural network training engine written in Python
- Custom autodiff engine (forward and backward passes)
- Works with CPUs, GPUs (via OpenCL/Metal), and custom backends
- Recently includes kernel fusion and compiler-level tricks
- Supports training and inference on common models (like MNIST, MobileNet)
- Under active development and experimentation

---

## 🧰 Supported Backends

| Backend        | Type        | Notes                                       |
|----------------|-------------|---------------------------------------------|
| Python         | CPU         | Default, very slow, great for understanding |
| OpenCL         | GPU         | Runs on AMD, Intel, and some NVIDIA GPUs    |
| Metal          | GPU (Apple) | Optimized for macOS/iOS hardware            |
| CUDA (WIP)     | GPU         | Limited community support                   |
| Compiled Kernels | CPU/GPU    | Generated at runtime, good performance      |
| TinyJIT        | JIT/Compiler| Runtime kernel fusion, graph optimization   |

---

## 🚀 Use Cases

- Education: learn autodiff and neural networks from scratch
- Benchmarking: explore optimization tradeoffs
- Hardware testing: explore backend implementations (e.g., M1 GPU)
- Experimental research: compiler design, kernel generation

---

## 📊 Comparison to Other Frameworks

| Framework     | Size/Complexity | Use Case             | Performance       | Autograd? | GPU Support | Notable For                  |
|---------------|------------------|-----------------------|-------------------|------------|--------------|------------------------------|
| tinygrad      | 🔹 Tiny (~1k LOC) | Learning, tinkering   | Medium–High (w/ JIT) | ✅         | ✅ (OpenCL/Metal) | Simplicity, extensibility   |
| [[PyTorch]]   | 🔸 Large          | Full production usage | Very High         | ✅         | ✅           | Ecosystem, flexibility      |
| [[TensorFlow]]| 🔸 Large          | Production/Research   | Very High         | ✅         | ✅           | Graph optimizations          |
| [[Keras]]     | 🔸 Medium         | High-level ML         | High              | ✅         | ✅           | Ease of use                 |

---

## ✅ Strengths

- Extremely easy to read and modify
- Great for learning how DL frameworks work
- Active community and experimentation
- Support for real inference workloads
- Encourages understanding over abstraction

---

## ❌ Weaknesses

- Not designed for large-scale production
- Limited documentation
- Performance varies greatly depending on backend
- Rapidly evolving — may be unstable

---

## 🔗 Related Projects and Concepts

- [[PyTorch]]
- [[TensorFlow]]
- [[Keras]]
- [[ONNX]]
- [[M1 GPU]]
- [[Compute APIs]]
- [[Autodiff]]

---

## 🌐 External Links

- [tinygrad GitHub](https://github.com/geohot/tinygrad)
- [tinygrad Discord](https://discord.gg/tinygrad)
- [Blog post from geohot](https://geohot.github.io/blog/jekyll/update/2020/12/28/tinygrad.html)
- [YouTube Talks / Coding Streams](https://www.youtube.com/@georgehotz)

---
