# Nx (Numerical Elixir)

Nx (Numerical Elixir) is a foundational numerical computing library for the Elixir ecosystem. It provides tensor operations, automatic differentiation, and just-in-time (JIT) compilation capabilities similar to [[NumPy]] and [[JAX]], enabling Elixir developers to perform high-performance numerical computation natively on the BEAM.

---

## ⚙️ Overview

Nx brings numerical computing and machine learning primitives to Elixir. It provides a unified tensor interface and computation graph abstraction that powers higher-level frameworks such as [[Axon]] for deep learning and [[Bumblebee]] for pretrained model usage. Nx is designed for extensibility and integrates with hardware accelerators through pluggable backends.

---

## 🧠 Core Concepts

- **Tensors** – Multidimensional arrays that are the core data structure of Nx, similar to arrays in [[NumPy]] or tensors in [[PyTorch]].  
- **Backends** – Nx operations can execute on different computation engines such as [[EXLA]] (XLA backend) or [[Torchx]] (LibTorch backend).  
- **JIT Compilation** – Functions defined with `defn` can be compiled and optimized for performance on CPU, GPU, or TPU.  
- **Automatic Differentiation** – Nx provides gradient computation support for building and training models (critical for [[Axon]]).  
- **Functional API** – Immutability and composability are emphasized, consistent with Elixir’s design philosophy.  

---

## 🧩 How It Works

Nx defines tensor operations in a backend-agnostic way. Computations written using `Nx.Defn` are compiled into optimized backend-specific code. For example, when using [[EXLA]], Nx compiles functions via Google’s XLA compiler, achieving performance similar to JAX or TensorFlow. 

Nx also supports native Elixir evaluation for debugging or environments where JIT compilation is unnecessary.

---

## ⚡ Key Features

- Unified tensor API
- Automatic differentiation (AD)
- Backend-agnostic computation
- JIT compilation via EXLA and other engines
- Serialization and data transfer utilities
- Support for mixed precision and batched computation
- Integration with [[Axon]] and [[Bumblebee]]

---

## 📊 Comparison Chart

| Feature / Library | Nx (Elixir) | NumPy | JAX | PyTorch | TensorFlow | Torchx |
|-------------------|-------------|--------|------|----------|-------------|---------|
| Language | Elixir | Python | Python | Python | Python | Elixir |
| Backend Support | EXLA, Torchx, Host | CPU only | XLA | ATen | XLA, Eigen | LibTorch |
| Autograd | Yes | No | Yes | Yes | Yes | Yes (via Nx) |
| JIT Compilation | Yes | No | Yes | Yes (TorchScript) | Yes | Partial |
| GPU/TPU Support | Yes | No | Yes | Yes | Yes | Yes |
| Integration with ML | [[Axon]], [[Bumblebee]] | Indirect | Native | Native | Native | Via Nx |
| API Style | Functional | Imperative | Functional | Imperative | Declarative | Functional |

---

## 🧰 Use Cases

- Numerical analysis and data processing in Elixir
- Backend computation for ML frameworks (e.g., [[Axon]])
- Deployment of JIT-compiled models in production systems
- Real-time data pipelines and sensor processing in robotics
- Differentiable programming within Elixir

---

## ✅ Strengths

- Fully integrated with Elixir and BEAM concurrency model  
- Extensible backend design (e.g., [[EXLA]], [[Torchx]])  
- High performance via JIT compilation  
- Composable and functional API design  
- Forms the foundation for Elixir ML ecosystem (e.g., [[Axon]], [[Bumblebee]])  

---

## ❌ Weaknesses

- Limited ecosystem maturity compared to Python ML stack  
- Documentation and community still growing  
- Performance depends on backend maturity  
- Fewer high-level utilities out-of-the-box compared to [[NumPy]] or [[JAX]]

---

## 🧱 Compatible Items

- [[Axon]] (Neural network library for Elixir)  
- [[Bumblebee]] (Pretrained models for Nx and Axon)  
- [[EXLA]] (XLA backend for Nx)  
- [[Torchx]] (LibTorch backend for Nx)  
- [[Elixir]] (Programming language)  
- [[Phoenix]] (Web framework, often paired for serving models)

---

## 🔗 Related Concepts / Notes

- [[Axon]] (Elixir neural network library)  
- [[EXLA]] (Accelerated Linear Algebra backend)  
- [[Torchx]] (LibTorch integration)  
- [[Bumblebee]] (Pretrained model interface)  
- [[Autograd]] (Automatic differentiation)  
- [[JAX]] (Functional ML library)  
- [[NumPy]] (Numerical Python library)  

---

## 🧭 External Resources

- GitHub: https://github.com/elixir-nx/nx  
- Documentation: https://hexdocs.pm/nx  
- Dashbit blog introducing Nx: https://dashbit.co/blog  
- ElixirConf talks on Nx and Axon development  

---

## 🧰 Developer Tools

- `Nx.Defn` for defining tensor functions  
- `Mix` for project and dependency management  
- `IEx` for interactive tensor computation  
- `Nx.Serving` for serving models in production  
- Integration with `Livebook` for visual experimentation  

---

## 📚 Summary

Nx serves as the numerical backbone of the Elixir ML ecosystem, enabling high-performance tensor computation, JIT compilation, and differentiation directly within Elixir. It provides the foundation for machine learning frameworks like [[Axon]] and pretrained model integrations like [[Bumblebee]], bringing modern ML capabilities to the BEAM’s robust and fault-tolerant environment.

---
