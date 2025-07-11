# JAX

**JAX** is a high-performance numerical computing library that brings together NumPy-style APIs with powerful tools like automatic differentiation (autograd), just-in-time (JIT) compilation via XLA, and GPU/TPU acceleration. It is increasingly used in machine learning, scientific computing, and reinforcement learning due to its performance and flexibility.

---

## 📚 Overview

JAX builds on NumPy's familiar interface but adds composable function transformations, such as gradients (`grad`), vectorization (`vmap`), and compilation (`jit`). It is especially useful for researchers and developers who want to experiment with custom training loops, differentiable physics simulations, and advanced RL algorithms.

In robotics and engineering, JAX is emerging as a tool for high-speed differentiable simulations and model-based reinforcement learning where fast gradient calculations are critical.

---

## 🧠 Core Concepts

- **Autograd**: Differentiation of native Python functions  
- **JIT Compilation**: Compiles Python+NumPy-like code to fast GPU/TPU kernels  
- **vmap**: Automatic vectorization of functions over batch dimensions  
- **pmap**: Parallel execution across multiple devices  
- **Functional Programming Style**: Code is often stateless and immutable  

---

## 🧰 Use Cases

- Model-based and gradient-based reinforcement learning  
- Training neural networks with custom loss functions  
- Differentiable physics simulation  
- Large-scale optimization problems  
- High-performance scientific computing  

---

## ✅ Pros

- Very fast for GPU/TPU workloads  
- Elegant handling of gradients and transformations  
- Encourages clean, stateless code  
- Scales to multi-GPU/TPU setups with little code change  
- Backed by Google and used in cutting-edge ML research  

---

## ❌ Cons

- Steep learning curve for new users  
- Functional programming style may be unfamiliar  
- Ecosystem smaller than PyTorch or TensorFlow  
- Debugging JIT-compiled code can be complex  
- Lacks high-level DL abstractions (on its own)  

---

## 📊 Comparison Table

| Feature                 | JAX         | [[PyTorch]]     | [[TensorFlow]] | [[NumPy]]       | [[CuPy]]        |
|-------------------------|-------------|-------------|------------|-------------|-------------|
| GPU Support             | Yes         | Yes         | Yes        | No          | Yes         |
| Automatic Differentiation | Yes       | Yes         | Yes        | No          | No          |
| JIT Compilation         | Yes (XLA)   | No          | Yes        | No          | No          |
| Vectorization (vmap)    | Yes         | Manual      | Limited    | No          | No          |
| Backend Flexibility     | XLA         | TorchScript | XLA        | n/a         | CUDA only   |
| Common in RL Research   | Increasing  | Very High   | Moderate   | Low         | Low         |

---

## 🤖 In a Robotics Context

| Application                   | Role of JAX                         |
|-------------------------------|-------------------------------------|
| Differentiable Simulation     | Gradient-based optimization for control  
| Model Predictive Control      | Fast optimization of system models  
| Reinforcement Learning        | Policy gradient and model-based RL  
| Inverse Kinematics            | Differentiable cost functions  
| Sim-to-Real Transfer          | Train policies that generalize  

---

## 🔧 Compatible Items

- [[Reinforcement Learning]] – JAX is often used in modern RL pipelines  
- [[PufferLib]] – Supports JAX backends for training  
- [[PettingZoo]] – Use JAX-compatible environments via wrappers  
- [[Flax]] – Neural network library built on top of JAX  
- [[Haiku]] – Deep learning library for JAX by DeepMind  
- [[Optax]] – Optimizer library for JAX models  
- [[NumPy]] – JAX mimics much of NumPy's API  

---

## 🔗 Related Concepts

- [[Reinforcement Learning]] (where JAX is increasingly used)  
- [[PufferLib]] (supports JAX and PyTorch backends)  
- [[NumPy]] (JAX is NumPy-compatible)  
- [[PettingZoo]] (works with RL pipelines that use JAX)  
- [[PyTorch]] (popular alternative to JAX for DL)  

---

## 📚 Further Reading

- [JAX Official Site](https://jax.readthedocs.io/en/latest/)  
- [JAX 101 by Google](https://jax.readthedocs.io/en/latest/jax-101/index.html)  
- [Flax (NN Library for JAX)](https://github.com/google/flax)  
- [Optax (Optimizers for JAX)](https://github.com/deepmind/optax)  
- [DeepMind JAX Ecosystem](https://github.com/deepmind)  
- [JAX vs PyTorch: ML Engineer Perspective](https://wandb.ai/wandb_fc/jax-vs-pytorch)  

---
