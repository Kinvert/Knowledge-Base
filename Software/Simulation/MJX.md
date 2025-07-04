# 🟢 MJX (MuJoCo eXtensions)

**MJX** is a high-performance reimplementation of the [MuJoCo](https://mujoco.org/) physics engine in **JAX**, designed for research in reinforcement learning, robotics, and differentiable physics. It is part of DeepMind's efforts to build **differentiable and composable** physics-based environments that integrate tightly with modern ML tooling.

---

## 🧠 Summary

- **MJX** stands for *MuJoCo eXtensions*.
- Rewritten MuJoCo simulator entirely in **JAX**.
- Focused on **differentiability**, **GPU/TPU acceleration**, and **just-in-time (JIT)** compilation.
- Allows **gradient-based optimization** directly through physics simulation.
- Designed for large-scale parallel environments and batch simulation.

---

## 🚀 Use Cases

- Differentiable physics simulations
- Model-based reinforcement learning (MBRL)
- System identification and inverse dynamics
- End-to-end optimization pipelines in JAX (e.g., with [[Flax]], [[Haiku]])
- Robotics simulation and motion planning

---

## ⚙️ Technical Highlights

| Feature                  | MJX                                    |
|--------------------------|-----------------------------------------|
| Language                 | Python (via JAX)                        |
| Backend                  | JAX (NumPy-like autodiff + JIT)        |
| Differentiable           | ✅ Fully differentiable dynamics         |
| Multi-core Support       | ✅                                       |
| GPU/TPU Acceleration     | ✅                                       |
| Compatible with          | [[MuJoCo]] models (`.mjcf`, `.xml`)     |
| Limitations              | No rendering or GUI (headless only)     |

---

## 🔍 Differences vs. MuJoCo

| Feature             | MuJoCo (Original)      | MJX (in JAX)                 |
|---------------------|------------------------|------------------------------|
| Language            | C/C++ (bindings in Python) | Pure Python (JAX)         |
| Differentiable      | Partially               | Fully differentiable         |
| Hardware acceleration| CPU only (some SIMD)   | CPU/GPU/TPU via XLA          |
| Rendering           | Built-in                | None (headless only)         |
| Performance         | Real-time optimized     | Batch-optimized              |
| JIT Compilation     | ❌                      | ✅                            |
| Integration         | Low-level control       | ML-native (JAX/Flax/etc.)    |

---

## 🧪 Compatibility & Inputs

- Can load standard **MuJoCo XML (.mjcf)** and **.mjb** model files.
- Currently supports only **headless simulation** (no GUI).
- Ideal for cloud-based or multi-environment setups for RL research.

---

## 🏆 Strengths

- Native JAX implementation—highly composable with ML tools.
- Enables gradient-based learning through physics.
- Extremely efficient on TPU and GPU hardware.
- Ideal for large-scale simulations and research.

---

## ⚠️ Limitations

- No real-time visualization or built-in renderer.
- Still experimental; APIs may change.
- Limited third-party tool integration compared to full MuJoCo.

---

## 🔗 Related Notes

- [[MuJoCo]]
- [[JAX]]
- [[Flax]]
- [[Reinforcement Learning]]
- [[Differentiable Physics]]
- [[Simulation Environments]]

---

## 🌐 External Links

- [DeepMind MJX GitHub Repo](https://github.com/google-deepmind/mjx)
- [MuJoCo Official Site](https://mujoco.org/)
- [MJX Paper / Blog (if available)](https://deepmind.com)

---
