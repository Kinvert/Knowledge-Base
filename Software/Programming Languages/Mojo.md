# Mojo (Programming Language)

Mojo is a relatively new programming language (Modular Inc.) designed to combine the ergonomics of Python with the performance of systems languages like C++ and Rust. It is heavily optimized for ML, GPU, heterogeneous compute, and high-performance numerical workloads. The core idea is: write Pythonic-looking code, get extremely fast performance and portability on modern accelerators.

---

## 🧠 Overview

Mojo is built on MLIR and targets modern accelerators. It is Python superset-ish (not fully compatible yet) that intends to be a “general purpose programming language for AI-first systems” including robotics, embedded, edge inference, and HPC.

---

## 🧩 Core Concepts

- Pythonic syntax but compiled
- Ownership model (similar-ish to Rust, but simplified)
- MLIR IR stack integration
- First-class `kernel` concept for accelerators
- Predictable perf via static types when you want them
- Gradual typing (optional)

---

## 🧮 Comparison Chart

| Item | Mojo | Python | Rust | C++ | Julia |
|---|---|---|---|---|---|
| Perf | near C++/Rust | slow unless optimized | top-tier | top-tier | high perf JIT |
| Typing | gradual | dynamic | static ownership | static | dynamic mostly |
| GPU/TPU first-class | yes | niche libs only | hardish | hardish | decent |
| Developer UX | Pythonic | excellent | steep | steep | in-between |
| Target domain | AI + systems | everything scripting | systems | systems | numerical/science |

---

## 🔧 Use Cases (likely robotics relevant)

- onboard ML inference on SBCs like Jetsons
- robotics control loops compiled, not interpreted
- high-rate perception pipelines
- real-time-ish optimization / MPC like workloads
- writing custom ops for accelerators

---

## ✅ Strengths

- near Python ergonomics
- compiles, so predictable performance
- targets accelerators natively
- no GIL problems

---

## ❌ Weaknesses

- still very early
- not full Python yet
- ecosystem extremely small compared to Python + ROS stack

---

## 🧰 Developer Tools

- `mojo` CLI compiler
- Jupyter style notebooks supported
- VSCode/Mojo extension ecosystem emerging
- MLIR debugging pipeline

---

## 🔗 Related Concepts / Notes

- [[Python]]
- [[Rust]]
- [[C++]]
- [[CUDA]]
- [[Zig]] (people compare them often)
- [[MLIR]] (Mojo is built on this)
- [[Programming Languages]]

---

## 📚 External Resources

- https:`//www.modular.com/mojo` (main site)
- https:`//docs.modular.com/mojo` (docs)
