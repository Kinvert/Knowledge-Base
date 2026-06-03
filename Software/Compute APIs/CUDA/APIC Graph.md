---
aliases:
  - APIC
  - API Capture Graph
  - Warp APIC
tags:
  - cuda
  - gpu
  - simulation
  - reinforcement-learning
title: APIC Graph
---

# ⚡ APIC Graph

An **APIC Graph** is NVIDIA Warp's serialized API Capture graph format: Python records a sequence of Warp operations, saves the captured computation to a `.wrp` artifact, and later replays it from Python or standalone C++ through Warp's native runtime. In reinforcement learning and robotics simulation, the useful idea is to use Python for defining and capturing the computation, then run the hot rollout loop as a native loadable graph with stable buffers.

---

## 🧾 Summary

- **APIC** means API Capture in [[NVIDIA Warp]].
- It records Warp API operations such as kernel launches, selected copies, and zero-initialization.
- Captured graphs can be serialized with `wp.capture_save()` and loaded with `wp.capture_load()`.
- The serialized artifact is a `.wrp` file plus compiled module artifacts for kernels.
- It is experimental in Warp 1.13 and the file/API format may change.
- The key performance goal is fewer Python calls and fewer repeated graph-capture costs in tight GPU simulation loops.

---

## 🧠 Core Idea

Traditional high-throughput RL simulation often pays overhead in the wrong place:

1. Python launches many small kernels.
2. The simulator and trainer exchange buffers repeatedly.
3. CUDA graph capture may be repeated if pointer addresses change.
4. Rollout collection becomes host-orchestration-bound instead of GPU-bound.

APIC changes the shape of the loop:

1. Use Python and Warp to define the computation.
2. Capture the simulation/update sequence once with API Capture enabled.
3. Serialize the graph to a `.wrp` file.
4. Load the graph in Python or C++.
5. Bind stable rollout buffers and launch the graph repeatedly.

For [[PufferLib]]-style training, this is attractive because rollout generation can become a native graph launch that feeds observation, action, reward, done, and state buffers without re-entering a Python environment step for every sub-operation.

---

## 🔁 APIC Workflow

| Stage | What Happens | Typical Tooling | Why It Matters |
|-------|--------------|-----------------|----------------|
| Define | Write simulation kernels and buffers in Python | [[NVIDIA Warp]], [[MuJoCo Warp]] | Keeps authoring ergonomic |
| Capture | Record kernel launches and supported ops | `wp.capture_begin(apic=True)` / `wp.capture_end()` | Freezes the launch sequence |
| Serialize | Save graph and module references | `wp.capture_save()` | Makes graph reusable outside the capture process |
| Load | Load the `.wrp` graph | `wp.capture_load()` or C `wp_apic_*` API | Moves execution toward native runtime |
| Bind | Attach real buffers or parameters | APIC graph parameter APIs | Connects rollout buffers to the graph |
| Replay | Launch graph repeatedly | `wp.capture_launch()` or native C++ replay | Removes repeated Python launch overhead |

---

## 🚀 Why It Matters for RL

APIC is most interesting when simulation is already GPU-native:

- [[MuJoCo Warp]] or [[MJX]]-Warp steps many worlds in parallel on NVIDIA GPUs.
- A policy consumes observations and writes actions on the same device.
- Rollout buffers stay resident on GPU.
- Environment reset, reward, termination, and observation packing are captured into a graph.
- The trainer sees a filled batch instead of driving every physics step from Python.

This creates a path toward "Python as graph authoring, native CUDA as rollout engine." It is especially relevant to small fast environments like CartPole, where per-step overhead can dominate, and to larger robotics tasks where stable graph replay can reduce launch and orchestration cost.

---

## 🧩 Relationship to CUDA Graphs

| Concept | Layer | Captures | Replay Surface | Notes |
|---------|-------|----------|----------------|-------|
| [[CUDA]] Graph | CUDA runtime/driver | CUDA kernel launches and GPU work | `cudaGraphLaunch()` | Low-level graph execution primitive |
| Warp Graph | Warp runtime | Warp kernel launches and supported Warp ops | `wp.capture_launch()` | Python-facing capture/replay |
| APIC Graph | Warp API Capture serialization | Warp API operation log plus graph metadata | Python or native C/C++ Warp API | Serializable `.wrp` graph |
| JAX/XLA Graph | ML compiler graph | JAX operations and XLA-compatible calls | JIT-compiled executable | Strong ML integration, pointer/layout issues can matter |
| Puffer rollout loop | RL systems layer | Environment stepping and buffer filling | Trainer-specific loop | APIC can move this lower in the stack |

APIC is not a replacement for CUDA graphs. It is a Warp-level serialization and replay mechanism that can sit above native CUDA graph execution and make captured Warp workloads easier to ship, reload, and call from a non-Python host.

---

## 🏎️ Puffer-Style Rollout Pattern

A possible [[PufferLib]] integration looks like:

1. Allocate persistent GPU buffers for observations, actions, rewards, dones, env state, and RNG state.
2. Use Python to build a [[MuJoCo Warp]] or Warp-native environment step.
3. Capture a fixed rollout block, such as `N` physics steps plus observation/reward packing.
4. Serialize it as an APIC graph.
5. In the training process, bind buffers and replay the graph to fill rollout batches.
6. Run PPO or another learner on the generated buffers.
7. Periodically launch evaluation or viewer code outside the captured hot path.

This is a good fit for benchmark tasks because it separates:

- **Authoring**: Python, Warp, MuJoCo model loading, debugging.
- **Execution**: native graph launch, stable buffers, fewer host transitions.
- **Training**: existing policy optimizer and rollout-buffer semantics.

---

## ⚙️ Constraints and Sharp Edges

- APIC is experimental; `.wrp` compatibility is not guaranteed across Warp versions.
- The recorded operation set is limited compared to arbitrary Python execution.
- Dynamic Python control flow does not become native graph control flow by magic.
- CUDA graph replay wants stable buffer addresses; changing buffer pointers may force recapture or break assumptions.
- `.wrp` portability across GPU architectures depends on how kernels were compiled, such as PTX vs architecture-specific output.
- Loading APIC artifacts may require a CUDA-enabled Warp native library even when replaying CPU graphs.
- [[MuJoCo Warp]] currently prioritizes NVIDIA GPU throughput, not automatic differentiation.
- Debugging a serialized graph can be harder than debugging the original Python stepping code.

---

## 📊 Comparison Chart

| Approach | Simulator Location | Hot Loop Driver | Strengths | Weaknesses | Best For |
|----------|--------------------|-----------------|-----------|------------|----------|
| Python Gym loop | CPU/Python | Python `env.step()` | Simple, debuggable | High overhead, poor GPU utilization | Small experiments |
| [[PufferLib]] C env | Native CPU | C/Python trainer bridge | Very fast simple envs | Less physics-rich | Custom lightweight tasks |
| [[MuJoCo]] CPU rollout | Native CPU | C/Python API | Mature physics, accurate contacts | Host/device transfer if learner is GPU | Classic robotics benchmarks |
| [[MJX]] JAX | GPU/TPU via XLA | JAX JIT | ML-native batching and autodiff | JAX constraints, contact/perf sharp edges | Differentiable batched sim |
| [[MuJoCo Warp]] | NVIDIA GPU via Warp | Warp/JAX/Python | High-throughput NVIDIA physics | Beta ecosystem, no Warp autodiff path | Robotics RL throughput |
| APIC graph replay | Native Warp graph | Python or C/C++ replay | Low overhead, serializable capture | Experimental, limited ops | Fixed high-throughput rollout kernels |
| Isaac-style GPU sim | NVIDIA GPU | Framework runtime | Mature large-scale GPU RL stack | Heavier ecosystem | Complex embodied RL |

---

## 🧪 Example Shape

```python
import warp as wp

wp.init()

device = "cuda"

# Allocate persistent buffers first. Pointer stability matters for graph replay.
obs = wp.zeros((num_envs, obs_dim), dtype=wp.float32, device=device)
actions = wp.zeros((num_envs, action_dim), dtype=wp.float32, device=device)
rewards = wp.zeros(num_envs, dtype=wp.float32, device=device)

with wp.ScopedCapture(device=device, apic=True) as capture:
    for _ in range(rollout_steps):
        wp.launch(step_physics, dim=num_envs, inputs=[actions], outputs=[obs], device=device)
        wp.launch(compute_reward, dim=num_envs, inputs=[obs], outputs=[rewards], device=device)

graph = capture.graph
wp.capture_save(graph, "cartpole_rollout.wrp")

# Later, or in another process:
loaded = wp.capture_load("cartpole_rollout.wrp")
wp.capture_launch(loaded)
```

The real implementation needs model/data setup, parameter binding, reset handling, action writes from the policy, and rollout-buffer ownership. The important point is that the fixed simulation/update sequence is captured once and replayed as a graph.

---

## 🧭 Design Questions for Puffer Integration

- Should APIC wrap only environment stepping, or also observation packing and reward calculation?
- Should the policy forward pass be inside the graph, or should APIC only fill rollout buffers?
- How are resets handled without Python branching per environment?
- How are domain randomization and curriculum updates represented as graph parameters?
- Which buffers are owned by the trainer versus the simulator graph?
- Should viewer/eval use the same graph, or a separate slower debug path?
- What is the smallest "puffer2real" example: CartPole, pendulum, reacher, or a simple manipulator?

---

## ✅ Strengths

- Moves repeated launch orchestration out of Python.
- Makes captured simulation pipelines reusable from native code.
- Fits GPU-resident rollout buffers and high-throughput RL.
- Preserves Python as the authoring/debugging language.
- Aligns with [[MuJoCo Warp]] and Warp's broader GPU robotics direction.

---

## ❌ Weaknesses

- Experimental API and file format.
- More brittle than a normal Python environment loop.
- Requires careful buffer lifetime and pointer stability.
- Less flexible for highly dynamic environment logic.
- Debugging graph replay failures can require lower-level CUDA/Warp tooling.

---

## 🔗 Related Notes

- [[CUDA]]
- [[MJX]]
- [[MuJoCo]]
- [[PufferLib]]
- [[Vectorized Environments]]
- [[RL Step]]
- [[RL Environment]]
- [[PPO]]
- [[Sim2Real]]

---

## 🌐 External Resources

- NVIDIA Warp runtime docs: https://nvidia.github.io/warp/user_guide/runtime.html
- NVIDIA Warp 1.13 changelog: https://nvidia.github.io/warp/user_guide/changelog.html
- NVIDIA Warp APIC issue #1349: https://github.com/NVIDIA/warp/issues/1349
- Distillative-AI Warp APIC commit: https://github.com/Distillative-AI/warp/commit/d11116fcaad6927f50dc8c26a9cae6d1806944d7
- MuJoCo MJX / MJX-Warp docs: https://mujoco.readthedocs.io/en/latest/mjx.html
- MuJoCo Warp repository: https://github.com/google-deepmind/mujoco_warp

---

## 🏁 Summary

APIC graphs make Warp computations more portable and replayable: Python captures the workload, APIC serializes it, and a native runtime can replay it against persistent buffers. For RL, the compelling use is a GPU-resident rollout engine where [[MuJoCo Warp]] or Warp-native environments fill training buffers with minimal host orchestration. It is still experimental, but it points directly at a practical bridge between ergonomic Python simulation authoring and fast native execution.
