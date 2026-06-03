---
aliases:
  - MJWarp
  - MuJoCo-Warp
  - MJX-Warp
tags:
  - simulation
  - robotics
  - reinforcement-learning
  - cuda
title: MuJoCo Warp
---

# ⚡ MuJoCo Warp

**MuJoCo Warp** (**MJWarp**) is a GPU-oriented implementation of [[MuJoCo]] written in NVIDIA Warp and optimized for NVIDIA hardware. It targets high-throughput batched physics simulation for [[Reinforcement Learning]], especially cases where thousands of worlds step on the GPU while policies and rollout buffers also live on the GPU.

---

## 🧾 Summary

- MJWarp is developed by Google DeepMind and NVIDIA.
- It implements MuJoCo-style rigid-body simulation in Warp for NVIDIA GPUs.
- It can be used directly as `mujoco_warp` or through [[MJX]] as the Warp implementation.
- It is optimized for throughput, not low-latency single-world stepping.
- It is useful when CPU MuJoCo becomes bottlenecked by host/device transfer and Python launch overhead.
- It currently does not provide the automatic differentiation path that MJX-JAX provides.
- It pairs naturally with [[APIC Graph]] capture for fixed, replayable rollout kernels.

---

## 🧠 Core Idea

Classic [[MuJoCo]] is excellent physics, but many RL pipelines use GPU policies. If simulation runs on CPU, every step can involve CPU stepping, observation packing, host-to-device transfer, policy inference, device-to-host action transfer, and Python orchestration. That overhead can dominate small tasks and becomes painful at high environment counts.

MJWarp changes the data placement:

1. Compile/load a MuJoCo model on the host.
2. Copy the model and batched data to GPU as Warp arrays.
3. Step many worlds in parallel with `mujoco_warp.step`.
4. Keep observations, rewards, resets, and actions on GPU.
5. Optionally capture the fixed stepping block into a Warp graph or [[APIC Graph]].

The practical pitch is "MuJoCo physics, CUDA-resident rollout buffers."

---

## 🧩 Relationship to MJX

[[MJX]] now describes two MuJoCo accelerator implementations:

| Implementation | Backend | Hardware | Differentiable | Best For | Notes |
|----------------|---------|----------|----------------|----------|-------|
| MJX-JAX | JAX/XLA | CPU, GPU, TPU | Yes | Differentiable simulation, JAX-native training | Can bottleneck on contacts/constraints |
| MJX-Warp / MJWarp | NVIDIA Warp | NVIDIA GPUs | No | High-throughput contact-rich RL | Better NVIDIA throughput path |
| CPU MuJoCo | C/C++ | CPU | Analytical derivatives in MuJoCo APIs | Accurate single-world simulation, control, debugging | Mature and low-latency |

MJX can call the Warp implementation with `impl='warp'`, but direct `mujoco_warp` usage gives lower-level control over buffers, graph capture, and Puffer-style rollout integration.

---

## 🏎️ Why It Matters for PufferLib

[[PufferLib]] is strong at fast environment loops and clean training interfaces. Historically, the cleanest Puffer path is often a native CPU environment that fills rollout buffers as quickly as possible. MJWarp suggests a GPU-native path:

- Keep `num_envs` worlds in one MJWarp data object.
- Store observations/actions/rewards/dones as contiguous GPU arrays.
- Let the policy forward pass produce actions directly into the MJWarp action buffer.
- Capture `policy -> step -> obs/reward/done packing` as a repeated graph segment.
- Hand PPO a rollout buffer that was generated with minimal Python involvement.

For small tasks like CartPole, this attacks Python dispatch overhead. For robotics tasks, it also keeps physics and learning on the same NVIDIA device.

---

## 🔁 PufferLib Integration Shape

| Layer | Responsibility | Possible Implementation |
|-------|----------------|-------------------------|
| Model setup | Load MJCF and tune batch/contact limits | Python `mujoco` + `mujoco_warp` |
| Device state | Own batched `mjw.Model`, `mjw.Data`, RNG, reset masks | Warp arrays |
| Action bridge | Expose policy output as `ctrl` or actuator buffer | Torch/Warp interop or explicit copy |
| Step kernel | Run one or more MJWarp physics steps | `mujoco_warp.step(m, d)` |
| Obs packing | Convert MuJoCo state to policy observations | Custom Warp kernels |
| Reward/done | Compute task reward, termination, truncation | Custom Warp kernels |
| Reset | Reset done worlds and randomize initial states | Custom Warp kernels plus static branch masks |
| Capture | Freeze fixed rollout block | Warp graph or [[APIC Graph]] |
| Trainer bridge | Present rollout tensors to Puffer PPO | Puffer buffer adapter |
| Viewer/eval | Debug policy and physics separately | CPU MuJoCo or MJWarp viewer path |

The first integration should probably not try to make MJWarp look like a normal Python `env.step()` internally. It should expose a Puffer-compatible rollout producer.

---

## 🧪 Minimum Viable Puffer Experiment

A practical first target:

1. Choose a simple MuJoCo task: CartPole, pendulum, or a low-DoF reacher.
2. Load the MJCF with normal MuJoCo and create `nworld` MJWarp data on GPU.
3. Write Warp kernels for:
   - observation packing
   - reward calculation
   - done/reset masks
   - randomized reset state
4. Connect a small Torch policy to the GPU action buffer.
5. Capture a fixed rollout block:
   - policy inference
   - action write
   - `mujoco_warp.step`
   - obs/reward/done write
   - masked reset
6. Fill Puffer-style rollout buffers.
7. Train PPO and verify that evaluation reaches the solved threshold.

The engineering milestone is not only steps per second. It is "policy learns, eval solves, buffers match Puffer semantics, and the fast path is captured."

---

## 🧵 APIC Graph Path

[[APIC Graph]] makes MJWarp more interesting because MJWarp steps are not one kernel. They are a bundle of Warp operations and kernel launches. The MuJoCo Warp docs recommend graph capture when repeatedly calling `mujoco_warp.step`, because direct calls launch constituent kernels individually.

The proposed APIC path:

1. Build the MJWarp model/data and Puffer rollout buffers in Python.
2. Enter `wp.ScopedCapture(device="cuda:0", apic=True)`.
3. Record the fixed rollout block.
4. Save it as `.wrp` plus compiled module artifacts.
5. Load it in the trainer process, or eventually in native C++.
6. Use named input/output bindings for actions, observations, rewards, and reset masks.
7. Replay the graph repeatedly.

The NVIDIA Warp APIC issue describes this as a unified graph capture layer for serializing captured graphs and replaying them from Python or standalone C++ without the original Python program.

---

## ⚙️ Performance Tuning

| Lever | Why It Matters | Puffer Relevance |
|-------|----------------|------------------|
| `nworld` | Number of parallel worlds | Main SPS scaling knob |
| `nconmax` / `naconmax` | Contact allocation limit | Too high wastes memory; too low overflows |
| `njmax` | Constraint allocation per world | Required for contact-rich scenes |
| Graph capture | Avoid repeated kernel launch overhead | Critical for tiny envs and rollout blocks |
| Solver iterations | Accuracy/speed tradeoff | Tune after contact limits |
| Contact sensors | Safer contact access path | Useful for rewards/observations |
| Reset design | Avoid dynamic Python branches | Needed for captured rollouts |
| Buffer layout | Coalescing and trainer compatibility | Determines PPO throughput |

MJWarp docs emphasize tuning contact and constraint limits per environment. A Puffer integration should make these visible in config rather than hiding them behind a generic wrapper.

---

## ⚠️ Sharp Edges

- MJWarp is throughput-optimized; CPU MuJoCo may still be better for single-world low-latency control.
- Contacts and constraints require explicit memory budgeting.
- Very complex scenes can lose performance; MJWarp docs call out degradation beyond roughly 60 DoF.
- MJWarp does not currently offer the same autodiff path as MJX-JAX.
- Dynamic resets and curriculum changes are awkward inside a captured graph.
- Policy inference inside APIC may be harder than capturing only physics and reward kernels.
- Warp APIC capture does not yet cover all dynamic graph constructs, multi-GPU use, or every array operation.
- Viewer/debug paths should remain separate from the captured training hot loop.

---

## 📊 Comparison Chart

| Stack | Physics Location | Policy Location | Graph/Capture Story | Strengths | Weaknesses |
|-------|------------------|-----------------|---------------------|-----------|------------|
| [[PufferLib]] C env | CPU/native | GPU or CPU | Custom loop, not physics graph | Extremely fast simple envs | No MuJoCo contacts |
| [[MuJoCo]] CPU + Puffer | CPU | GPU | Python/C bridge | Mature physics, easy debug | Transfer and launch overhead |
| [[MJX]] JAX | XLA device | JAX device | JIT/XLA | Differentiable, ML-native | JAX stack constraints |
| MuJoCo Warp direct | NVIDIA GPU | Torch/JAX/Warp possible | Warp graph capture | High throughput with MuJoCo semantics | Integration work |
| MuJoCo Warp + [[APIC Graph]] | NVIDIA GPU native graph | Trainer-provided buffers | Serializable `.wrp` replay | Lowest Python overhead path | Experimental APIC surface |
| [[Isaac Gym]] / Isaac Lab | NVIDIA GPU | GPU | Framework-managed | Mature GPU RL ecosystem | Heavier simulator stack |
| [[Genesis]] | GPU via Taichi | GPU/CPU bridge | Framework JIT | Very high claimed throughput | Younger physics stack |

---

## 🧱 Suggested Puffer Architecture

```text
Puffer PPO trainer
        |
        | owns rollout tensors / policy weights
        v
Puffer MJWarp adapter
        |
        | binds obs/actions/rewards/dones/state buffers
        v
APIC or Warp captured rollout graph
        |
        | calls policy/action bridge, mjw.step, reward, reset kernels
        v
MJWarp batched worlds on CUDA
```

The adapter should be explicit about what is captured and what stays outside the graph. A good first version captures physics, observation packing, rewards, and resets, while leaving policy inference in the existing Puffer/PyTorch trainer. A more aggressive version captures a fused policy-plus-physics rollout block after the buffer contract is stable.

---

## ✅ Strengths

- Brings MuJoCo-style physics into a CUDA-resident training loop.
- Scales naturally across many worlds.
- Reduces CPU/GPU transfer overhead.
- Fits on-policy algorithms like [[PPO]] that consume large fresh rollout batches.
- APIC can turn the Python-authored rollout into a native replayable graph.
- Provides a bridge from Puffer's fast-env philosophy toward robotics simulation.

---

## ❌ Weaknesses

- Requires a real integration layer, not just a Gym wrapper.
- The fastest path is less flexible and less debuggable than Python `env.step()`.
- Needs environment-specific observation, reward, reset, and contact handling.
- Contact/constraint memory tuning becomes part of the environment config.
- Serialized APIC graphs are experimental artifacts.
- Solving a toy task does not prove robotics-scale robustness.

---

## 🔗 Related Notes

- [[APIC Graph]]
- [[MuJoCo]]
- [[MJX]]
- [[PufferLib]]
- [[Vectorized Environments]]
- [[PPO]]
- [[RL Environment]]
- [[RL Step]]
- [[CUDA]]
- [[Isaac Gym]]
- [[Genesis]]
- [[Sim2Real]]

---

## 🌐 External Resources

- MuJoCo Warp docs: https://mujoco.readthedocs.io/en/latest/mjwarp/index.html
- MuJoCo MJX docs: https://mujoco.readthedocs.io/en/latest/mjx.html
- MuJoCo Warp repository: https://github.com/google-deepmind/mujoco_warp
- NVIDIA Warp APIC issue #1349: https://github.com/NVIDIA/warp/issues/1349
- Distillative-AI APIC commit: https://github.com/Distillative-AI/warp/commit/d11116fcaad6927f50dc8c26a9cae6d1806944d7

---

## 🏁 Summary

MuJoCo Warp is the NVIDIA-GPU path for batched MuJoCo simulation. For [[PufferLib]], the interesting integration is not a conventional Python wrapper; it is a rollout producer that keeps state, actions, rewards, dones, and observations on GPU and optionally captures the fixed rollout block with [[APIC Graph]]. That could make "Puffer plus real robot physics" feel closer to first-party fast environments while preserving the MuJoCo modeling ecosystem.
