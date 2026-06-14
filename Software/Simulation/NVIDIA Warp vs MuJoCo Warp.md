---
title: NVIDIA Warp vs MuJoCo Warp
aliases:
  - Warp Comparison
  - NVIDIA Warp and MJWarp
  - MJWarp vs Warp
tags:
  - simulation
  - cuda
  - robotics
  - reinforcement-learning
  - comparison
---

# NVIDIA Warp vs MuJoCo Warp

**NVIDIA Warp** is a general-purpose GPU programming framework for simulation, geometry, and ML kernels.
**MuJoCo Warp (MJWarp)** is a MuJoCo-specific implementation written in NVIDIA Warp, optimized for NVIDIA GPUs and batched rigid-body simulation.

---

## Core difference

The distinction is mostly *scope*:

- NVIDIA Warp: low-level kernel framework with many domain modules (collision, rendering, FEM, sparse/mesh/sampling, etc.).
- MuJoCo Warp: a physics engine implementation that reuses MuJoCo semantics and APIs while executing on top of NVIDIA Warp.

If you need a broad toolkit to build custom GPU simulation operators, use NVIDIA Warp directly.
If you need MuJoCo dynamics + many GPU worlds, start with MuJoCo Warp.

---

## What each gives you

### NVIDIA Warp

- Python kernel API (`@wp.kernel`, `@wp.func`) for custom parallel code.
- CPU and CUDA device backends; runs on CUDA-capable NVIDIA GPUs and supports CPU fallback.
- Spatial primitives like `wp.Bvh`, `wp.Mesh`, `wp.HashGrid`, `wp.Volume` for custom sensor/geometry work.
- Graph capture and `.wrp` serialization with C/C++ replay for fixed compute pipelines.
- Direct interoperability paths for PyTorch/JAX through DLPack.

### MuJoCo Warp

- A batched GPU variant of MuJoCo (`import mujoco_warp as mjw`) optimized for throughput.
- Uses `mjw.Model`/`mjw.Data` workflows with `nworld`, `nconmax`, `naconmax`, and `njmax` as first-class scaling parameters.
- Preserves MuJoCo-style APIs for many physics workflows.
- Uses NVIDIA Warp under the hood, so it can capture repeated stepping workflows with `wp.ScopedCapture`.
- Designed for RL-style batching and large rollout volumes.

---

## Performance model and tradeoffs

- MuJoCo Warp is throughput-oriented. The docs call out that it is optimized for total steps per unit time across many worlds, while MuJoCo CPU stepping is typically the better choice for single-world low-latency control.
- MuJoCo docs explicitly note:
  - good scaling in contact-rich batched scenarios,
  - possible degradation as scene complexity grows (around the 60 DoF range is cited as a common pain point),
  - and that feature parity with CPU MuJoCo is not yet complete.
- NVIDIA Warp gives you performance by custom operator design and graph reuse; you pay more upfront in kernel design and launch topology.
- MuJoCo Warp gives you less custom design burden if MuJoCo dynamics are already your target, but exposes MuJoCo-specific structure rather than a generic kernel API.

---

## Differentiability and training implications

- MuJoCo Warp currently does not provide the same autograd story as `MJX`/JAX paths; this matters if end-to-end differentiable training is central.
- NVIDIA Warp supports differentiable kernel recording via its built-in tape mechanism and integrates with ML workflows for custom differentiable operations.
- In practical RL pipelines:
  - Use NVIDIA Warp when your reward, sensor, or policy-adjacent logic needs highly custom tensor+geometry fusion.
  - Use MuJoCo Warp when simulation accuracy + MuJoCo semantics matter more than full custom differentiability.

---

## API and integration ergonomics

| Dimension | NVIDIA Warp | MuJoCo Warp |
|---|---|---|
| Primary abstraction | Kernels and device arrays | `mjw.Model` + `mjw.Data` + `mjw.step` |
| Main advantage | Custom operators and domain modules | Drop-in MuJoCo dynamics for GPU batched worlds |
| Integration target | Build your own sim stack | Reuse MuJoCo-style task code and MJCF assets |
| Capture path | `wp.capture_*` and APIC replay | `wp.ScopedCapture` around MJWarp calls |
| Output behavior | You define your own state layout | MuJoCo state layout semantics mostly preserved |
| Differentiability | Built-in kernel-level AD | Not on par with MJX-JAX autodiff |
| Best fit | Sensor stacks, geometry/physics hybrids | Batched RL environments with MuJoCo fidelity |

---

## Choosing between them

- Choose **NVIDIA Warp** when:
  - you need custom spatial/sensor kernels (BVH, mesh ray, custom fields),
  - you are not strictly tied to MuJoCo semantics,
  - you want kernel-level control and cross-domain fusion on GPU.

- Choose **MuJoCo Warp** when:
  - you already depend on MuJoCo models/control code,
  - you need large batched rollout throughput,
  - and you can live with the current feature/autodiff boundaries.

- A common pattern is:
  - keep physics in MuJoCo Warp,
  - implement sensor, perception, and custom observation/radiance/occlusion logic in NVIDIA Warp kernels,
  - and replay the joint pipeline via captured graph for RL.

---

## Compatibility caveats

- MuJoCo Warp requires supported MuJoCo model fields; unsupported fields can throw during model transfer to device.
- Contact-rich and high-complexity scenes require careful setting of `nconmax`, `naconmax`, and `njmax`.
- NVIDIA Warp and MuJoCo Warp both benefit from capture for repeated loops, but GPU graph behavior must be profiled on your specific workload.
- CUDA driver/toolchain and package version pinning is usually where integration work appears in production.

---

## Related docs and notes

- NVIDIA Warp docs: https://nvidia.github.io/warp/stable/
- NVIDIA Warp runtime and graph capture: https://nvidia.github.io/warp/stable/user_guide/runtime.html
- NVIDIA Warp API reference: https://nvidia.github.io/warp/stable/api_reference/warp.html
- MuJoCo Warp docs: https://mujoco.readthedocs.io/en/latest/mjwarp/index.html
- MuJoCo Warp package: https://github.com/google-deepmind/mujoco_warp

### Related notes

- [[NVIDIA Warp]]
- [[MuJoCo Warp]]
- [[APIC Graph]]
- [[MuJoCo]]
- [[MJX]]
- [[PufferLib]]
- [[Reinforcement Learning]]
