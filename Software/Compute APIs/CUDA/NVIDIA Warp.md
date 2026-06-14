---
title: NVIDIA Warp
aliases:
  - Warp
  - warp-lang
  - Warp Python
tags:
  - cuda
  - simulation
  - robotics
  - machine-learning
  - spatial-computing
---

# NVIDIA Warp

**NVIDIA Warp** is a Python framework for writing GPU-accelerated simulation, geometry, and machine-learning kernels. It compiles regular Python functions marked with `@wp.kernel` into kernel code that can execute on CPU or CUDA devices, and it provides built-in spatial data structures (BVH, hash grids, meshes, sparse volumes) plus ML interoperability.

---

## What it is (in practical terms)

Warp is not a single simulator. It is an execution+kernel framework:

- You write computation in Python with Warp kernel/function decorators.
- Warp JIT-compiles that code for the selected device.
- You run custom kernels directly against device arrays and domain-specific primitives.
- You connect with other frameworks (PyTorch/JAX/Paddle) through shared memory protocols (e.g., DLPack).

From the official docs: Warp is a Python framework for GPU-accelerated simulation, robotics, and machine learning, with differentiable kernels usable in ML pipelines [`PyTorch, JAX, Paddle`](https://nvidia.github.io/warp/stable/).

---

## Why this matters for your vision/audio stack

You asked specifically about long-horizon training with vision/audio and BVH trees. That use case usually needs:

- fast ray/visibility queries for virtual cameras,
- scene occlusion checks,
- nearest-neighbor / proximity searches,
- and deterministic, batched GPU throughput with low host overhead.

Warp has direct primitives for this kind of spatial acceleration (BVH, mesh rays, hash grids, volumes). That means you can model the heavy spatial work in Warp kernels while keeping sensor networks and policy nets in PyTorch/JAX.

The `warp/examples` gallery includes open-source rendering and fluid-dynamics content (for example `raycast`, `raymarch`, `marching_cubes`, `nvdb`, `fluid`, `sph`, and `wave`), which is a useful signal that the ecosystem is actively used for sensory and geometry-heavy simulation.

---

## Core architecture

### Runtime model

1. **Python-side program construction**
   - create arrays (CPU/CUDA), meshes, BVH, hash grids, volumes.
2. **Kernel definition**
   - define kernels with decorators (`@wp.kernel`) and typed arguments.
3. **Compilation**
   - kernel compilation is done by Warp’s runtime and cached.
4. **Launch**
   - kernels are launched explicitly (`wp.launch(...)`) or captured in graphs for replay.

Typical flow:

```python
import warp as wp
import numpy as np

wp.init()

@wp.kernel
def scale_positions(pos: wp.array(dtype=wp.vec3), out: wp.array(dtype=wp.vec3), s: float):
    tid = wp.tid()
    out[tid] = pos[tid] * s
```

---

## Installation and runtime baseline

From current installation docs:

- Python **3.10+** is required.
- NVIDIA CUDA-capable GPU support exists (minimum GeForce GTX 9xx era per docs; CUDA/toolchain versions define exact driver requirements for modern packages).
- Standard install is `pip install warp-lang`.
- Conda and source builds are also supported.

Common practical notes:

- CPU execution is possible for debugging on non-CUDA paths.
- Different CUDA runtime versions have different driver minimums; mismatches usually warn at init time.
- Source builds require standard dev toolchain + CUDA Toolkit.

---

## Built-in spatial acceleration you actually want for sensory sims

Warp includes spatial primitives under its **spatial computing** area:

- `wp.Bvh` for bounding volume hierarchies and ray-based acceleration workflows,
- `wp.Mesh` for triangle geometry,
- `wp.HashGrid` for neighborhood queries,
- `wp.Volume` for sparse volumetric data (NanoVDB-backed).

The runtime docs explicitly call out `wp.Bvh` and `wp.Mesh` use for accelerated ray and collision-like queries, and `Mesh.refit()` when vertex positions move.

For BVH-heavy sensory workloads, this is the important part:

- Build BVH once from bounds/geometries, then refit/rebuild when geometry changes.
- Query it per step from kernels (ray/overlap queries) to answer:
  - what geometry a virtual ray hits first,
  - which volumes overlap,
  - which neighbors are in proximity when your sensor model or contact logic needs it.

---

## BVH explained (for your context)

**BVH (Bounding Volume Hierarchy)** is a hierarchical spatial index:

- Each node stores a coarse bounding volume (often an AABB).
- Internal nodes group child bounds; leaves store primitive proxies (triangles, points, bounds).
- Query traversal descends only branches whose bounds intersect the query.

Why this helps:

- Instead of testing an incoming ray against all triangles/cells, BVH cuts out large parts of the scene with one bound test.
- Complexity drops from near \(O(N)\) per query to near \(O(\log N)\) for many scene types.
- For many rays (vision and lidar-like sensors) and occlusion queries, this is exactly the data-structure win you need.

For acoustics, this pattern is also useful for batched obstacle-aware checks:

- approximate source/receiver visibility with ray-like probes,
- prune impossible reflection/refraction candidates before heavier propagation math,
- pull nearby candidate surfaces faster than scanning all triangles.

Use-case mapping:

- **Vision rendering proxy** (depth/raycast): BVH reduces per-pixel ray traversal work.
- **Audio occlusion/ray tracing approximate**: fast obstacle checks for path/integral estimates.
- **Lidar-style beams**: broad-phase pruning of long-hit lists before exact geometry checks.
- **Contact-aware policies**: quick neighborhood rejection before detailed response computations.

---

## Interop with ML frameworks

Warp is designed to share data with ML tensors via common protocols:

- DLPack for PyTorch/JAX interop paths,
- direct array workflows with NumPy-like and framework tensors where possible.

This is important if you want Warp handling environment stepping while your policy stays in PyTorch or JAX.

---

## Performance posture

Warp’s performance profile is usually best when you:

- keep data on-device and avoid host-device ping-pong,
- prefer persistent buffers and batched execution,
- use graph capture when stepping loops repeat.

Warp exposes capture APIs and launchable graph paths to reduce Python/launch overhead in repetitive rollout loops.

The runtime docs describe `wp.ScopedCapture`, `capture_save`/`capture_load`, and C-replay entry points (APIC) for fixed pipelines.

For sensory pipelines, this often matters more than raw kernel speed:

- fewer launch boundaries,
- stable graph + buffer strategy,
- predictable occupancy when many threads run similar work.

---

## Limits and sharp edges

- **NVIDIA-only compute** (CUDA orientation).
- Compile latency can be significant for new kernels in development.
- Dynamic scene structures need explicit maintenance (`refit`/rebuild patterns for BVH/mesh updates).
- Debugging kernel behavior is lower-level than PyTorch-only workflows; useful for throughput, but you must own correctness/testing.
- Memory layout and object lifetimes for acceleration structures are meaningful (`Mesh`, `BVH`, query objects should remain valid while used in kernels).

---

## Comparison

| System | Role | Target Domain | GPU Model | Diff. Capability | Best For |
|---|---|---|---|---|---|
| NVIDIA Warp | Python GPU kernel framework | Custom simulation / geometry / physics | CUDA kernels (CPU fallback) | Explicit differentiability support + low-level kernels + BVH/mesh/volume primitives | Custom spatial kernels for robotic perception/sensors |
| Taichi | Python/DSL for high-performance compute | Graphics/physics/prototyping | CUDA/CPU/backends | Statically scheduled kernels, strong compiler path | Physically based compute and domain-specific compilers |
| Numba CUDA | LLVM/Python JIT to CUDA | General compute kernels | CUDA only | Manual kernel coding, no built-in spatial accel stack | Custom scientific kernels close to CUDA style |
| CUDA C++ | Lowest-level control | Full-stack GPU systems | CUDA | Maximum control/performance | When you need custom driver/runtime architecture |
| PyTorch | Tensor-centric DL + custom CUDA ops via extensions | ML and tensor workloads | CUDA/CPU + others | Autograd + ecosystem-first | RL policy/inference and model-centric pipelines |
| JAX | Functional array programming + transforms | Array-centric ML + custom kernels | TPU/GPU/CPU | Transformations (JIT, grad, vmap) | Differentiable program transformations and math-first workflows |

---

## Related notes

- [[APIC Graph]]
- [[MuJoCo Warp]]
- [[NVIDIA Warp vs MuJoCo Warp]]
- [[CUDA]]
- [[PyCuda]]
- [[cuBLAS]]
- [[PTX]]
- [[CUDA Streams]]
- [[Spatial Indexing Algorithms]]

---

## Hello-world / starter tutorials

- **First project path**
  - https://nvidia.github.io/warp/stable/user_guide/installation.html
  - https://nvidia.github.io/warp/stable/user_guide/basics.html
  - https://nvidia.github.io/warp/stable/user_guide/runtime.html
- **Beginner tutorial notebook**
  - https://github.com/NVIDIA/accelerated-computing-hub/blob/32fe3d5a448446fd52c14a6726e1b867cbfed2d9/Accelerated_Python_User_Guide/notebooks/Chapter_12_Intro_to_NVIDIA_Warp.ipynb
  - https://colab.research.google.com/github/NVIDIA/accelerated-computing-hub/blob/32fe3d5a448446fd52c14a6726e1b867cbfed2d9/Accelerated_Python_User_Guide/notebooks/Chapter_12_Intro_to_NVIDIA_Warp.ipynb
- **Executable example starters**
  - https://github.com/NVIDIA/warp/tree/main/warp/examples/core/example_graph_capture.py
  - https://github.com/NVIDIA/warp/tree/main/warp/examples/core/example_raycast.py
  - https://github.com/NVIDIA/warp/tree/main/warp/examples/core/example_fluid.py
- **Renderer / visual pipeline mini projects**
  - https://github.com/NVIDIA/warp/tree/main/warp/examples/core/example_marching_cubes.py
  - https://github.com/NVIDIA/warp/tree/main/warp/examples/core/example_render_opengl.py
- **Reference repo**
  - https://github.com/NVIDIA/warp

## Related docs and references

- NVIDIA Warp documentation (stable): https://nvidia.github.io/warp/stable/
- Installation: https://nvidia.github.io/warp/stable/user_guide/installation.html
- Runtime + spatial primitives + BVH examples: https://nvidia.github.io/warp/stable/user_guide/runtime.html
- API reference: https://nvidia.github.io/warp/stable/api_reference/warp.html
- Interoperability docs: https://nvidia.github.io/warp/stable/user_guide/interoperability.html
- Repository: https://github.com/NVIDIA/warp

---

## Quick takeaway

If your project is doing **many custom spatial queries + simulation steps + sensory pipelines**, Warp gives you a better fit than tensor-first engines alone because it was built to author GPU kernels for simulation, geometry, and ML-assisted workflows while exposing the right hooks for BVH and interop.
