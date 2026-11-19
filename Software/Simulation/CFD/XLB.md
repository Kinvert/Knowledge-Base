# XLB

`XLB` is a Python-first, differentiable Lattice Boltzmann (LBM) CFD library from Autodesk Research designed for geometry-in-the-loop workflows where speed and gradient flow matter.

It is explicitly built to pair fluid simulation with ML workflows: XLB is differentiable, supports multiple hardware backends, and is positioned for physics-based machine learning use cases.

---

## Why XLB is relevant for AI-driven design loops

Your loop—generate geometry by rules → export CAD/mesh → run CFD → evaluate objectives → update design—matches XLB’s strengths when the flow model is formulated on regular lattices.

- XLB is fully differentiable, so geometry/simulation-driven optimization can use gradients directly rather than relying only on black-box search.
- It supports GPU-native execution via NVIDIA Warp and has explicit performance claims versus JAX in the same formulation.
- It also supports multi-resolution and multi-GPU strategies through Neon/JAX for larger runs.

Autodesk and NVIDIA position XLB as an AI-physics bridge where simulation sits inside broader optimization and surrogate loops. NVIDIA’s Warp blog explicitly frames this with examples including surrogate models, RL, and design optimization in end-to-end AI systems. [[NVIDIA Warp blog]](https://developer.nvidia.com/blog/build-accelerated-differentiable-computational-physics-code-for-ai-with-nvidia-warp/)

---

## Core identity and supported backends

From the official docs and README:

- Fully differentiable 2D/3D LBM library.
- Backend support for **JAX**, **NVIDIA Warp**, and **Neon**.
- **Warp** backend targets state-of-the-art single-GPU execution.
- **Neon** and JAX paths add distributed/multi-GPU and multi-resolution workflow options.
- Apache-2.0 licensed, with pip installation support.

`xlb` README install modes:
- `pip install xlb` (CPU default)
- `xlb[warp]` (Warp backend)
- `xlb[cuda]` (JAX+CUDA)
- `xlb[tpu]` (JAX+TPU)

These are all documented in upstream installation examples. [[XLB README]](https://raw.githubusercontent.com/Autodesk/XLB/main/README.md)

---

## Solver model and capabilities

- LBM family includes:
  - BGK
  - KBC (high-Re suitability)
  - Smagorinsky LES option
- Lattice models:
  - D2Q9
  - D3Q19
  - D3Q27 (used in KBC workflows)
- Boundary conditions include equilibrium, bounceback variants, Zou/regularized, outflow, interpolated BCs, and hybrid BC composition.
- Output pathways include VTK/HDF5/XDMF-style numerical outputs and visualization hooks.

XLB also advertises a 3D mesh voxelizer and in-situ GPU rendering options, which is directly relevant when converting geometry into structured simulation fields. [[XLB site]](https://autodesk.github.io/XLB/) [[XLB README]](https://raw.githubusercontent.com/Autodesk/XLB/main/README.md)

---

## Performance claims and what to expect in practice

Publicly reported comparisons from Autodesk/NVIDIA blog material indicate:

- ~8× speedup for Warp vs GPU-accelerated JAX on specific GH200/LBM benchmarks.
- Up to around 95% performance parity with a C++/OpenCL FluidX3D baseline on 512³ lid-driven cavity at reported case settings.
- Warp backend memory and scaling behavior that can outperform JAX on large-domain settings in the same publication context.

Interpretation for loop design:

- Use Warp when execution throughput is your primary bottleneck.
- Use checkpointing/single-source objective extraction to keep design iterations tight.
- Treat published benchmark numbers as case-specific, not universal; they still inform where XLB is most likely to win on wall-time for fixed-resolution iterative loops.

[[XLB site]](https://autodesk.github.io/XLB/) [[NVIDIA Warp speedup on GH200]](https://developer.nvidia.com/blog/autodesk-research-brings-warp-speed-to-computational-fluid-dynamics-on-nvidia-gh200/) [[NVIDIA XLB benchmark case]](https://developer.nvidia.com/blog/build-accelerated-differentiable-computational-physics-code-for-ai-with-nvidia-warp/)

---

## Recommended AI-loop architecture with XLB

### 1) Geometry synthesis and rule engine

Use your geometry generator to emit a CAD-native intermediate (STEP `.step`/`.stp` or Parasolid `.x_t`), then normalize:
- naming/assembly strategy
- feature-level constraints
- meshability checks

Cadence tips:
- cache geometry signatures so identical design states do not re-enter downstream meshing.
- version both generator prompt/rules + geometry output path.

STEP is a long-lived neutral CAD exchange standard and OpenCASCADE documents explicit STEP translator import/export behavior, including schema control for AP214/AP203/AP242. [[OpenCASCADE STEP docs]](https://dev.opencascade.org/doc/overview/html/occt_user_guides__step.html) [[X_T glossary]](https://www.spatial.com/glossary/x-t-file-format)

### 2) Meshing and voxelization path

For LBM/XLB you need structured volume fields, so practical loops usually do:

- CAD/BREP → surface mesh or boundary representation
- tessellation/repair
- volume rasterization into occupancy/solid masks
- boundary-face tagging for inlet/outlet/walls/object surfaces

XLB’s own capability list includes voxelization-oriented support in its output/processing stack, so this is a reasonable downstream representation fit.

### 3) Simulation stage

For each candidate:
- instantiate simulation field from voxelized/boxed flow domain
- run fixed-length solve / horizon solve
- extract metrics (lift/drag surrogates, recirculation metrics, pressure loss, local shear proxies)
- write compact objective tables for optimizer

### 4) Update and policy/optimizer

Because XLB can be differentiated end-to-end, design control loops can:
- compute gradients where geometry-to-objective path is differentiable
- switch to black-box optimization when assumptions fail
- mix both (hybrid strategy) for robustness.

NVIDIA’s Warp examples show practical “simulation + optimization” loops where gradients are taken through solver states using tape/adjoint workflows. [[NVIDIA Warp blog]](https://developer.nvidia.com/blog/build-accelerated-differentiable-computational-physics-code-for-ai-with-nvidia-warp/)

---

## Practical tradeoffs for “very fast loop” goals

### Good fit
- Fast parametric design exploration where geometry changes mostly map to structured masks.
- Moderate-Re to external-flow style setups where LBM turbulence fidelity tradeoffs are acceptable for iteration speed.
- Pipelines where Python-level orchestration + GPU kernels dominate.

### Limits to watch
- LBM assumptions differ from finite-volume high-order compressible/transient solvers for some regimes.
- Geometric preprocessing can dominate wall-time if CAD repair/voxelization is unstable.
- Complex multiphase or thermal coupling is a roadmap/WIP area in current docs.

---

## How this compares to adjacent notes

Your pipeline-level next writes should keep this as the “fast differentiable loop” spoke:
- [[FluidX3D]] (OpenCL GPU LBM)
- [[Lattice Boltzmann Method]]
- [[OpenFOAM]] (non-LBM alternatives and rich physics stacks)

Potential follow-up notes to complete the ecosystem:
- `Differentiable CFD Optimization Loop`
- `Geometry-to-Voxel Pipeline for CFD`
- `STEP and Parasolid for AI CAD-to-CFD`
- `Neon + JAX scaling patterns`

---

## External references

- XLB site/home: https://autodesk.github.io/XLB/
- XLB source repo: https://github.com/Autodesk/XLB
- XLB README with backends and install modes: https://raw.githubusercontent.com/Autodesk/XLB/main/README.md
- XLB changelog notes (notable 0.2.x updates): https://raw.githubusercontent.com/Autodesk/XLB/main/CHANGELOG.md
- XLB paper (Comp. Physics. Commun.): https://doi.org/10.1016/j.cpc.2024.109187
- NVIDIA Warp + XLB performance writeup (GH200): https://developer.nvidia.com/blog/autodesk-research-brings-warp-speed-to-computational-fluid-dynamics-on-nvidia-gh200/
- NVIDIA Warp + XLB + differentiable solver integration: https://developer.nvidia.com/blog/build-accelerated-differentiable-computational-physics-code-for-ai-with-nvidia-warp/
- OpenCASCADE STEP translator docs: https://dev.opencascade.org/doc/overview/html/occt_user_guides__step.html
- Parasolid X_T format overview: https://www.spatial.com/glossary/x-t-file-format
- XLB roadmap/WIP/limitations context: https://autodesk.github.io/XLB/
