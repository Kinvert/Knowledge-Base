# FluidX3D

`FluidX3D` is a GPU-first Lattice Boltzmann Method (LBM) CFD code focused on high throughput on OpenCL hardware. The project describes it as a lattice-Boltzmann DNS-oriented solver with optional free-surface, thermal, force, and LES extensions, plus in-situ visualization paths for flow inspection without exporting intermediate fields.

---

## Overview

The project is maintained as a source distribution with a strong emphasis on speed and low memory overhead on commodity hardware.

- Physics: incompressible/weakly compressible LBM
- Method family: D2Q9 (2D), D3Q15, D3Q19 (default), D3Q27 velocity sets
- Collision option: SRT/BGK by default (TRT available in docs)
- Compute backend: OpenCL (vendor-neutral), with CPU fallback paths for small/large-memory use cases
- License model: free for non-commercial/public education/research use; source-available with explicit commercial-use restriction in current releases

---

## What it is best at

- Fast transient incompressible flow on uniform grids
- Real-time/interactive exploration workflows where rendering and solve are coupled
- Large-domain GPU runs where VRAM pressure matters
- High-end laptop-to-server setups where OpenCL portability reduces CUDA lock-in concerns

This aligns with typical external CFD needs where you want quick iteration on mesh-independent, fixed-grid studies and a strong throughput-to-velocity ratio.

---

## Design and implementation traits

From upstream docs and README, likely key architecture choices are:

- OpenCL-only compute model so it runs on AMD/Intel/NVIDIA GPUs (and older integrated or CPU paths in some setups)
- In-place streaming variant (`Esoteric-Pull`) plus low-precision memory options for DDF storage to reduce memory bandwidth and improve cache/throughput behavior
- Domain decomposition for multi-GPU runs (same-node pools with transfer paths across PCIe in current docs)
- Built-in extension toggles for:
  - free-surface flow (including PLIC/VOF style options)
  - body force forcing
  - temperature convection
  - particle tracing and simple object force accumulation
  - turbulence modeling via Smagorinsky-Lilly LES-style damping

These are explicitly described in the FluidX3D documentation and README as implementation-facing features in active development history.

---

## Practical tradeoffs and limits

### Likely strengths

- Excellent for uniform-grid, explicit unsteady incompressible studies
- Very strong throughput and interactive workflow velocity
- Good “single stack” loop when visualization needs are tied to a running simulation
- Very accessible portability for people without vendor-specific GPU stacks

### Likely limits

- Not a compressible-transonic solver (Mach < 0.3 usage guidance)
- No native two-fluid compressible gas/liquid coexistence in base model; free-surface behavior is VoF/PLIC-focused
- No explicit checkpoints according to the FAQ-style docs
- MRT collision strategies are documented as deliberately deprioritized in that branch
- Not a silver-bullet for highly stiff, multiphysics industrial packages where finite-volume/finite-element workflows and mature pre/post tooling dominate

---

## Common setup flow

The official docs present a compile-first flow:

1. Install OpenCL runtime/driver stack for platform
2. Clone or download source
3. Run the local build script / IDE build path
4. Enable extensions in `src/defines.hpp`
5. Select or edit a setup in `src/setup.cpp`
6. Run with device selection for single or multi-GPU device IDs

The build flow expects users to compile locally; there is no pre-bundled installed package experience in the same style as many closed solvers.

Example command style shown in docs:

```bash
chmod +x make.sh && ./make.sh 0
./bin/FluidX3D 0
```

---

## Position vs other CFD families

| Tool | Method | Target hardware style | Typical best use | Notes |
|---|---|---|---|---|
| `FluidX3D` | LBM | OpenCL GPU-first (+CPU) | fast incompressible external/transient studies | strong throughput focus |
| `simpleFoam` | finite-volume RANS | CPU clusters/workstations | broad industrial incompressible workflows | mature boundary-condition ecosystem |
| `pimpleFoam` | PISO+SIMPLE (FVM) | CPU/cluster | coupling-robust external transient | better for pressure-velocity stiff setups |
| `OpenFOAM` | framework | mixed | multiphysics and production pipelines | broader solver ecosystem |
| `OpenLB` | LBM | HPC/CPU-GPU | structured LBM research/prototyping | LBM-specific ecosystem note in `Lattice Boltzmann Method` |
| `Palabos` | LBM | HPC/CPU | parallel structured-grid mesoscopic studies | alternate open LBM framework |
| `Sailfish` | LBM | GPU-centric | Python-driven LBM experiments | lightweight scripting-oriented option |

Use this table as a quick lens for selection:
- If your objective is direct throughput on a uniform box and rapid visual feedback, `FluidX3D` is often a better first pass.
- If you need broad turbulence/compressible/combustion/CFD-post stack interoperability, established OpenFOAM paths remain the safer long-haul pipeline.

---

## Related notes

- [[Lattice Boltzmann Method]]
- [[simpleFoam]]
- [[pisoFoam]]
- [[pimpleFoam]]
- [[OpenFOAM]]
- [[CFD Solvers]]

---

## External references (reputable sources)

- GitHub repository (project docs + source): https://github.com/ProjectPhysX/FluidX3D
- Official documentation hub: https://github.com/ProjectPhysX/FluidX3D/blob/master/DOCUMENTATION.md
- Build and feature docs mirrored in release repo paths:
  - https://github.com/ProjectPhysX/FluidX3D/blob/master/src/setup.cpp
  - https://github.com/ProjectPhysX/FluidX3D/blob/master/src/defines.hpp
- Official website release page: https://fluidx3d.com/
- README and quick-start with FAQs: https://github.com/ProjectPhysX/FluidX3D/blob/master/README.md
