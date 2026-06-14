# SU2

**SU2** is an open-source CFD framework focused on computational fluid dynamics for analysis and optimization, with strong roots in aerospace and shape/design workflows.

It is widely used for Euler/Navier–Stokes simulations, aerodynamic shape optimization, and adjoint-based sensitivity workflows where fluid simulations are repeatedly evaluated inside optimization loops.

---

## Overview

SU2 is structured as a finite-volume CFD stack with capabilities spanning:

- compressible and incompressible flow solvers,
- turbulence models,
- multiphysics coupling in selected modules,
- unsteady simulations,
- shape and topology-sensitive design workflows,
- optimization and gradient-based design loops.

It is commonly used in:

- external aerodynamics (airfoils/wings/body work),
- propulsion/ducting studies,
- thermal and flow-structure interaction contexts (depending on stack),
- control and parametric studies in research pipelines.

---

## Core model and methods

SU2 is typically organized around:

- **Spatial discretization:** finite-volume formulation on structured or unstructured meshes.
- **Conservation form:** mass/momentum/energy balance solved through numerical fluxes and appropriate solvers.
- **Time integration:** explicit/implicit variants for steady and unsteady targets depending on problem stiffness/accuracy needs.
- **Turbulence options:** model choices vary by setup and are selected to match target Reynolds regime and flow physics.
- **Adjoint support:** built-in support for gradient-based optimization workflows in many configurations.

For design loops, this is important because it turns SU2 from a “single solve” solver into a differentiable optimization workhorse.

---

## Why SU2 is different from OpenFOAM in practice

- **OpenFOAM** is a broad multiphysics toolbox with many solver families and lots of ecosystem extensions.
- **SU2** is strongly optimized for aerodynamic workflows, especially automated optimization/adjoint loops.
- In short: if you need heavy general-purpose engineering plus custom solver development, OpenFOAM is often your first stack;  
  if you need tight aero optimization and gradient-based design iteration, SU2 is often a better fit.

---

## Typical SU2 workflows

1. **Geometry + mesh prep** in external tools.
2. **Configuration setup** (`.cfg` style options): solver, BCs, model options, convergence settings.
3. **Run primal solve** to generate reference solution fields and objective data.
4. **Objective/integral extraction** (drag/lift/pressure objective, constraints, etc.).
5. **Optional adjoint solve** to compute design gradients.
6. **Optimizer update** based on gradients.
7. **Iterate** with batch automation scripts until objective/constraint targets converge.

In production style workflows, step 7 is typically driven by Python or shell orchestration with scripted case management.

---

## Major use cases

- Aircraft and UAV external aerodynamics.
- Airfoil/wing and nacelle section refinement.
- Drag/lift constrained optimization and geometric parameter sweeps.
- High-throughput design-space exploration with automated evaluators.
- Sensitivity verification for reduced-order or data-driven control loops.

---

## Installation and execution notes

- SU2 supports Linux-centric scientific stack usage most commonly, with source and package-based install routes.
- You typically define solver cases in config files and invoke command-line binaries.
- Solver output is usually written as field files and iteration logs, then post-processed for objective extraction.
- Typical practical requirements:
  - clean mesh workflow,
  - stable preconditioning and CFL strategy,
  - careful boundary-condition consistency,
  - solver-specific restart/checkpoint discipline for long runs.

---

## Strengths

- Strong for aerodynamic design and optimization.
- Adjoint-enabled pipelines reduce expensive finite-difference loops.
- Good for parametric automation and reproducible case studies.
- Finite-volume core is a good match for compressible external flow problems.

---

## Weaknesses

- Not as general-purpose as broad multiphysics toolboxes for every domain.
- Setup and configuration can be verbose without templates.
- Multiphysics breadth is narrower than heavyweight commercial suites in some regimes.
- Steep workflow tuning for convergence if problem conditioning is poor.

---

## SU2 vs other CFD options

| Tool | Focus | Primary strength | Typical cost | Best fit |
|---|---|---|---|---|
| SU2 | CFD + optimization | Adjoint-based design and aerodynamic workflows | Medium–high (depends on mesh/time horizon) | Parametric optimization and aero design |
| OpenFOAM | General CFD toolbox | Solver breadth and user extensibility | Medium–high | Complex flows and custom solver extensions |
| XFOIL | 2D airfoil analysis | Extremely fast early aero screening | Very low | Preliminary airfoil evaluation |
| ANSYS Fluent | Commercial industrial CFD | GUI + mature multiphysics support | High | Large industrial workflows and enterprise integration |
| COMSOL | Multiphysics + FE | Strongly coupled workflows | High | Fully coupled multiphysics studies |
| Lattice Boltzmann codes | Mesoscopic fluid methods | Easy mesoscopic boundary behavior in some domains | Medium | Specialized meso/particle-style modeling |

---

## Comparison with XFOIL

| Criterion | SU2 | XFOIL |
|---|---|---|
| Dimension | 2D/3D | 2D |
| Physics | Euler/Navier–Stokes + turbulence paths | Viscous panel + boundary layer |
| Best first use | Full geometry + optimization | Fast airfoil screening |
| Runtime | Higher | Very low |
| Optimization | Adjoint and gradients built around flow solves | Limited in this direction |

---

## Related notes

- [[CFD]]
- [[OpenFOAM]]
- [[XFOIL]]
- [[Lattice Boltzmann Method]]
- [[Finite Volume Method]]
- [[Turbulence Modeling]]
- [[ParaView]]

## Hello-world / starter projects

- **Getting started path**
  - https://su2code.github.io/docs_v7/Build-SU2-Linux-MacOS/
  - https://su2code.github.io/docs_v7/Quick-Start/
  - https://su2code.github.io/docs_v7/Execution/
  - https://su2code.github.io/docs_v7/Test-Cases/
- **Tutorial progression (all self-contained examples)**
  - https://su2code.github.io/tutorials/
  - https://su2code.github.io/tutorials/Turbulent_NACA0012/
  - https://su2code.github.io/tutorials/Unsteady_NACA0012/
  - https://su2code.github.io/tutorials/Inviscid_2D_Unconstrained_NACA0012/
  - https://su2code.github.io/tutorials/Unsteady_Shape_Opt_NACA0012/
- **Source and case data**
  - https://github.com/su2code/SU2
  - https://github.com/su2code/TestCases

## External resources

- https://su2code.github.io/docs_v7/
- https://su2code.github.io/docs_v7/How-to-Contribute/
- https://su2code.github.io/tutorials/
