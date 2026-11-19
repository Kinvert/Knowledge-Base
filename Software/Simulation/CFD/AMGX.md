# AMGX

`AMGX` (Algebraic Multigrid Solver) is a **linear algebra library** for solving sparse systems on NVIDIA GPUs, not a complete CFD solver.  
In CFD it is mostly used as a high-performance backend for expensive linear solves, especially pressure-type equations.

---

## 🧩 Overview

NVIDIA describes AMGX as a GPU-accelerated core solver library for linear systems, with:

- single-GPU and distributed-MPI support,
- C API,
- configurable nested solver/preconditioner stacks in JSON,
- Krylov and AMG methods (including Classical/Ruge-Steuben and Unsmoothed Aggregation variants),
- mixed precision / `fp32` / `fp64` options.

Source: GitHub repository README and NVIDIA research abstract.

In CFD pipelines this means AMGX acts behind the scenes: OpenFOAM (or another framework) assembles a matrix for a field, AMGX solves it, then the solver proceeds to the next nonlinear/coupling step.

---

## 🧠 Where AMGX sits in an OpenFOAM-style CFD stack

OpenFOAM has this separation:

- **Application solver**: defines which PDEs are solved and how they are coupled (`simpleFoam`, `pisoFoam`, `pimpleFoam`, `rhoPimpleFoam`, `rhoCentralFoam`, etc.).
- **Linear solver selection**: controlled per field in `system/fvSolution` (`solvers`, `relaxationFactors`, etc.), including tolerances and residual control.

OpenFOAM user docs explicitly describe pressure-velocity coupling algorithms (`SIMPLE`, `PISO`, `PIMPLE`) for incompressible/transient workflows and that linear solver settings are managed in `fvSolution`.

AMGX generally enters through the external linear-solver layer (directly or via PETSc) rather than replacing the OpenFOAM solver loop.

---

## 🧪 Pressure is one equation class, not the whole CFD problem

In incompressible CFD, pressure and velocity are coupled through SIMPLE/PISO/PIMPLE loops, where pressure is the correction/pressure field variable enforcing mass continuity and velocity is updated in momentum equations.

For OpenFOAM:
- `simpleFoam` / `pisoFoam` / `pimpleFoam` are the usual pressure-based incompressible families.
- `rhoPimpleFoam` / `rhoSimpleFoam` handle compressible pressure + density coupling (with thermal effects).
- `rhoCentralFoam` and related density-based solvers target high-speed wave/shock-dominant compressible modes.

This is why user guides describe solver **applications** separately from the **linear solver** used on each equation.

---

## 🌀 About your “swirl/density/velocity” question

You are right to separate these:

- **Pressure solvers**: usually the dominant implicit Poisson-like solves in pressure-based CFD loops, and the area with the clearest published AMGX pressure-solve benchmarking in OpenFOAM workflows.
- **Velocity solvers**: solve momentum transport for `U` with convection/diffusion operators and turbulence/source terms. AMGX can help if the entire external-solver path is routed to GPU, but most public AMGX-focused OpenFOAM reports focus on pressure offload.
- **Density/thermophysical solvers**: belong to compressible families (`rho*` solvers) where equation coupling often shifts effort toward EOS/energy/turbulence/source terms.
- **Swirl**: typically a **flow feature / boundary/source configuration**, not a standalone “swirl solver” class in OpenFOAM. OpenFOAM documentation lists `swirlFlowRateInletVelocity` as a velocity BC entry point for swirling inlets.

---

## 🚦 AMGX usage patterns in OpenFOAM ecosystems

A NAFEMS/OpenFOAM HPC report on `amgx4Foam` describes pressure-solve coupling through PETSc4FOAM, where the OpenFOAM matrix is copied to the GPU and a preconditioned conjugate-gradient style pressure solve is performed with AMGX, then data is copied back.  
That implies:

- AMGX can significantly accelerate the pressure stage,
- the full case speedup depends on what fraction of runtime is in that stage,
- data transfer and CPU-side stages can dominate if many non-pressure components are expensive.

PETSc also exposes AMGX through `PCAMGX`, so in principle AMGX can be one preconditioner path inside PETSc-based flows.

---

## 📊 Speed / accuracy expectations

**Speed**

Public benchmark material reports pressure-solve speedups in the order of:

- up to ~9× over a CPU FOAM-GAMG-PCG baseline in some dual-CPU baseline comparisons,
- ~7× in another GPU-only comparison using AMGX-AMG-CG-c (also with a CPU baseline reference),
- with notes about architecture scaling (e.g., A100 expected gains over older V100-class GPUs).

These are useful but very case/hardware-specific; they are not full-application guarantees.

**Accuracy / convergence**

AMGX does not change the governing equations by itself; it changes how a linear system is solved.
In OpenFOAM, accuracy control still comes from per-field linear solve tolerances (`tolerance`, `relTol`) and relaxation/coupling settings in `fvSolution`, plus residual control criteria and residual strategy.

So the practical decision is:
- if convergence behavior is stable at chosen tolerances, AMGX usually improves wall-time,
- if residual/iterations become noisy, tune preconditioner + tolerances first rather than assuming GPU speed alone fixes robustness.

---

## 📈 Comparison Chart (CFD equation groups)

| Area | Typical unknowns / equations | Typical OpenFOAM examples | AMGX relevance | Speed tendency | Accuracy / risk notes |
|---|---|---|---|---|---|
| Pressure solve | `p` continuity-correction equation (pressure Poisson / pressure equation) | `simpleFoam`, `pisoFoam`, `pimpleFoam` | **Primary published AMGX use case** through amgx4Foam / PETSc pathways | High potential; can be the largest linear gain contributor when pressure phase is dominant | Accuracy mostly from `fvSolution` tolerances; residuals still matter |
| Velocity/momentum | `U` transport with advection-diffusion | `simpleFoam`, `pisoFoam`, `pimpleFoam`, `rhoPimpleFoam` | Possible through a full external-solver stack, less commonly highlighted in public AMGX pressure-focused benchmarking | Moderate, but gains are workflow-dependent because source terms/nonlinear coupling often dominate | Harder convergence behavior can appear on strong advection settings |
| Density / thermal compressible coupling | `rho`, `T` + EOS/energy coupling | `rhoPimpleFoam`, `rhoSimpleFoam`, `rhoCentralFoam` | Weakly represented in current AMGX pressure-focused reports | Case-dependent; pressure-based vs density-based solver choice often dominates runtime | Risk shifts to thermodynamic model consistency and shock/transonic handling |
| Scalars / turbulence fields | `k`, `ε`, scalar transport, species transport | `reactingFoam`, `fireFoam`, `interFoam` variants | Not dominant in published AMGX cases; only effective if whole external stack is on accelerated path | Usually lower than pressure-only gains unless coupling is heavily pressure-iteration driven | Solver tuning can become nonlinear because scalar transport stiffness varies by discretization |
| Swirl / rotational effects | `swirlFlowRateInletVelocity` BC + rotational source terms in momentum/thermodynamics | Any incompressible/transient solver with swirl inlet or rotating frame setup | **Not a solver family by itself**; no standalone “swirl solver” slot in standard lists | Not a direct AMGX category | Wrong BC or turbulence model setup often hurts convergence more than linear solve speed |

This can guide where a follow-up note set should happen:
- Pressure solver behavior is its own first-class topic,
- Velocity and density coupling bring different stability bottlenecks,
- Swirl and rotational terms are configuration-level physics, not separate sparse-linear backends.

---

## 🛠 Practical workflow check

- Start with CPU/GPU baseline `simpleFoam`/`pimpleFoam` runs and check:
  - pressure solve fraction in timing,
  - non-orthogonal corrections count,
  - residual curve stability in pressure and momentum.
- If pressure dominates, test AMGX path for pressure linear solves first.
- Keep tolerances and residual-control policy explicit (`tolerance`, `relTol`, residual checks), so speed gains are not traded for uncontrolled residual drift.

---

## 🔗 Related notes

- [[OpenFOAM]]
- [[simpleFoam]]
- [[pisoFoam]]
- [[pimpleFoam]]
- [[rhoPimpleFoam]]
- [[rhoSimpleFoam]]
- [[rhoCentralFoam]]
- [[interFoam]]
- [[reactingFoam]]
- [[CFD]]

---

## 📚 External resources

- https://github.com/NVIDIA/AMGX
- https://research.nvidia.com/publication/2015-10_amgx-library-gpu-accelerated-algebraic-multigrid-and-preconditioned-iterative
- https://www.openfoam.com/documentation/user-guide/6-solving/6.3-solution-and-algorithm-control
- https://www.openfoam.com/documentation/user-guide/a-reference/a.1-standard-solvers
- https://www.openfoam.com/documentation/user-guide/5-models-and-physical-properties
- https://www.openfoam.com/documentation/user-guide/6-solving/6.3-solution-and-algorithm-control
- https://www.openfoam.com/documentation/user-guide/6-solving/6.3.1-linear-solver-control
- https://www.openfoam.com/documentation/user-guide/6.3-solution-and-algorithm-control
- https://www.nafems.org/publications/resource_center/nwc21-262-b/
- https://www.mdpi.com/2226-4310/10/9/792
- https://petsc.org/main/manualpages/PC/PCAMGX/
- https://petsc.org/release/manual/ksp/  
