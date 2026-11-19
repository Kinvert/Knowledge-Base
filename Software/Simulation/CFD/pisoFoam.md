# pisoFoam

`pisoFoam` is the OpenFOAM transient incompressible solver that uses the PISO pressure-velocity coupling loop.
For external single-phase incompressible work, this is usually the first transient choice after steady screening with `simpleFoam`.

OpenFOAM’s solver class page describes it as an incompressible, transient solver with PISO coupling for pressure correction within each time step.

Source: <https://doc.openfoam.com/2306/tools/processing/solvers/rtm/incompressible/pisoFoam/>

---

## Why `pisoFoam` matters for external flow

Use `pisoFoam` when at least one of these is true:

- wake, separation onset, or startup transients are part of your objective,
- `simpleFoam` gives stable geometry ranking but KPI mismatch against measured phase-sensitive behavior,
- inlet ramping, gust-like forcing, or unsteady boundary-layer development matters.

Compared with `simpleFoam`, this is usually the smallest step up in cost because it keeps one equation family but adds time dependence and PISO corrections.

The OpenFOAM pressure-velocity method taxonomy places it as the transient complement to SIMPLE in the incompressible family.

- Quickstart flow logic: <https://doc.openfoam.com/2306/quickstart/>
- Pressure-velocity overview: <https://doc.openfoam.com/2312/tools/processing/solvers/pressure-velocity/>

---

## What it solves (practical level)

Core behavior:

- Incompressible momentum and continuity.
- Transient progression through `Δt` and `controlDict`.
- PISO pressure correction cycles inside each time step.
- Turbulence models through `constant/turbulenceProperties` as in standard OpenFOAM cases.

Think of it as:

- steady pressure correction (`SIMPLE`) removed from pseudo-time loop,
- replaced by time-step-by-time-step pressure enforcement with one or more correctors.

---

## Where to place it in an external single-phase optimization pipeline

A pragmatic sequence for your likely cases:

1. `simpleFoam` for cheap geometry sweep and baseline ranking.
2. `pisoFoam` on the best candidates where unsteadiness changes drag/lift/pressure recovery.
3. `pimpleFoam` only if coupling stiffness appears (small time steps needed, or unstable transients).

Decision signal to escalate from `simpleFoam`:

- unsteady vortical modes affecting force history,
- startup transients drive exit plane fluctuations,
- pseudo-steady solution appears over-smoothed relative to expected physics.

---

## Comparison against adjacent choices

| Solver | Flow class | Primary coupling | Time behavior | Typical speed | Typical use | Main reason to switch |
|---|---|---|---|---|---|---|
| [[simpleFoam]] | incompressible single-phase | SIMPLE | steady | fastest | pure steady design screening | switch if true unsteady effects matter |
| [[pisoFoam]] | incompressible single-phase | PISO | transient | medium | startup and moderate transients | baseline transient accuracy |
| [[pimpleFoam]] | incompressible single-phase | PIMPLE | transient | slower than PISO | stiff coupling / larger timestep stability |
| [[icoFoam]] | incompressible laminar | PISO-style (laminar path) | transient | typically fast | laminar no-turbulence baseline |
| [[interFoam]] | multiphase | PISO/PIMPLE variants | transient | higher | free-surface or interface flow |
| [[rhoPimpleFoam]] | compressible weakly | pressure-based PIMPLE | transient | higher | thermal density coupling becomes dominant |

For each file family, this table is the practical difference for external single-phase CAD loops.

---

## CLI, controls, and settings you actually change first

### Baseline run

```bash
pisoFoam -case ../cases/bluff_A -parallel
pisoFoam -case ../cases/bluff_A -postProcess -func "wallShearStress" -time 0.5
```

### Key dictionaries

`system/controlDict`

- `startTime`, `endTime`, `deltaT` (or event-driven controls)
- time step constraints (`maxCo`, `maxDeltaT` if present)
- write strategy for checkpoints and KPI slices

`system/fvSolution`

- `PISO` subsection (`nCorrectors`, `nNonOrthogonalCorrectors`, residual control blocks)
- pressure and velocity tolerances tied to transient acceptance criteria

`system/fvSchemes`

- `ddtSchemes` for transient transport,
- `gradSchemes` and `divSchemes` choices for wake resolution,
- upwind/blended choices tuned for stability vs diffusion.

`constant/turbulenceProperties`

- model family consistency (RANS/LES/laminar baseline),
- wall treatment suitable for boundary-layer intent.

---

## External-flow-specific practical checks

- Always run a mesh quality gate before the run.
- Use identical geometry param sets across candidate states to preserve comparability.
- Use identical turbulence and scalar settings when comparing candidate designs.
- Start with small, stable `Δt` and increase only after residual and force history remain clean.

A robust per-sample pattern:

1. `checkMesh`
2. short warmup horizon (5–20 steps)
3. confirm no pressure/velocity spike divergence
4. collect key KPI windows after statistical burn-in
5. archive only stable transients


---

## What to do when things fail

| Failure mode | Typical cause | Corrective move |
|---|---|---|
| Oscillatory pressure and fail-fast divergence | too aggressive `nCorrectors`/`nNonOrthogonalCorrectors` | lower effective correction aggressiveness and reduce `Δt` |
| Very slow force-history convergence | overly diffusive `divSchemes` or poor mesh layering | refine near-wall topology before changing time controls |
| Numerical noise interpreted as physics | wake is coarse or under-resolved | increase wake/core resolution and verify with `simpleFoam` + `pimpleFoam` cross-check |
| Runs diverge after first startup | inconsistent initialization and BC magnitude | harmonize inlet scale units and restart from cleaner state |
| High cost with little benefit | geometry is quasi-steady | return to `simpleFoam` for that branch |

Escalation rule:

- if transient behavior remains unstable or too stiff, migrate selected candidates to `pimpleFoam`.

---

## Why this is still useful after you move to `pimpleFoam`

`pisoFoam` remains valuable because it is cheaper and gives a cleaner signal than heavier transient solvers:

- as a first transient filter before `pimpleFoam`,
- as a baseline in sensitivity studies where `pimpleFoam` should not be default, and
- as a reproducibility baseline when you compare turbulence model sensitivity.

---

## Strengths and limits (compact summary)

### Strengths

- lower runtime than `pimpleFoam` on similar transient accuracy targets,
- straightforward to automate,
- clean for startup and forcing transients,
- usually enough for periodic shedding onset checks in mid-fidelity loops.

### Limits

- not as robust for strong coupling with aggressive source terms,
- can require smaller time steps for stiffness,
- not ideal when you need outer-loop pressure coupling strength beyond standard transient PISO behavior.

---

## How `pisoFoam` relates to the rest of our hub

- `simpleFoam` is your quasi-steady rung.
- `pisoFoam` is the first transient rung for single-phase incompressible flow.
- `pimpleFoam` is the stronger coupling rung for harder transient cases.
- `icoFoam` is the laminar/low-model baseline when you explicitly want minimal model stack.

This maps directly to the solver-family map in [CFD Solvers] and the notes backlog around external flow.

---

## Related notes

- [[OpenFOAM]]
- [[simpleFoam]]
- [[pimpleFoam]]
- [[icoFoam]]
- [[interFoam]]
- [[CFD Solvers]]
- [[OpenFOAM Projects]]

---

## External resources (reputable sources)

- OpenFOAM `pisoFoam` documentation: <https://doc.openfoam.com/2306/tools/processing/solvers/rtm/incompressible/pisoFoam/>
- OpenFOAM pressure-velocity methods: <https://doc.openfoam.com/2312/tools/processing/solvers/pressure-velocity/>
- OpenFOAM quickstart and solver selection context: <https://doc.openfoam.com/2306/quickstart/>
- OpenFOAM command-line reference: <https://doc.openfoam.com/2306/fundamentals/command-line/>
- OpenFOAM solver catalog index: <https://www.openfoam.com/documentation/user-guide/a-reference/a.1-standard-solvers>
