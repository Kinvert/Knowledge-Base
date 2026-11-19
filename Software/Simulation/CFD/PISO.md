# PISO

`PISO` is the **Pressure-Implicit with Splitting of Operators** coupling sequence used by OpenFOAM transient incompressible solvers.

Compared to SIMPLE, PISO performs pressure correction inside each time step and is better aligned with unsteady external-flow signals such as shedding growth, startup transients, and gust-like forcing.

--- 

## Core mechanics

1. Predict velocity from momentum equations.
2. Build and solve pressure correction to enforce continuity.
3. Correct fluxes and velocity.
4. Repeat pressure corrections inside the same physical time step (`nCorrectors`).

`pisoFoam` is the standard OpenFOAM implementation.

--- 

## Where it sits in the optimization ladder

- Use PISO after `simpleFoam` when steady assumptions break ranking consistency.
- Use it before `pimpleFoam` unless strong stiffness/coupling warrants outer-loop iterations.
- Keep run settings (`maxCo`, discretization, inlet consistency) conservative in first-pass screening.

For external single-phase CAD loops, this is usually the first transient rung.

--- 

## Practical setup points

### `system/fvSolution` (high-impact)
- `PISO` block controls:
  - `nCorrectors`,
  - `nNonOrthogonalCorrectors`,
  - residual control targets.

### `system/controlDict`
- Keep transient controls explicit: `deltaT`, `endTime`, `writeInterval`.
- Use short warmup windows initially, then longer averaging windows for KPI stability.

### Runtime triage signals
- residual oscillations with realistic phase content → candidate is physically transient,
- immediate divergence at first steps → time-step/mesh/coefficient mismatch,
- excessive numerical damping → increase temporal resolution or refine wake.

--- 

## Comparison chart (PISO vs close alternatives)

| Approach | Flow-time treatment | Coupling depth | Stability behavior | Cost | Best use |
|---|---|---|---|---:|---|
| `pisoFoam` (`PISO`) | transient | inner pressure-correction loops | good at moderate Co | medium | startup wake and short-time transients |
| `simpleFoam` (`SIMPLE`) | steady | outer global loops | very robust for steady | high throughput | initial steady candidate screening |
| `pimpleFoam` (`PIMPLE`) | transient | outer + inner loops | higher robustness on stiff coupling | medium | larger time steps, tighter coupling demand |
| `icoFoam` (transient laminar) | transient | PISO-style transient core | clean for low-Re educational baselines | low-medium | laminar sanity baseline |
| `interFoam` (transient multiphase) | transient | PISO/PIMPLE variants | interface + PISO constraints | medium-high | free-surface or two-fluid transient studies |
| `pisoFoam` + coarse mesh + short time window | transient micro-batch | reduced complexity | highest throughput | highest | early uncertainty pruning |

Use rule: in AI loops, run a small batch with PISO before promoting survivors to PIMPLE.

--- 

## What not to use it for
- Dominant steady objectives with cheap transients; use SIMPLE for throughput.
- Strongly stiff thermal-density or compressibility coupling where pressure-based variable-density solvers are needed.
- Very large physical steps where explicit-like phase growth becomes unstable; that is a PIMPLE/mesh boundary case.

--- 

## Source links

- OpenFOAM `pisoFoam` solver page: https://doc.openfoam.com/2306/tools/processing/solvers/rtm/incompressible/pisoFoam/
- OpenFOAM pressure-velocity algorithms overview: https://doc.openfoam.com/2212/tools/processing/solvers/pressure-velocity/
- OpenFOAM solver selection and pressure-velocity context: https://doc.openfoam.com/2312/tools/processing/solvers/pressure-velocity/
- OpenFOAM standard solver catalog (`pisoFoam` location): https://www.openfoam.com/documentation/user-guide/a-reference/a.1-standard-solvers
- Source path reference in OpenFOAM tree/API: `$FOAM_SOLVERS/incompressible/pisoFoam`,  
  https://api.openfoam.com/2212/pisoFoam_8C.html

--- 

## Related notes

- [[pisoFoam]]
- [[SIMPLE]]
- [[PIMPLE]]
- [[simpleFoam]]
- [[CFD Solvers]]
