# pimpleFoam

`pimpleFoam` is the incompressible transient solver built on the PIMPLE algorithm (PISO inside a SIMPLE outer loop).
It is the default “high-value” OpenFOAM choice for bluff-body aerodynamics where you want unsteady behavior without going full LES research complexity out of the gate.

---

## Core model behavior

- Incompressible, transient finite-volume solver.
- Uses `PIMPLE` coupling (multiple outer corrections + inner pressure correction).
- Turbulence model support through `constant/turbulenceProperties` (RANS and LES variants depending on dictionaries and case setup).

That combo makes it a practical middle ground:
- more stable than pure PISO under challenging flows,
- less intrusive than fully custom compressible/combustion frameworks.

---

## When `pimpleFoam` is the right tool

- Geometry-driven transients (diffuser opening angle sweeps with unsteady wake growth).
- Moderate Re sweeps where convergence must finish in fixed wall-clock windows.
- Cases with moving inlet conditions or cyclic boundary behavior requiring repeated correction per time step.

It is especially useful in design loops because it supports:

- parameterized case directories,
- short convergence windows with residual caps,
- and scripted restarts if the design loop is interrupted.

---

## When to avoid it

- Long fully developed unsteady turbulent statistics where incompressibility assumptions break down.
- Compressible gas flows with pressure-density coupling.
- Strongly stiff chemically reacting setups (use [[reactingFoam]] or [[fireFoam]]).

---

## Workflow example

```bash
cd ../cases/bluff_b
pimpleFoam -case . -parallel
pimpleFoam -case . -postProcess -func sample -time 100
```

Key dicts:

- `system/fvSolution`: `PIMPLE` sub-dict (`nOuterCorrectors`, `nCorrectors`)
- `system/controlDict`: `maxCo`, `maxDeltaT`
- `system/fvSchemes`: advection + gradient choices for unsteady stability

---

## High-value comparison (bluff body + wake metrics)

| Solver | Flow class | Robustness | Cost per step | What it changes fast | Known setup pain |
|---|---|---|---|---|---|
| [[pimpleFoam]] | Incompressible transient | High | Medium | wake and recirculation startup | More tunables in `fvSolution` |
| [[pisoFoam]] | Incompressible transient | Medium | Lower | clean startup transient fields | Less robust on stiff transients |
| [[simpleFoam]] | Incompressible steady | Medium | Low | steady KPIs only | Not transient |
| [[rhoPimpleFoam]] | Compressible transient | Medium | High | compressible energy effects | Requires thermal setup |
| [[interFoam]] | Incompressible multiphase transient | Medium | High | free-surface timing/phase fractions | Interface stabilization |
| [[OpenFOAM]] | Framework | Varies | Varies | entire pipeline | solver choice risk |

---

## Related notes

- [[OpenFOAM]]
- [[pisoFoam]]
- [[simpleFoam]]
- turbulence models (RANS/LES)
- [[buoyantPimpleFoam]]

## External resources

- https://doc.openfoam.com/2306/tools/processing/solvers/rtm/incompressible/pimpleFoam/
- https://doc.openfoam.com/2212/tools/processing/solvers/rtm/compressible/rhoPimpleFoam/
- https://doc.openfoam.com/2606/fundamentals/command-line/

