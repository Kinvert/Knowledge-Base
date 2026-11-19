# reactingFoam

`reactingFoam` is the compressible heat-transfer/chemistry family solver for non-premixed or partly premixed reacting flows in OpenFOAM.
It is for high-fidelity chemical/thermal coupling where reaction source terms are essential to momentum and density evolution.

---

## What it models

- Compressible, transient transport for `p`, `U`, `T` plus species scalars.
- Chemistry through `chemistryProperties` and thermophysical models.
- Turbulence + combustion options selected through dictionaries.
- PIMPLE-style coupling in documented configuration.

This is a heavier solver by setup and run cost, but gives you chemically coupled forcing in a single workflow.

---

## When to prefer `reactingFoam`

- Flame anchoring sensitivity analysis with temperature coupling.
- Reactive bluff-body with heat-release-modified recirculation.
- Parametric species/stoichiometry sweeps (e.g., equivalence ratio).

Its role in optimization loops is usually:

1. Use single-phase/adiabatic surrogate (`[[pisoFoam]]` or `[[rhoPimpleFoam]]`) for geometry pruning.
2. Promote top candidates into `reactingFoam` for source-term closure.

---

## What can break fast

- Bad initial species composition and thermo tables.
- Poor coupling stiffness between chemistry and flow causing CFL collapse.
- Inadequate time stepping near ignition/extinction events.
- Mesh regions with too much skew for scalar transport.

---

## Common run pattern

```bash
reactingFoam -case ../cases/reactive_A
reactingFoam -case ../cases/reactive_A -postProcess -func speciesFlux
```

Tune:

- `constant/chemistryProperties`: mechanism + kinetics library.
- `constant/thermophysicalModels`: compressible thermo + transport.
- `system/controlDict`: adaptive time-step for stiffness.

---

## Comparison table

| Solver | Chemistry | Thermal coupling | Runtime | Best use | Not recommended when |
|---|---|---|---|---|---|
| [[reactingFoam]] | Yes | Yes | High | Premixed/non-premixed reactive design loops | Non-reactive geometry filtering |
| [[fireFoam]] | Yes | Yes | High | Fire/wall-driven pyrolysis setups | Pure jet/engine core no wall pyrolysis |
| [[rhoPimpleFoam]] | No | Yes | Medium | Non-reactive compressible unsteady | Chemistry ignored |
| [[pimpleFoam]] | No | Optional | Medium | Non-reactive turbulent transients | Missing species sources |
| [[buoyantPimpleFoam]] | No | Yes | Medium-high | Buoyant combustion-coupled chamber loops | No chemistry |
| [[OpenFOAM]] | Framework | Framework | Framework | Framework-level choice | Solver mismatch |

---

## Related notes

- [[OpenFOAM]]
- [[fireFoam]]
- [[rhoPimpleFoam]]
- turbulence models (RANS/LES)
- Chemistry

## External resources

- https://doc.openfoam.com/2312/tools/processing/solvers/rtm/combustion/reactingFoam/
- https://doc.openfoam.com/2606/fundamentals/command-line/
- https://doc.openfoam.com/2212/tools/processing/solvers/rtm/compressible/
