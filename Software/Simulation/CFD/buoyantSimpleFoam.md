# buoyantSimpleFoam

`buoyantSimpleFoam` is a steady-state incompressible-appearing but compressible-buoyant thermal solver using SIMPLE.
It is a practical bridge between pure aerodynamic screening and full conjugate/solids coupling.

---

## What it is

- Category: heat transfer + turbulence + thermophysical coupling.
- Steady state.
- Uses `SIMPLE` with compressible-buoyant assumptions and `p_rgh`.
- Handles natural/forced buoyancy through thermal expansion and gravity terms.

This is the steady thermal option when compressibility from temperature is not negligible, but you do not want the full complexity of `chtMultiRegionFoam`.

---

## Typical uses

- Heated cavity and diffuser thermal rise benchmarks.
- Ventilation-driven recirculation studies with heat sources.
- First-pass screening of thermal stratification and buoyant pressure loss.

---

## Common issues

- Over-converging on `p_rgh` when gravity and pressure BCs conflict.
- Radiation coupling may require careful model matching.
- Not a substitute for explicit solid conduction unless thermal lag is negligible.

---

## Example loop fragment

```bash
buoyantSimpleFoam -case ../cases/thermal_A
buoyantSimpleFoam -case ../cases/thermal_A -postProcess -func "surfaceFieldValue"
```

Checklist:

- `constant/thermophysicalModels`: equation of state and transport consistency.
- `constant/turbulenceProperties`: buoyant flow-compatible turbulence model.
- Gravity and reference pressure in setup files.

---

## Comparison table

| Solver | Steady/transient | Buoyancy | Solids coupling | Use if | Don’t use if |
|---|---|---|---|---|---|
| [[buoyantSimpleFoam]] | Steady | Yes | No | You need thermal buoyancy at low compute cost | Need wall conduction explicitly |
| [[buoyantPimpleFoam]] | Transient | Yes | No | Warm-up transients and unsteady heat release | You need steady-only KPI only |
| [[chtMultiRegionFoam]] | Transient | Yes | Yes | Fluid + solid heat-transfer coupling needed | Geometry too simple for solids |
| [[reactingFoam]] | Transient | Indirect | No | Chemistry dominates thermal feedback | Non-reactive heat rise only |
| [[rhoSimpleFoam]] | Steady | No (base formulation) | No | Non-buoyant compressible loops | Buoyancy dominates |
| Ansys Fluent | Varies | Varies | Varies | GUI-prepackaged thermal workflows | Pure CLI batch scripting priority |

---

## Related notes

- [[OpenFOAM]]
- [[buoyantPimpleFoam]]
- [[chtMultiRegionFoam]]
- turbulence models (RANS/LES)
- [[CFD]]

## External resources

- https://doc.openfoam.com/2306/tools/processing/solvers/rtm/heat-transfer/buoyantSimpleFoam/
- https://doc.openfoam.com/2606/fundamentals/command-line/

