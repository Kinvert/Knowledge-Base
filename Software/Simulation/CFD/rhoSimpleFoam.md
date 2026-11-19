# rhoSimpleFoam

`rhoSimpleFoam` is OpenFOAM’s steady-state compressible solver in the same pressure-coupling family as [[simpleFoam]], but with density and thermophysical coupling.
It is the usual first step when compressibility becomes non-negligible and you still want steady screening.

---

## What it does

- Compressible, steady transport equations.
- Uses SIMPLE with thermophysical state support.
- Supports optional `transonic` mode in solver settings depending on case setup.

Mandatory fields include pressure `p`, velocity `U`, and temperature `T`, plus `thermophysicalModels`.

---

## Typical use in diffuser/CHT design loops

- Quasi-steady heated or cooled flow where density variation impacts separation size and losses.
- Fan/cavity ventilation-type parametric sweeps where exit temperature shift matters.
- Preliminary thermal coupling checks before moving to full multi-region heat transfer.

---

## Limitations

- Transient behavior is not modeled.
- Requires careful property consistency: equation of state, specific heat, viscosity models.
- Can be brittle with under-resolved thermal gradients.

---

## CLI

```bash
rhoSimpleFoam -case ../cases/diffuser_A -parallel
rhoSimpleFoam -case ../cases/diffuser_A -postProcess -func flowRate
```

Checklist:

- `constant/thermophysicalModels`: `thermoType`, species if needed.
- `system/fvSolution`: `SIMPLE` entries for pressure correction.
- `system/fvSchemes`: `ddt` for steady runs is typically steady-state settings.

---

## Comparison table

| Solver | Compressibility | Heat transfer | Time dependence | Common mismatch mode | Upgrade path |
|---|---|---|---|---|---|
| [[rhoSimpleFoam]] | Yes | Yes | Steady | Temperature-property inconsistency | [[rhoPimpleFoam]] |
| [[simpleFoam]] | No | Optional via user fields | Steady | density-based bias in thermal problems | [[rhoSimpleFoam]] |
| [[rhoPimpleFoam]] | Yes | Yes | Transient | More expensive per sample | [[rhoCentralFoam]] for high-speed shocks |
| [[buoyantSimpleFoam]] | Compressible/buoyant | Yes | Steady | gravity/hydrostatic handling | [[chtMultiRegionFoam]] if solids matter |
| [[interFoam]] | No (incompressible) | No by default | Transient | two-phase overkill for pure gas flow | Use if free surface exists |
| [[OpenFOAM]] | Framework | Framework | Framework | solver mismatch | use proper workflow gating |

---

## Related notes

- [[OpenFOAM]]
- [[rhoPimpleFoam]]
- [[buoyantSimpleFoam]]
- turbulence models (RANS/LES)
- [[CFD]]

## External resources

- https://doc.openfoam.com/2306/tools/processing/solvers/rtm/compressible/rhoSimpleFoam/
- https://doc.openfoam.com/2306/tools/processing/solvers/rtm/compressible/
- https://doc.openfoam.com/2306/fundamentals/command-line/

