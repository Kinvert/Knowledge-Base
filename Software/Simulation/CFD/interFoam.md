# interFoam

`interFoam` is the canonical two-phase incompressible Volume of Fluid (VOF) solver in OpenFOAM.
It tracks immiscible phases (typically two-phase water/air) with interface capturing and pressure-velocity coupling.

---

## Core behavior

- Multiphase (incompressible), transient.
- Uses PIMPLE in the documented configuration.
- Uses `p_rgh` (pressure minus hydrostatic head) and requires correct initial `alpha` fields.
- Optional surface tension/contact-angle entries.

The mandatory fields are usually `p`, `p_rgh`, and `U`.

---

## What it is best at

- Dam-break, sloshing, free-surface shedding, and overtopping-like transient multiphase behavior.
- Geometry parameter studies where interface motion is a KPI (e.g., spray intrusion, free-surface height, entrainment path).
- Diffuser and cavity wetting/drying studies when phase splitting matters.

For bluff-body drag loops, this is only for cases where the flow is not single-phase.

---

## Practical limitations

- High sensitivity to mesh orthogonality and interface thickness settings.
- Numerical damping/oscillation tradeoff is common; tuning `cAlpha`, compression terms, and interface compression transport can dominate.
- Longer run times than equivalent single-phase solvers.
- Free-surface BC quality can silently break convergence.

---

## Automation considerations

You should parameterize not just geometry:

- initial phase field location (`setFieldsDict` / `topoSet`),
- contact angle / tension coefficients,
- interface compression coefficients,
- timestep control around interface travel distance.

Sample:

```bash
interFoam -case ../cases/water_oil_A -parallel
interFoam -case ../cases/water_oil_A -postProcess -func surfaceFieldValue
```

Use `checkCase` checks before full runs:

- `1 0`/`0` fields exist for required phase and pressure variables,
- patch names are consistent,
- `p_rgh` BCs and `alpha.phaseChange` terms align with gravity direction.

---

## Comparison table (where `interFoam` fits)

| Solver | Flow class | Phase handling | Computation cost | Automation complexity | Recommended pre-filter |
|---|---|---|---|---|---|
| [[interFoam]] | Incompressible multiphase transient | VOF, 2 immiscible | High | Medium-high | Run single-phase surrogate first ([[simpleFoam]]) |
| [[icoFoam]] / [[pisoFoam]] / [[pimpleFoam]] | Single-phase | None | Low-medium | Low | Use if no interface physics |
| [[compressibleInterFoam]] | Compressible multiphase | VOF-like family | Very high | Very high | Use only if gas compressibility is critical |
| [[chtMultiRegionFoam]] | Fluid + solid | N/A | Very high | High | Use for conjugate effects |
| [[OpenFOAM]] | Framework | Framework | Framework | Framework | Wrong class kills loop |
| COMSOL Multiphysics | Multiphase (via modules) | Depending module | Very high | GUI-heavy | Non-VOF workflow |

---

## Related notes

- [[OpenFOAM]]
- [[compressibleInterFoam]]
- turbulence models (RANS/LES)
- [[Mesh Generation]]
- [[chtMultiRegionFoam]]

## External resources

- https://doc.openfoam.com/2312/tools/processing/solvers/rtm/multiphase/interFoam/
- https://doc.openfoam.com/2606/fundamentals/command-line/
- https://doc.openfoam.com/2212/tools/processing/solvers/

