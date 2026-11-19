# buoyantPimpleFoam

`buoyantPimpleFoam` extends the buoyancy thermal framework into transient operation using PIMPLE.
This is the unsteady counterpart to [[buoyantSimpleFoam]] and is commonly the first “thermal-transient” pick when natural convection, heating pulses, or flow cycling matters.

---

## Problem class

- Heat transfer + buoyancy + compressible-fluid assumptions.
- Transient with `PIMPLE` coupling.
- Supports turbulence + thermophysical models + `fvOptions`.
- Uses `p_rgh` with gravity-aware pressure handling.

It is often the workhorse for time-dependent thermal convection in open channels, diffusers, and cavity-like regions.

---

## Good for

- Periodic heating/cooling in airflow loops.
- Thermal plume formation and rise dynamics around geometry features.
- Design-of-experiments on surface heat flux levels combined with airflow path changes.

---

## Poor fit

- Not for solid-conduction-dominated targets (solid thermal inertia is not explicit).
- Not the first pick for high-Mach compressible transient flow.
- Can be expensive if transient statistics require long physical times.

---

## Workflow

```bash
buoyantPimpleFoam -case ../cases/vent_A -parallel
buoyantPimpleFoam -case ../cases/vent_A -postProcess -func temperature
```

Important controls:

- `system/controlDict`: physical time horizon, adaptive `deltaT`.
- `system/fvSolution`: `PIMPLE` outer correction count.
- `constant/thermophysicalModels`: fluid properties and heat source coupling.

---

## Comparison table (transient thermal workflow)

| Solver | Heat transfer | Transient | Buoyancy | Cost | When to switch |
|---|---|---|---|---|---|
| [[buoyantPimpleFoam]] | Strong | Yes | Yes | Medium-high | to `chtMultiRegionFoam` for solids |
| [[buoyantSimpleFoam]] | Strong | No | Yes | Medium | if transients are irrelevant |
| [[reactingFoam]] | Strong | Yes | Indirect | High | add chemistry requirements |
| [[rhoPimpleFoam]] | Yes | Yes | No | Medium | no buoyancy/gravity forcing needed |
| [[pimpleFoam]] | Optional | Yes | No | Medium | no thermal coupling |
| [[chtMultiRegionFoam]] | Very strong | Yes | Yes | High | includes explicit solids |

---

## Related notes

- [[OpenFOAM]]
- [[buoyantSimpleFoam]]
- [[chtMultiRegionFoam]]
- turbulence models (RANS/LES)
- [[CFD]]

## External resources

- https://doc.openfoam.com/2212/tools/processing/solvers/rtm/heat-transfer/buoyantPimpleFoam/
- https://doc.openfoam.com/2306/fundamentals/command-line/

