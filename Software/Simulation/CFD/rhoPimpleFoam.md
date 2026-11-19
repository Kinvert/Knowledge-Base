# rhoPimpleFoam

`rhoPimpleFoam` is the transient compressible OpenFOAM baseline.
It sits where steady compressible screening (`[[rhoSimpleFoam]]`) is not enough but you still want a general-purpose engineering workflow.

---

## What it solves

- Compressible, transient finite-volume equations for `p`, `U`, and `T`.
- Uses PIMPLE coupling with density/energy transport.
- Includes `thermophysicalModels` support for temperature-dependent properties.
- Transonic treatment is available via solver settings in `fvSolution`.

It is the standard upgrade path for unsteady bluff-body loops where:

- density changes affect wake size,
- inlet total pressure changes with operating point,
- thermal coupling shifts flow stability.

---

## Common workflow strengths

- Strong fit for loops that generate many geometry variants from CAD automation.
- Supports restart-friendly time histories for KPI extraction every `writeInterval`.
- Handles both RANS and some LES-compatible model stacks depending on turbulence config.

---

## What it struggles with

- High-Mach shock-dominated cases where density-based shocks dominate over pressure-based treatment; consider [[rhoCentralFoam]] there.
- Very stiff chemistry (reactive flames) should go to [[reactingFoam]] or [[fireFoam]].
- Setup cost is higher than incompressible family (thermophysical data required).

---

## CLI pattern

```bash
rhoPimpleFoam -case ../cases/loop_001 -parallel
rhoPimpleFoam -case ../cases/loop_001 -dry-run
```

Recommended controls:

- `controlDict`: `deltaT` and `adjustTimeStep` by Courant target.
- `fvSolution`: `PIMPLE` + `transonic yes` if needed.
- `thermophysicalModels`: EOS, cp, conductivity, viscosity laws.

---

## Comparison table

| Solver | Transient compressible | Shock handling | Complexity | Best use | Typical failure mode |
|---|---|---|---|---|---|
| [[rhoPimpleFoam]] | Yes | Limited | Medium | Engine/jetlet-like unsteady with heat transfer | Property table mismatch |
| [[rhoSimpleFoam]] | No | Limited | Low | Steady density-sensitive studies | Misses unsteady physics |
| [[rhoCentralFoam]] | Yes | Stronger for shocks | Medium-high | High-speed, compressible wave-dominated flow | More solver tuning required |
| [[pimpleFoam]] | No | N/A | Low-medium | Incompressible transients | No density coupling |
| [[interFoam]] | No (in docs for this variant) | N/A | High | Free-surface transients | Overkill for pure gas flow |
| [[OpenFOAM]] | Framework | Framework | Framework | Full pipeline choice | Wrong solver selection |

---

## Related notes

- [[OpenFOAM]]
- [[rhoSimpleFoam]]
- [[rhoCentralFoam]]
- [[pimpleFoam]]
- [[CFD]]

## External resources

- https://doc.openfoam.com/2212/tools/processing/solvers/rtm/compressible/rhoPimpleFoam/
- https://doc.openfoam.com/2306/tools/processing/solvers/rtm/compressible/
- https://doc.openfoam.com/2606/fundamentals/command-line/

