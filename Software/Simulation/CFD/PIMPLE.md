# PIMPLE

`PIMPLE` is the **PISO + SIMPLE hybrid**, used when transient coupling needs more stability than PISO alone but you still want OpenFOAM-style pressure correction efficiency.

In OpenFOAM terms it wraps PISO-style inner corrections in an outer SIMPLE-like coupling loop, which helps for:
- stronger source-term coupling,
- larger time steps,
- transient runs where pure `pisoFoam` becomes unstable.

--- 

## Core mechanics

- Outer loop:
  - pressure correction and variable updates (SIMPLE-like),
  - optional under-relaxation behavior.
- Inner loop:
  - momentum solve,
  - one or more pressure-correction steps (PISO-like),
  - flux/velocity update.

`pimpleFoam` is the standard implementation, with variants (`rhoPimpleFoam`, `buoyantPimpleFoam`) handling extra coupling dimensions.

--- 

## Typical use for external single-phase incompressible flows

For parametric shape-to-CFD pipelines:
- use **PIMPLE** when unsteady dynamics are real but you need fewer instability failures than plain PISO.
- it is a good middle rung before full high-fidelity alternatives (grid adaptation, LES-heavy setups, body-fitted late-stage verification).

Compared to PISO:
- usually slower per sample,
- usually more robust at comparable setup quality when coupling is stiff.

--- 

## Control levers that move the speed/robustness trade

- `nOuterCorrectors`: major robustness lever for strong coupling.
- `nCorrectors`: PISO-correction count inside each outer cycle.
- `momentumPredictor`: on/off switch for predictor pattern in some variants.
- `maxCo` / `maxDeltaT`: cap physical step scale.

Start small (e.g., `nOuterCorrectors=1`, `nCorrectors=2`) and scale up only when instability appears.

--- 

## Comparison chart

| Solver strategy | Time mode | Coupling style | Expected stability | Runtime | Typical deployment |
|---|---|---|---|---:|---|
| `pimpleFoam` (`PIMPLE`) | transient | outer+inner loops | High | Medium | default transient single-phase benchmark rung |
| `pisoFoam` (`PISO`) | transient | inner-only correction | Medium | Medium-high | short-transient first checks |
| `simpleFoam` (`SIMPLE`) | steady | outer global | High | High | first-pass geometry ranking |
| `buoyantPimpleFoam` (`PIMPLE` + buoyancy) | transient | outer+inner + buoyancy models | Medium-high | Medium | convective/thermal external cases |
| `rhoPimpleFoam` (`PIMPLE` + variable density) | transient | outer+inner + thermophys. updates | Medium | Lower throughput | low-Mach compressibility effects |
| `overPimpleDyMFoam` (`PIMPLE` + moving mesh) | transient | moving mesh + PIMPLE | Medium | Medium-high | rotating/moving geometry cases |

--- 

## When PIMPLE is usually the wrong choice
- fully steady KPI studies where transient physics are negligible,
- very mild transients where PISO already gives clean results faster,
- cases where solver cost is already too high for the early sweep.

--- 

## Source links

- OpenFOAM `pimpleFoam` solver page: https://doc.openfoam.com/2306/tools/processing/solvers/rtm/incompressible/pimpleFoam/
- Pressure-velocity method overview and PIMPLE placement: https://doc.openfoam.com/2312/tools/processing/solvers/pressure-velocity/
- OpenFOAM application index (`pimpleFoam`, `buoyantPimpleFoam`, `rhoPimpleFoam`, `overPimpleDyMFoam`): https://doc.openfoam.com/2606/tools/processing/solvers/
- OpenFOAM API reference for `pimpleFoam`: https://api.openfoam.com/2212/pimpleFoam_8C.html
- Source path: `$FOAM_SOLVERS/incompressible/pimpleFoam`  
  Historical code reference: https://github.com/OpenFOAM/OpenFOAM-11/blob/master/applications/solvers/incompressible/pimpleFoam/pimpleFoam.C

--- 

## Related notes

- [[pimpleFoam]]
- [[buoyantPimpleFoam]]
- [[rhoPimpleFoam]]
- [[overPimpleDyMFoam]]
- [[PISO]]
- [[SIMPLE]]
- [[CFD Solvers]]
