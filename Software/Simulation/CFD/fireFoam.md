# fireFoam

`fireFoam` is OpenFOAM’s combustion solver path for fire/flame problems with surface reaction, film, and pyrolysis support.
It is often selected when `reactingFoam` is not enough because walls and phase change are first-class effects.

---

## Core capabilities

- Transient, compressible, turbulent, and heat-transfer coupled.
- PIMPLE loop with multi-region options (fluid plus wall film/pyrolysis pathways in many case templates).
- Requires species fields, `thermophysicalModels`, `chemistryProperties`, `combustionProperties`, and `p_rgh`.

This is a high-fidelity thermal chemistry engine intended for safety/reaction-driven engineering.

---

## Why it’s special in automation loops

Compared with [[reactingFoam]], it adds wall/film-thermal complexity:

- near-wall mass transfer,
- pyrolysis progression,
- source term interactions between gas and solid-like boundaries in fire cases.

This is expensive, so use it as a “second-stage” solver after coarse filtering.

---

## Practical risks

- Sensitive initial temperature and species maps.
- Mesh near wall interfaces controls result quality more than anything else.
- Can produce very long convergence tails as reaction front propagates.
- Time-step limits are usually stricter than non-reactive runs.

---

## Automation fit

Pseudo-pipeline:

1. Build geometry candidates and generate consistent surface refinement in suspected flame zones.
2. Run a non-reactive pre-pass (`[[pimpleFoam]]`/`[[rhoPimpleFoam]]`) to filter obviously bad cases.
3. Promote winners to `fireFoam`.
4. Post-process temperature field and species depletion metrics with function objects.

Command:

```bash
fireFoam -case ../cases/firebench_A -parallel
```

---

## Comparison table

| Solver | Includes wall film/pyrolysis | Uses species transport | Thermal feedback | Typical cost | Best loop position |
|---|---|---|---|---|---|
| [[fireFoam]] | Yes | Yes | Strong | Very high | Late-stage high-fidelity validation |
| [[reactingFoam]] | No | Yes | Strong | High | Mid-stage thermal-reactive tests |
| [[buoyantPimpleFoam]] | No | No | Medium | Medium | HVAC/combustion chamber non-reactive heating |
| [[rhoPimpleFoam]] | No | No | Medium | Medium | First-stage unsteady compressible pruning |
| [[pimpleFoam]] | No | No | Limited | Medium | Non-reactive flow only |
| [[OpenFOAM]] | Framework | Framework | Framework | Varies | Topology-level decision |

---

## Related notes

- [[OpenFOAM]]
- [[reactingFoam]]
- [[buoyantPimpleFoam]]
- turbulence models (RANS/LES)
- [[CFD]]

## External resources

- https://doc.openfoam.com/2312/tools/processing/solvers/rtm/combustion/fireFoam/
- https://doc.openfoam.com/2306/fundamentals/command-line/
- https://doc.openfoam.com/2212/tools/processing/solvers/

