# chtMultiRegionFoam

`chtMultiRegionFoam` couples multiple compressible fluid and solid regions in one transient OpenFOAM case.
It is the primary OpenFOAM workhorse for conjugate heat transfer when you need explicit solid conduction in the same automated solve.

---

## Core model

- Multi-region, transient pressure-based finite-volume coupling.
- Uses PIMPLE.
- Supports fluid regions (with `p`, `p_rgh`, `U`, `T`) and solid regions (`T`).
- Requires region-aware dictionaries and coupling interfaces in `constant` and `system`.

This is an explicit extension of single-region thermal frameworks for CFD-CFD/solids co-simulation in one case folder.

---

## Why this is important for real design loops

- Correct wall thermal inertia without replacing fluid-only solver assumptions.
- Realistic heat loss through fins, walls, and embedded solid features.
- Better basis for design-of-experiments where thermal compliance constraints matter.

For blower/diffuser loops, it is typically your step-up from buoyancy-only fluid models when you start checking structure temperature limits.

---

## Complexity and common setup failures

- Region naming mistakes (`air`, `solid`, `porous`) are the #1 failure mode.
- Interface patch naming mismatches silently break heat-flux continuity.
- Much more dictionary editing overhead: each region has own transport and schemes.
- Longer run times due to coupled implicit heat transfer.

---

## CLI and execution

```bash
chtMultiRegionFoam -case ../cases/heatExchanger_A -parallel
```

Use:

- `system/controlDict` with consistent global and per-region write controls,
- `system/fvSchemes` and `fvSolution` in per-region or region-specific dictionaries,
- `constant/*/thermophysicalModels` and interface data for every region.

---

## Comparison table

| Solver | Multi-region solids | Transient | Buoyancy | Typical KPI quality | Cost |
|---|---|---|---|---|---|
| [[chtMultiRegionFoam]] | Yes | Yes | Yes | High for thermal gradients | Very high |
| [[buoyantPimpleFoam]] | No | Yes | Yes | Medium for flow field, lower for solids | Medium-high |
| [[buoyantSimpleFoam]] | No | No | Yes | Medium for steady thermal fields | Medium |
| [[fireFoam]] | Indirect | Yes | Limited | High for reactive thermal fields | High |
| [[pimpleFoam]] | No | Yes | No | Medium | Medium |
| COMSOL Multiphysics | Yes | Yes | Yes | High for coupled multiphysics | High/GUI-heavy |

---

## Related notes

- [[OpenFOAM]]
- [[buoyantPimpleFoam]]
- [[buoyantSimpleFoam]]
- [[CFD]]
- [[Mesh Generation]]

## External resources

- https://doc.openfoam.com/2306/tools/processing/solvers/rtm/heat-transfer/chtMultiRegionFoam/
- https://doc.openfoam.com/2606/fundamentals/command-line/

