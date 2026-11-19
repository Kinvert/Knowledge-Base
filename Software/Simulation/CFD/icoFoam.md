# icoFoam

`icoFoam` is the lightweight transient incompressible solver for laminar flows using PISO.
It has minimal model stack and is often used as the lowest-complexity benchmark for unsteady geometry sweeps.

---

## Solver profile

- Incompressible transient
- PISO pressure-velocity coupling
- Minimal to no turbulence model coupling by default (laminar formulation)

The mandatory fields are `p` and `U`.
In practice this makes it a clean “first principles” transient test case before you add turbulence complexity.

---

## Where it shines

- Laminar validation with reproducible mesh-converged transients.
- Baseline comparison for startup/closure of a new parameter set before moving to turbulent models.
- Very cheap per-step runtime.

Common pattern:

1. Generate a coarse-to-fine mesh sequence.
2. Run [[icoFoam]] for first 100–500 time steps.
3. Compare velocity/pressure transients to `pisoFoam` or [[pimpleFoam]] reference.
4. Promote to turbulence-capable solver only if errors are physical.

---

## Where it fails for optimization loops

- Laminar assumption often invalid for bluff-body turbulence.
- Missing turbulence model means underprediction of loss/drag in most practical industrial use cases.
- Not suitable for heated/compressible/combustion studies.

---

## Command-line

```bash
icoFoam -case ../cases/bluff_variant_A
```

Important run controls:

- keep `deltaT` conservative until residual behavior is stable,
- monitor `maxCo`,
- if `continuity errors` grow: refine spatial scheme, time step, or turbulence model choice.

---

## Practical comparison table

| Solver | Turbulence in base setup | Primary use | Typical runtime | Upgrade path |
|---|---|---|---|---|
| [[icoFoam]] | Laminar (no turb model by default) | Quick transient baseline | Very low | [[pisoFoam]] / [[pimpleFoam]] |
| [[pisoFoam]] | Optional turbulence | General transient setup with RANS models | Low | Add buoyancy or multiphase coupling |
| [[pimpleFoam]] | Optional turbulence + PIMPLE | Robust transient wake/recirculation | Medium | Keep if convergence stalls |
| [[simpleFoam]] | Optional turbulence | Steady-state screening | Very low | Use after transients settle |
| [[interFoam]] | Turbulence + multiphase | Free-surface transient work | High | Separate if two-phase required |
| [[reactingFoam]] | Turbulence + chemistry | Reactive transient | High | Only for combustion cases |

---

## Related notes

- [[OpenFOAM]]
- [[pisoFoam]]
- [[pimpleFoam]]
- turbulence models (RANS/LES)
- [[CFD]]

## External resources

- https://doc.openfoam.com/2306/tools/processing/solvers/rtm/incompressible/icoFoam/
- https://doc.openfoam.com/2306/fundamentals/command-line/

