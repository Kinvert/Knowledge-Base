# potentialFoam

`potentialFoam` is the OpenFOAM legacy incompressible potential-flow solver used to generate a fast, stable initial velocity/flux field before switching to full Navier–Stokes solvers.

The OpenFOAM v13 standard solvers page classifies `potentialFoam` as a potential flow solver that solves for a velocity potential and reconstructs a flux field from that field.

---

## What it solves

`potentialFoam` is best understood as a setup/initialization solver:

- Incompressible flow assumptions.
- Irrotational approximation (no vorticity transport).
- Velocity comes from scalar potential reconstruction, not direct momentum coupling.
- Very fast per-iteration solve path compared with transient coupled solvers.

In OpenFOAM documentation, `potentialFoam` appears in the standard solver listing and in legacy-application context for modules-era workflows.

---

## Solver role in a practical external single-phase stack

A likely workflow for external incompressible geometry screening:

1. `potentialFoam` for a clean start field.
2. `simpleFoam` for fast steady screening.
3. `pisoFoam` if startup and transient response matters.
4. `pimpleFoam` when robustness or stronger pressure-velocity coupling is needed.

This keeps early runs cheap while still giving realistic starting conditions for later stages.

---

## Comparison against likely alternatives

| Solver | Flow model | Main purpose | Steady/Transient | Coupling strength | Typical role |
|---|---|---|---|---|---|
| [[potentialFoam]] | incompressible potential | potential field initialization | steady-ish | low (potential-only) | cheap seeding |
| [[simpleFoam]] | incompressible turbulent/RANS-style pressure correction | steady baseline CFD | steady | medium | fast geometry ranking |
| [[pisoFoam]] | incompressible transient | time-dependent flow capture | transient | medium | first transient rung |
| [[pimpleFoam]] | incompressible transient (outer+inner corrections) | robust transient coupling | transient | high | stronger convergence robustness |
| [[icoFoam]] | incompressible laminar transient | basic transient laminar flow | transient | low | low-model laminar baseline |
| [[laplacianFoam]] | scalar Laplace-type solve | scalar potential style testing | steady-like | low | benchmarking scalar equation behavior |

---

## When to use it

Use `potentialFoam` when:

- you want physically plausible pressure/flux initialization before main solver.
- external flow is in the pre-design stage and you need quick convergence to seed the case.
- you need a reproducible starting potential field for a fixed mesh/BC setup.

Avoid it when:

- separation, wake dynamics, viscous losses, or turbulence feedback are core outputs.
- you already need transient accuracy for forces, loads, or vortex dynamics.

---

## Practical settings

Common patterns:

- keep it as a dedicated preprocessing step,
- check residual trend and continuity behavior,
- hand off to `simpleFoam`/`pisoFoam`/`pimpleFoam` from the same case folders.

Example control flow:

```bash
cd ../case_A
potentialFoam
simpleFoam -case .
# or
pisoFoam -case .
```

---

## Source and code links

- OpenFOAM v13 standard solvers: https://doc.cfd.direct/openfoam/user-guide-v13/standard-solvers
- Generated class/API reference for `potentialFoam.C`: https://api.openfoam.com/2306/potentialFoam_8C.html
- OpenFOAM 7 legacy source file: https://github.com/OpenFOAM/OpenFOAM-7/blob/master/applications/solvers/basic/potentialFoam/potentialFoam.C
- OpenFOAM 7 folder (legacy source tree): https://github.com/OpenFOAM/OpenFOAM-7/tree/master/applications/solvers/basic/potentialFoam

---

## Related notes

- [[OpenFOAM]]
- [[simpleFoam]]
- [[pisoFoam]]
- [[pimpleFoam]]
- [[icoFoam]]
- [[laplacianFoam]]
- [[CFD Solvers]]
