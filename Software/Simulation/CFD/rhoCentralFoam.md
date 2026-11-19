# rhoCentralFoam

`rhoCentralFoam` is a compressible density-based solver based on central-upwind Kurganov-Tadmor fluxes.
It is the high-speed branch you reach for in OpenFOAM when shock resolution and compressible wave transport dominate.

---

## What it is (and is not)

- Compressible, transient, density-based workflow.
- Can use thermophysical transport models and turbulence models.
- Better suited than pressure-based compressible solvers when wave propagation / shocks are first-order design drivers.

It is still a full OpenFOAM case workflow (case dictionaries, mesh decomposition, and function objects), so it fits scripted loops if you prepare consistent case templates.

---

## Typical use cases

- External flow regimes with sharp acoustic/compressible features.
- Forward step/shocktube style sensitivity studies.
- Quick “can this geometry still choke/oscillate” screenings when `transonic` approximations are insufficient.

---

## Weak points

- Not the right first tool for low-Mach bluff-body loops with weak compressibility.
- Meshing and stabilization burden is higher than `rhoPimpleFoam` for many cases.
- You need to spend more time in `fvSchemes`/time stepping to avoid unphysical oscillations.

---

## Command and setup notes

```bash
rhoCentralFoam -case ../cases/eta_24deg
```

Core checks:

- Keep consistent time-step control around shock speed.
- Prefer stable flux and limiter settings in your case schemes.
- Compare with mesh-refined runs before trusting quantitative losses.

---

## Solver comparison for compressible design loops

| Solver | Physical model | Shock-capable | Best for low-speed design sweep | Best for transonic/supersonic sweep | Runtime profile |
|---|---|---|---|---|---|
| [[rhoCentralFoam]] | Density-based | Yes | Usually overkill | Strong | Medium-high |
| [[rhoPimpleFoam]] | Pressure-based PIMPLE | Limited | Good | Limited | Medium |
| [[rhoSimpleFoam]] | Pressure-based | No | Good | No | Low |
| sonicFoam | Legacy transonic framework | Partial | No | Historical/legacy | Medium |
| [[interFoam]] | Incompressible multiphase | No | No | No | High (interface) |
| [[reactingFoam]] | Combustion + compressible | Depends on setup | Weak | Weak | High |

---

## Related notes

- [[OpenFOAM]]
- [[rhoPimpleFoam]]
- [[reactingFoam]]
- [[CFD]]

## External resources

- https://doc.openfoam.com/2306/tools/processing/solvers/rtm/compressible/rhoCentralFoam/
- https://doc.openfoam.com/2306/fundamentals/command-line/
- https://doc.openfoam.com/2306/tools/processing/solvers/rtm/compressible/
