# compressibleInterFoam

`compressibleInterFoam` is the compressible, non-isothermal VOF solver family for immiscible fluids.
It is the compressible extension of the interfacial multiphase idea and is usually reached after [[interFoam]] when gas/liquid density changes are material to interface dynamics.

---

## Core behavior

- Multiphase, compressible, transient.
- Volume-of-fluid interface capture for two (or more) immiscible phases.
- Thermophysical coupling of non-isothermal energy fields.
- Phase-specific transport/turbulence handling in `compressibleInterFoam` family variants.

The docs available in the OpenFOAM API family describe it as “two compressible, non-isothermal immiscible fluids using VOF,” with iso-advector variants (`compressibleInterIsoFoam`) available in newer branches.

---

## When this is the right choice

- Bubble/slug flow where density ratio and compressibility both matter.
- High-pressure phase-change-driven interface problems where `interFoam` is under-predicting energy coupling.
- Advanced thermal transients in propulsive or flashing multiphase chambers.

---

## Why it is hard to automate

- Phase thermodynamics are sensitive to consistent property dictionaries.
- Interface and temperature coupling creates a wider acceptance-rejection envelope than `interFoam`.
- Mesh quality and interface compression tuning are more expensive and more case-specific.

---

## Command/automation note

There is no official OpenCFD `doc.openfoam.com` page snippet as accessible as other solvers in this pass, so treat version-specific behavior as build-dependent.
Use API/source references to confirm exact variable names and options for your distribution before automated generation.

Typical wrapper pattern:

```bash
compressibleInterFoam -case ../cases/flash_A
compressibleInterFoam -case ../cases/flash_A -postProcess -func p
```

---

## Comparison table

| Solver | Phase model | Compressibility | Heat transfer | Setup cost | Best place in workflow |
|---|---|---|---|---|---|
| [[compressibleInterFoam]] | VOF | Yes | Yes | Very high | late-stage specialized study |
| [[interFoam]] | VOF | No (base) | Limited by model choice | High | first-pass if gas compressibility is small |
| compressibleInterIsoFoam (legacy / external branches) | VOF + isoAdvector | Yes | Yes | Very high | interface advection-sensitive cases |
| [[rhoPimpleFoam]] | Single-phase | Yes | Yes | Medium | not for two-phase interfaces |
| [[chtMultiRegionFoam]] | None, region-based | Usually no | Strong | High | if solids coupling matters |
| [[pimpleFoam]] | None | No | Optional | Medium | single-phase alternatives |

---

## Related notes

- [[OpenFOAM]]
- [[interFoam]]
- [[rhoPimpleFoam]]
- [[fireFoam]]
- [[CFD]]

## External resources

- https://api.openfoam.com/2506/dir_fb05bcc430bbe51efa429e7fa7725607.html
- https://manpages.debian.org/trixie/openfoam/compressibleInterFoam.1.en.html
