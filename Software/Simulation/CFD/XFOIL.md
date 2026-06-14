# XFOIL ✈️

**XFOIL** is an open-source 2D airfoil analysis code developed at MIT from work by Mark Drela and contributors, built around a coupled inviscid panel method + boundary-layer model.  
It is optimized for quick iteration on lift/drag/pitching-moment polars and is widely used in preliminary wing and propeller airfoil design.

---

## Overview

XFOIL is not a full 3D RANS/LES solver.

It is primarily used for:

- attached or mildly separated 2D incompressible/subsonic flows,
- fast polar generation for lift/drag vs angle of attack,
- airfoil shape exploration (`NACA`, custom `*.dat` points, and modified foils),
- transition/turbulence modeling studies at preliminary fidelity.

The practical value is speed: you can evaluate many candidate sections before moving to higher-order 3D tools.

---

## Numerical model

The method combines:

- **Inviscid panel solution** for core circulation and pressure on the surface.
- **Integral boundary-layer solver** for skin friction, separation onset, transition control, and viscous drag components.
- **Compressibility correction** for low-Mach subsonic/transonic correction workflows.

Why this mix matters:

- It is much cheaper than volumetric CFD for early design loops.
- It gives interpretable section-level outputs (Cl, Cd, Cm, Cp distribution, transition).
- It is especially good when coupled to geometry optimizers that need thousands of evaluations.

---

## What you can compute

- `Cl`, `Cd`, `Cm` across angle or Reynolds sweeps.
- Surface pressure coefficient `Cp(x)` curves.
- Boundary-layer properties and transition prediction.
- Surface shape constraints and post-processed polar files.
- Inverse design modes for target pressure/moment/control targets (depending on version/build).

---

## Typical workflow

1. Define airfoil:
   - built-in NACA (`NACA 2412`) or custom `load filename.dat`.
2. Set geometry/solver controls:
   - Reynolds range, viscosity model, convergence settings, Mach setting.
3. Run operating-point sweep:
   - angle sweep or fixed alpha.
4. Record outputs:
   - polar file (Cl/Cd/Cm) + optional pressure/streamline dumps.
5. Export candidate foils to your CAD/meshing path for 3D pre-processing.

Common in tooling pipelines:

- Start with XFOIL for candidate filtering.
- Export top candidates to [[OpenFOAM]]/[[SU2]] for full-domain 3D studies.
- Use final candidates in shape/design tooling and downstream structural checks.

---

## Example session patterns

Non-interactive XFOIL input files are common for CI-style sweeps.
Exact command names can vary by installed build, but the structure is typically:

```text
LOAD some_airfoil.dat
PLOT
OPER
VISC
RE 2000000
M 0.0
ASEQ -8 14 0.5
PACC
polar.dat
```

```text
PACC
polar.dat

QUIT
```

Use whichever build syntax your package exposes (many downstream bundles keep command compatibility but vary exact prompts/help output).

---

## Use cases (what it is good at)

- Early airfoil optimization loops where you need many candidates per hour.
- Propeller and wind turbine section tuning.
- Sensitivity work over Re, Mach, and camber/thickness perturbations.
- Educational/research work needing quick interpretation of lift/drag and pressure behavior.

---

## Strengths

- Very fast for section-level aerodynamic analysis.
- Low setup overhead (no heavy meshing needed).
- Strong workflow fit for optimization and design-space exploration.
- Open and scriptable, so good for batch sweeps and regression tests.
- Useful bridge between hand analysis and full CFD.

---

## Weaknesses

- 2D-only core model; no full 3D wing effects by itself.
- Not ideal for strong shock, highly separated, or very high-Mach conditions.
- Drag prediction can be optimistic in some complex separated regimes compared to high-fidelity CFD.
- Geometry preprocessing quality matters a lot (smoothness and resolution of coordinates).

---

## CFD comparison chart

| Tool | Core model | Dimensionality | Typical runtime | Typical use | Best starting point for |
|---|---|---|---|---|---|
| XFOIL | Panel + boundary layer | 2D | Very fast | Airfoil polar generation | Early aero section design |
| SU2 | Finite volume solver | 2D/3D | Medium to slow | Compressible aerodynamics, adjoints | Full aerostructure design loops |
| OpenFOAM | Finite volume | 2D/3D | Medium to slow | General CFD workflows | Complex geometry + multiphysics |
| ANSYS Fluent | Finite volume + commercial ecosystem | 2D/3D | Slow | Industrial-grade multiphysics CFD | Production-grade engineering studies |
| COMSOL | Finite element / multiphysics | 2D/3D | Slow | Coupled thermal-structural-fluid studies | Fully coupled multi-physics problems |
| AVL / XFLR5 | Lifting-line/2D panel hybrids | 3D/2D | Fast | Preliminary aircraft layout / conceptual | Aircraft-level conceptual sizing |

---

## Installation and ecosystem

- Linux/macOS binaries commonly available via package managers or source builds in some ecosystems.
- Typical outputs are plain-text polars and coordinate/surface dumps compatible with scripts.
- Integration target: Python post-processing, optimization scripts, and aero shape generation workflows.

---

## Related notes

- [[CFD]]
- [[OpenFOAM]]
- [[SU2]]
- [[Finite Volume Method]]
- [[Turbulence Modeling]]
- [[Boundary Layer]]
- [[Airfoil]]
- [[ParaView]]
- [[Mesh Generation]]

## External resources

- https://web.mit.edu/drela/Public/web/xfoil/
- https://www.mh-aerotools.de/airfoils/ (legacy tooling context and examples)
- XFOIL community discussion forums and script examples
- XFOIL paper and user notes from Drela’s original documentation

## Hello-world / starter tutorials

- **Official getting-started references**
  - https://web.mit.edu/drela/Public/web/xfoil/
  - https://web.mit.edu/drela/Public/web/xfoil/xfoil_doc.txt
- **Install and build path**
  - https://airfoildatabase.readthedocs.io/en/latest/xfoil_installation.html
  - https://airfoildatabase.readthedocs.io/en/latest/installation.html
- **Community starter examples**
  - https://www.mh-aerotools.de/airfoils/ (legacy tooling context and examples)
