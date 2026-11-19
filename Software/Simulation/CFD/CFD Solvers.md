# CFD Solvers

CFD simulation workflows usually pick a solver by equation coupling, not by brand. OpenFOAM explicitly documents this as the standard pattern: choose a solver family from standard categories instead of expecting one solver to handle every flow regime [A.1 Standard solvers](https://www.openfoam.com/documentation/user-guide/a-reference/a.1-standard-solvers).

This note is a hub for spokes: pressure solvers, velocity strategies, density handling, energy/species transport, and swirl/rotation-oriented setups.

---

## 🌐 First pass: compare solver families

| Solver family | Primary unknowns | Time behavior | Representative OpenFOAM solvers | Typical speed | Typical accuracy / robustness trade-off | Notes |
|---|---|---|---|---|---|---|
| Steady pressure-based | Momentum + continuity (`U`, `p`) | Steady | `simpleFoam`, `buoyantSimpleFoam`, `rhoSimpleFoam` | Usually fastest per iteration | Good for converging to steady state; convergence quality depends on relaxation and mesh quality | SIMPLE pressure-velocity coupling; no physical time integration |
| Transient pressure-based | Momentum + continuity (`U`, `p`) | Explicit/implicit transient loops | `icoFoam`, `pisoFoam`, `interFoam` | Higher per-step cost than simpleFoam | Better capture of unsteady transients than SIMPLE | PISO coupling; `interFoam` adds interface transport |
| Transient hybrid pressure-based | Momentum + continuity + optional thermophysical transport (`U`, `p`, `T`, etc.) | Transient with inner/outer coupling loops | `pimpleFoam`, `rhoPimpleFoam`, `buoyantPimpleFoam` | Moderate to high | Better pressure-velocity-momentum coupling than pure PISO for large time steps or strong source terms | PIMPLE combines SIMPLE + PISO logic |
| Pressure-based compressible (low-to-transonic) | `U`, `p`, `ρ`/`T` | Steady/transient depending on variant | `rhoSimpleFoam`, `rhoPimpleFoam`, `buoyantPimpleFoam` | Moderate | Handles variable density with pressure correction style coupling | Uses pressure-based approach and thermophysical models |
| Density-based compressible | Full compressible state set | Usually transient | `rhoCentralFoam`, `sonicFoam` | Higher per-step than pressure-based methods | Better shock handling and higher-Mach behavior due density-based flux formulation | Central-upwind compressible framework |
| Multiphase interface (incompressible) | Velocity, pressure, phase fraction | Transient | `interFoam`, `interFoam` variants | Moderate to high | Accurate interface transport with VOF, coupled with standard pressure algorithms | Interface-aware pressure formulation includes `p_rgh` in some setups |
| Multiphase interface (compressible) | Two-phase density/energy/momentum fields | Usually transient | `compressibleInterFoam`, `compressibleInterDyMFoam`, `compressibleMultiphaseInterFoam` | High | Captures phase-change and density variations with additional phase transport terms | Heavier memory/solve cost from multiple fields |
| Heat transfer and buoyancy | Momentum + continuity + energy | Steady/transient | `buoyantSimpleFoam`, `buoyantPimpleFoam`, `chtMultiRegionFoam` | Moderate to high | Strongly coupled thermal buoyancy terms can improve physical realism but slow convergence | Needs turbulence + thermophysical setup |
| Combustion/species coupling | Flow + energy + species/reactions | Transient | `reactingFoam`, `fireFoam`, related modules | High | Most expensive among common OpenFOAM classes because of chemistry and source stiffness | Often solver choice is driven by reaction model and chemistry fidelity |
| Cross-solver (single-framework comparison) | Depends on formulation family | Varies | `simpleFoam`, `pisoFoam`, `pimpleFoam`, `rhoCentralFoam` | Depends on equation count/linear solves | OpenFOAM does not provide a single universal solver [A.1 Standard solvers](https://www.openfoam.com/documentation/user-guide/a-reference/a.1-standard-solvers) | Useful check: choose by flow type + coupling needs |

---

## 🌊 Pressure solvers

In this note, pressure solvers are treated as **pressure-velocity coupling strategies**, not separate equations-only solvers. OpenFOAM solver pages explicitly label this split by algorithm.

### Pressure-coupling comparison

| Method | Solver examples | When it applies | Speed behavior | Accuracy/robustness notes |
|---|---|---|---|---|
| `SIMPLE` | `simpleFoam`, `buoyantSimpleFoam`, `rhoSimpleFoam`, `buoyantPimpleFoam` uses PIMPLE only via hybrid mode | Steady or pseudo-transient loops | Lowest cost per global outer cycle | Reliable steady-state baseline; accuracy can be slower to converge without careful under-relaxation |
| `PISO` | `pisoFoam`, `icoFoam`, `interFoam` | Transient incompressible flow where pressure correction is repeated inside each step | More work per time step than SIMPLE | Strongly supports unsteady continuity enforcement with fewer global under-relax cycles |
| `PIMPLE` | `pimpleFoam`, `rhoPimpleFoam`, `buoyantPimpleFoam` | Transient and difficult-to-couple flows | Higher cost per step than pure PISO | Stronger coupling than PISO and more transient flexibility than SIMPLE |
| Density-based alternative | `rhoCentralFoam`, `sonicFoam`, `sonicDyMFoam` | Compressible waves/shocks and transonic/supersonic regimes | Higher per-step and higher linear-system cost | No SIMPLE/PISO-style pressure correction loop; compressibility is embedded in full conservative system |

`simpleFoam` and `pisoFoam` pages show their variable sets and clearly tie `simpleFoam` to SIMPLE and `pisoFoam` to PISO [simpleFoam](https://doc.openfoam.com/2306/tools/processing/solvers/rtm/incompressible/simpleFoam/) and [pisoFoam](https://doc.openfoam.com/2306/tools/processing/solvers/rtm/incompressible/pisoFoam/).

OpenFOAM Quickstart and algorithm pages also describe the three core options as SIMPLE, PISO, and PIMPLE with a hybrid recommendation path [OpenFOAM Quickstart](https://doc.openfoam.com/2306/quickstart/), [pressure-velocity algorithms](https://doc.openfoam.com/2312/tools/processing/solvers/pressure-velocity/).

### Pressure summary

- `SIMPLE` is often first choice for stable steady solves and low-cost development loops.
- `PISO` is the safer default for short-time transient momentum transport with stricter continuity enforcement.
- `PIMPLE` is typically preferred when transient stability, larger time steps, and coupling strength are both important.
- `rhoCentralFoam` is usually brought in when shock-capturing and compressibility effects become primary.

---

## 🧭 Velocity solvers

Velocity is not usually a separate top-level algorithm family in OpenFOAM; most named solvers solve `U` as part of a coupled momentum block with either PISO/PIMPLE/SIMPLE control. Even then, velocity transport quality depends on schemes, turbulence model, mesh, and source terms.

### Velocity-focused comparison

| Class | Representative solvers | Momentum treatment | Stability controls |
|---|---|---|---|
| PISO family | `icoFoam`, `pisoFoam`, `interFoam`, some multiphase/combustion variants | Momentum equation solved with PISO pressure correction loop | Needs transient stability settings and consistent `fvSchemes` |
| PIMPLE family | `pimpleFoam`, `rhoPimpleFoam`, `buoyantPimpleFoam`, turbine/agitation cases using rotating meshes | Momentum + pressure corrections with outer coupling loops | Additional outer correctors can reduce split error at higher Courant numbers |
| Steady segregated | `simpleFoam`, `buoyantSimpleFoam`, `rhoSimpleFoam` | Momentum equation under SIMPLE control | Usually lower memory footprint per outer loop |
| Potential/initialization path | `potentialFoam` | Solves velocity potential first, then derives velocity for initialization | Useful for inlet/boundary consistency before full momentum solve |
| Scalar-transport style coupling | `scalarTransportFoam`, `chtMultiRegionFoam` scalar coupling | Momentum may be absent or simplified depending on region and setup | Good for velocity-dependent scalar tests and thermal coupling cases |

For direct references, the solver pages describe required fields for each solver and the expected physical model blocks (for example `icoFoam` uses no turbulence model dependency while `pisoFoam` typically does) [icoFoam](https://doc.openfoam.com/2306/tools/processing/solvers/rtm/incompressible/icoFoam/) and [pisoFoam](https://doc.openfoam.com/2306/tools/processing/solvers/rtm/incompressible/pisoFoam/).

### Velocity summary

- Most velocity work starts by deciding pressure coupling; velocity accuracy usually improves only after that coupling choice is correct.
- `icoFoam` remains the stripped laminar baseline for educational/transient incompressible flow.
- `potentialFoam` is a pre-processing velocity tool rather than a final production solver.

---

## ⚖️ Density solvers

Density behavior changes the coupling strategy more than many people expect. OpenFOAM documents both pressure-based (transonic-aware but still pressure-coupled) and density-based approaches.

### Density strategy comparison

| Density strategy | Representative solvers | Coupling style | Typical regime |
|---|---|---|---|
| Constant density incompressible | `simpleFoam`, `pisoFoam`, `pimpleFoam` | Kinematic pressure formulation (`p`) and continuity constraints | Liquids/gases where density variation is small |
| Compressible pressure-based (`pressure-based variable-density`) | `rhoSimpleFoam`, `rhoPimpleFoam`, `buoyantSimpleFoam`, `buoyantPimpleFoam` | SIMPLE/PIMPLE-like pressure correction with thermophysical updates | Low-to-moderate Mach and heat-transfer driven density change |
| Pressure with gravity correction (`p_rgh`) | `interFoam`, `buoyant*` variants, some multiphase/CHT variants | Pressure shifted by hydrostatic contribution | Buoyancy / hydrostatic flows, thermal natural convection |
| Density-based compressible | `rhoCentralFoam`, `sonicFoam`, `sonicDyMFoam` | Conservative compressible form via central-upwind / shock-capable fluxes | Transonic/supersonic or high-Mach compressibility-dominated flow |
| Multiphase compressible density transport | `compressibleInterFoam`, `compressibleInterDyMFoam`, `compressibleMultiphaseInterFoam`, `compressibleInterIsoFoam` | Solves phase-wise density and energy transport | Two-fluid phase change and non-isothermal immiscible flows |

OpenFOAM’s standard solver index explicitly places `rhoSimpleFoam`, `rhoPimpleFoam`, and `rhoCentralFoam` in compressible categories and explicitly marks `compressibleInterFoam` as a compressible, non-isothermal multiphase VoF solver [A.1 Standard solvers](https://www.openfoam.com/documentation/user-guide/a-reference/a.1-standard-solvers).

### Density summary

- If density is fixed or weakly varying, pressure-based incompressible families are simpler and often faster.
- If density is strongly coupled to pressure, temperature, or phase fraction, use pressure-based compressible or density-based methods.
- When Mach/shock physics matter, prefer density-based formulations (`rhoCentralFoam`, `sonic*`).

---

## ♨️ Energy, heat, species, and reactive flow

A lot of production CFD is now “velocity + pressure + energy” plus source models. That is why this section is grouped separate from pure pressure and pure velocity classes.

### Transport complexity comparison

| Transport objective | Representative solver(s) | Equation set growth |
|---|---|---|
| Isothermal flow only | `simpleFoam`, `pisoFoam`, `interFoam` | Base set (`U`, `p`) |
| Temperature included | `buoyantSimpleFoam`, `buoyantPimpleFoam`, `reactingFoam` | Adds energy transport and thermophysical states |
| Multiregion conduction/convection | `chtMultiRegionFoam` | Adds region-wise energy coupling and interface heat flux terms |
| Reactive species + combustion | `reactingFoam`, `fireFoam`, `reactingParcelFoam`-family options | Adds species transport + reaction source stiffness |
| Radiation/extended heat sources | `reactingFoam` extensions / external functionals | Model-dependent additions (optical, source terms, particle coupling) |

OpenFOAM `reactingFoam` sits in combustion category and pairs with turbulence and thermophysical models (source page + user-guide references) [reactingFoam](https://doc.openfoam.com/2312/tools/processing/solvers/rtm/combustion/reactingFoam/).

### Energy summary

- Energy coupling usually increases runtime materially because transport + chemistry + stiff sources add nonlinear iteration loops.
- For thermal buoyancy, pressure shift (`p_rgh`) can be important for hydrostatic consistency.
- For conjugate heat transfer, region coupling in `chtMultiRegionFoam` typically dominates setup complexity.

---

## 🌀 Swirl and rotation ("swirl solver" area)

There is no dedicated general-purpose OpenFOAM solver named `swirlFoam` in the standard solver index. Swirl is usually represented by:

- A dedicated inlet BC such as `swirlFlowRateInletVelocity`.
- Rotating meshes, `MRF` frames, or moving reference formulations.
- The same pressure/velocity solver family (often `pimpleFoam`) as the surrounding physics.

### Swirl implementation comparison

| Swirl tool | What it does | Typical pairings |
|---|---|---|
| `swirlFlowRateInletVelocity` BC | Adds normal flow rate + tangential angular component from RPM specification | Use with `simpleFoam`, `pimpleFoam`, `rhoPimpleFoam` where an inlet swirl profile is needed | 
| Turbulent swirl inlet BCs (`turbulentDFSEMInlet`, etc.) | Adds eddies and turbulent content to inlet velocity | Transitional LES/RANS start-up or benchmark matching |
| Rotating mesh or MRF setup | Rotating cell zones with non-inertial terms | Fan/pump/turbomachinery and transient fan wake studies |

`swirlFlowRateInletVelocity` is documented as a dedicated derived inlet condition with swirl component and flow-rate specification [swirl BC docs](https://doc.openfoam.com/2312/tools/processing/boundary-conditions/rtm/derived/inlet/swirlFlowRateInletVelocity/) and appears in the standard boundary-condition reference [A.4 standard boundary conditions](https://www.openfoam.com/documentation/user-guide/a-reference/a.4-standard-boundary-conditions).

---

## 🧱 Solver-family comparison matrix by variable type

### Pressure-centric
- `SIMPLE`: `simpleFoam`, `rhoSimpleFoam`, `buoyantSimpleFoam`
- `PISO`: `pisoFoam`, `icoFoam`, `interFoam`
- `PIMPLE`: `pimpleFoam`, `rhoPimpleFoam`, `buoyantPimpleFoam`

### Velocity-centric
- `icoFoam` (laminar transient baseline), `pisoFoam` / `pimpleFoam` (standard transient), `pimpleFoam` for mixed LES/RANS transients

### Density-centric
- `rhoSimpleFoam` (pressure-based variable density), `rhoPimpleFoam` (transient), `rhoCentralFoam` (density-based), `compressibleInterFoam` (multiphase density/energy transport)

### Other important categories
- `interFoam` family (interface-capturing multiphase)
- `chtMultiRegionFoam` (conjugate heat transfer)
- `reactingFoam` (+ combustion species)
- `Lattice Boltzmann Method` where meso-scale method replaces continuum pressure-coupling strategy

---

## 🔗 Hub-to-spoke backlog

This note is a hub; spokes to write next in `Notes to Write Soon.md` or as standalone notes:

- [[SimpleFoam]] and [[OpenFOAM]]
- [[pisoFoam]] and [[PISO algorithm]]
- [[pimpleFoam]]
- [[simpleFoam]]
- [[rhoSimpleFoam]] and [[rhoPimpleFoam]]
- [[rhoCentralFoam]] and [[sonicFoam]]
- [[interFoam]] and [[compressibleInterFoam]]
- [[buoyantSimpleFoam]] and [[buoyantPimpleFoam]]
- [[chtMultiRegionFoam]]
- [[reactingFoam]] and [[fireFoam]]
- swirlFlowRateInletVelocity
- [[OpenFOAM]], [[SU2]] and solver taxonomy notes

---

## 📚 External resources

- OpenFOAM Documentation: `Pressure-velocity` algorithms and solver pages (SIMPLE/PISO/PIMPLE context) — <https://doc.openfoam.com/2312/tools/processing/solvers/pressure-velocity/>
- OpenFOAM Documentation: `simpleFoam` — <https://doc.openfoam.com/2306/tools/processing/solvers/rtm/incompressible/simpleFoam/>
- OpenFOAM Documentation: `pisoFoam` — <https://doc.openfoam.com/2306/tools/processing/solvers/rtm/incompressible/pisoFoam/>
- OpenFOAM Documentation: `pimpleFoam` — <https://doc.openfoam.com/2312/tools/processing/solvers/rtm/incompressible/pimpleFoam/>
- OpenFOAM Documentation: `rhoSimpleFoam` and `rhoPimpleFoam` — <https://doc.openfoam.com/2312/tools/processing/solvers/rtm/compressible/rhoSimpleFoam/>, <https://doc.openfoam.com/2312/tools/processing/solvers/rtm/compressible/rhoPimpleFoam/>
- OpenFOAM Documentation: `rhoCentralFoam` — <https://doc.openfoam.com/2312/tools/processing/solvers/rtm/compressible/rhoCentralFoam/>
- OpenFOAM Documentation: `interFoam` — <https://doc.openfoam.com/2312/tools/processing/solvers/rtm/multiphase/interFoam/>
- OpenFOAM User Guide index: standard solver list — <https://www.openfoam.com/documentation/user-guide/a-reference/a.1-standard-solvers>
- OpenFOAM boundary condition reference for swirl — <https://doc.openfoam.com/2312/tools/processing/boundary-conditions/rtm/derived/inlet/swirlFlowRateInletVelocity/>
- SU2 docs: physical definitions and solver setup — <https://su2code.github.io/docs_v7/Physical-Definition/>, <https://su2code.github.io/docs_v7/Solver-Setup/>
- SU2 governing equations and discretization strategy — <https://su2code.github.io/docs_v7/Theory/>
