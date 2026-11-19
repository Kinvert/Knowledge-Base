# OpenFOAM

OpenFOAM is an open-source CFD stack, not a single solver. It is a library+case framework that provides dozens of applications, utilities, and solvers through dictionaries and command line workflows. It covers meshing, solve, and post-processing in the same case-based system, which makes it a strong fit for scripted optimization loops and repeatable CFD pipelines.

---

## What it is (and what it is not)

OpenFOAM is used for more than “CFD only”:

- It includes pre-processing utilities (e.g. `blockMesh`, `snappyHexMesh`, converters).
- It includes solver applications (e.g. incompressible, compressible, multiphase, combustion, heat transfer, particle/lagrangian).
- It includes post-processing hooks (`reconstructPar`, `paraFoam`, data converters like `foamToVTK`).

What it does not provide:

- A single GUI-driven turnkey flow from CAD import to post-processed report like managed CAD-CFD platforms (e.g. Ansys Workbench/Fluent/CFX, Siemens STAR-CCM+, COMSOL, SimScale, or Autodesk CFD).
- A single “all-in-one” solver for every flow class (you choose solver by physics class).

The OpenFOAM user guide explicitly states there is no generic solver for all cases and directs users to class-specific solvers by category (incompressible, compressible, multiphase, turbulence class, etc.).

## High-value solver families in OpenFOAM

For an agentic CFD loop, the practical families are usually:

- [[simpleFoam]] + [[pisoFoam]] + [[pimpleFoam]] (incompressible baseline family)
- [[rhoSimpleFoam]] + [[rhoPimpleFoam]] + [[rhoCentralFoam]] (compressible pressure/density based family)
- [[interFoam]] + [[compressibleInterFoam]] (two-phase volume-of-fluid family)
- [[reactingFoam]] + [[fireFoam]] (combustion and heat-release family)
- [[buoyantSimpleFoam]] + [[buoyantPimpleFoam]] (buoyancy + convection family)
- [[chtMultiRegionFoam]] (conjugate heat transfer, multi-region)
- [[icoFoam]] (minimal laminar transient family for benchmarking)

This is the map that usually matters for loop design, not the internal solver executable list:

| Family | Core coupling | Main variable class | Typical physics | Good first choice when |
|---|---|---|---|---|
| Incompressible steady/transient | SIMPLE / PISO / PIMPLE | `p, U` (+ turbulence, no energy unless configured) | Low-Mach single-phase flow, wake and drag studies | You sweep geometry and want quick, robust turnover |
| Compressible pressure-based | SIMPLE / PIMPLE + thermophysics | `p, U, T` (+ species if enabled) | Low-to-moderate Mach, heated ducts, density effects | Density changes matter, but full shock capture is not first priority |
| Compressible density-based | Central-upwind Kurganov-Tadmor | `p, U, T` + compressible transport | High-speed features, wave/shock transport | Local Mach effects dominate and steady assumptions fail |
| Incompressible VOF | PIMPLE | `p_rgh, U, alpha` (+ turbulence, surface tension) | Air/water sloshing, wetting, entrainment | Phase interface is a KPI, free-surface motion is important |
| Compressible VOF | PIMPLE / thermodynamic coupling | `alpha` + phase-specific thermodynamics | Flashing, flashing jets, gas/liquid compressibility interaction | Interface and compressible energy coupling are both needed |
| Combustion/Flame | PIMPLE + reacting thermochemistry | `p, U, T` + species, chemistry source terms | Flame anchoring, heat release, ignition/evaporation workflows | Heat release and species kinetics materially change flow structure |
| Buoyant thermal | SIMPLE / PIMPLE + `p_rgh` | `p, U, T` + buoyancy forcing | Thermal plumes, stratification, heat source studies | Buoyancy drives pressure and recirculation, but multi-region heat transfer is not needed |
| Conjugate regions | Multi-region PIMPLE with couplers | `T` in solids, `p, U, T` in fluids | Thermal lag/inertia across metal/solid walls | You need wall conduction and thermal capacitance in same solve |

### Solver-availability cheat-sheet for each family

| Use case | Recommended OpenFOAM family | Why this one | Typical alternatives in this set |
|---|---|---|---|
| Geometry-only drag coefficient trend in steady separated flow | [[simpleFoam]] | Fast and stable for parametric studies | [[pisoFoam]], [[pimpleFoam]], [[rhoPimpleFoam]] if unsteady/thermal effects appear |
| Unsteady wake growth, diffuser transient effects | [[pimpleFoam]] | Stable hybrid coupling for moderate transients | [[pisoFoam]] for smaller budgets; [[rhoPimpleFoam]] when density coupling matters |
| Low-speed thermal diffusers with temperature-driven density shifts | [[rhoSimpleFoam]] / [[rhoPimpleFoam]] | Adds `T`/density coupling without abandoning OpenFOAM dictionaries | [[buoyantSimpleFoam]] if buoyancy is main driver; [[chtMultiRegionFoam]] if solids matter |
| Free-surface entrainment and interface deformation | [[interFoam]] | VOF path with `p_rgh` and `alpha` control | [[compressibleInterFoam]] for compressible phase energy coupling |
| Reactive flame / high heat release cases | [[reactingFoam]] | Canonical combusting OpenFOAM family | [[fireFoam]] when walls/films/pyrolysis are integral |
| Buoyant thermal recirculation where heat source and transient flow interact | [[buoyantPimpleFoam]] | Unsteady thermal-buoyant coupling | [[buoyantSimpleFoam]] for steady screening |
| Wall conduction and thermal inertia affect flow outcome | [[chtMultiRegionFoam]] | Native fluid-solid coupling | External one-way or loose coupling in external workflow tools |
| Laminar code-path validation benchmark | [[icoFoam]] | Minimal model overhead for sanity checks | [[simpleFoam]] / [[pisoFoam]] when adding turbulence |

---

## Core mechanics: how OpenFOAM cases are organized

OpenFOAM uses plain-text dictionary-driven cases with three key directories:

- `constant/` (mesh + material/model data in `polyMesh`, `transportProperties`, etc.)
- `system/` (run control, numerics, and meshing/solver dictionaries)
- time directories (usually `0/`) containing field initial/boundary values and result write directories.

This structure is the reason OpenFOAM is script-friendly: most control is editable text and can be versioned, templated, or generated automatically.

The OpenFOAM command-line docs also describe the toolchain as primarily CLI-driven and explicitly suitable for flexible automation and batch processing.

---

## What it can and cannot do in practice

OpenFOAM is strongest when you treat it as a pipeline you own end-to-end. A realistic decision rule is:

- If you can define geometry/method assumptions as data and checkpoints, OpenFOAM is excellent.
- If you need a GUI wizard that prevents bad setup decisions up-front, you often spend more time with OpenFOAM.

### OpenFOAM engine profile

- Numerical method: finite-volume discretization across control volumes.
- Primary design pattern: dictionary-based configuration (`fvSchemes`, `fvSolution`, `controlDict`, dictionaries in `system/` and `constant/`).
- Primary pressure-velocity couplers:
  - `SIMPLE` for steady-state classes,
  - `PISO` for transient pressure-based classes,
  - `PIMPLE` as hybrid for larger implicit transient loops.
- This is often confused with a "turbulence class solver"; in OpenFOAM terminology, `turbulenceModels` are model plugins (RANS/LES/DES choices in `turbulenceProperties`), while the solver executable defines the governing flow class and coupling pattern.
- Standard scalar/vector discretization controls, runtime settings, and linear solver choices are controlled from dictionaries; this is where reproducibility and automation depth come from.
- Dynamic mesh and mesh-motion support exists through dedicated utilities/solvers, but setup quality is heavily model- and case-specific and much less "black-box-safe".

### What can OpenFOAM solve, and where it stops (algorithm scope)

OpenFOAM covers most mainstream industrial-class workflows, but it is not a single method.

- Discretization: finite-volume in all major transport families; high-order options depend on chosen schemes and mesh quality.
- Temporal integration: explicit and implicit stepping, with algorithmic control via solver dictionary settings.
- Turbulence: standard RANS and selected LES frameworks are configurable through model dictionaries.
- Multiphysics coupling: conjugate heat transfer, particle/lagrangian add-ons, scalar transport and reacting flow extensions are available in solver ecosystems.
- High-order / low-Mach compressible: available through specific solver configurations, not all solvers.

Gaps that matter in loops:

- Rarely offers true out-of-the-box multiphysics “wizards”; you own boundary-condition, region, and model compatibility consistency.
- No universal “one click” for:
  - fully implicit shock-sensitive design optimization with broad automatic case adaptation,
  - guaranteed CAD topological healing for arbitrary imported geometry,
  - automatic turbulence-model selection by geometry type.
- Performance tuning on GPU is not a core, first-class path in the base OpenFOAM toolchain you get from community docs.

### What physical models are represented in the solver catalog

Use the official solver index pages as the truth source for exact solver names, then map each to your physics assumptions:

1. Incompressible flows:
   - steady/transient incompressible RANS and laminar options (for example `simpleFoam`, `pimpleFoam`, `pisoFoam`).
2. Compressible flows:
   - compressible solvers with explicit thermophysical controls (`rhoPimpleFoam`, `rhoSimpleFoam`, `sonicFoam` families).
3. Turbulence models:
   - RANS and selected LES classes exposed through solver/model dictionaries in turbulence model lists and solver examples.
4. Multiphase / VOF / free-surface:
   - dedicated solvers such as `interFoam` and related variants.
5. Combustion / reacting flow:
   - dedicated combustion workflows like `fireFoam`, `reactingFoam` with chemistry and temperature-coupled source terms.
6. Heat transfer:
   - conduction-convection-energy transfer families, conjugate setups with region-based case structures in specialized solvers.
7. Particles / multiphase coupling:
   - Lagrangian and particle-tracking features available in the modular solver/tool ecosystem.
8. Mesh-to-solution utilities:
   - `checkMesh`, `renumberMesh`, conversion utilities, and field tools that are essential for closed-loop automation.

### What that means for mesh types

If you keep this practical:

- `blockMesh` is best for structured, block-structured regions where you can control grading and grading transitions explicitly.
- `snappyHexMesh` is strongest on clean triangulated surface geometry, with staged `castellated` → `snapping` → `addLayers` behavior.
- External mesh conversion is necessary for CAD-first flows that prefer meshed STL/other formats before fluid preprocessing.

### Mesh format/geometry acceptance vs failure modes

OpenFOAM accepts triangulated surface inputs widely, but topology hygiene is the bottleneck:

- Tolerates high-volume automation, but not garbage-in geometry.
- Fragile corners are usually:
  - non-manifold edges,
  - zero-area triangles,
  - inconsistent patch naming,
  - disconnected shells/very thin sliver geometry.
- For robust sweeps, add geometry validation gates before `snappyHexMesh`.

### What it can do in a campaign loop that many teams miss

- Deterministic reruns from git-committed template cases.
- Parameterized meshing + solve + post chains with scriptable checkpoints.
- Easy comparison of pre/post states because case directories are plain assets:
  - same geometry generator but different dict versions.
- Fine-grain KPI extraction with `postProcess -func` and functionObject output.

### What it cannot do as easily as people expect

- Turnkey CAD feature-tree editing in one step (OpenFOAM is not a CAD system).
- Guaranteed mesh quality from complex trim surfaces without iterative cleanup.
- Out-of-the-box vendor-level workflow support for every special physics branch.
- GUI-level safeguards around physics model compatibility; you must enforce validation yourself.

### Solver depth by user intent

If your loop is mostly:
- optimization over a few geometric variables (diffuser angle, inlet profile),
- repeatable templates,
- and scripted KPI gates,

OpenFOAM is in its comfort zone.

If your loop is:
- design review where you expect GUI guidance for every boundary/model combination,
- strict one-click audit trails across multidisciplinary teams without pipeline ownership,
- or heavily managed enterprise support requirements,

a commercial suite may move you faster initially, though with higher cost and fewer direct automation freedoms.

### Meshing

OpenFOAM’s meshing stack is broad and integrated:

- `blockMesh` creates hexahedral structured blocks using vertex/edge/block definitions; it supports grading (`simpleGrading` / `edgeGrading`) and explicit patch naming/patching in `blockMeshDict`.  
- `snappyHexMesh` generates hex/split-hex meshes from triangulated surfaces (`STL`) and typically proceeds through castellation, snapping, and layer stages. It reads triangulated geometry and supports `-parallel`, `-overwrite`, and feature extraction workflows.  
- `decomposePar` and related utilities support mesh/field distribution and reconstruction for parallel runs.  
- Mesh conversion utilities bring in external meshes (Fluent, STAR-CCM/STAR-CD lineage, Ansys, etc.) into OpenFOAM.

Important constraints in conversions:

- Fluent importer: multiple material meshes are not preserved; unsupported interfaces/trees are filtered or simplified; internal patches in source mesh are not retained as-is.  
- STAR-CCM / STAR-CD conversion: supports many mesh types but has explicit unsupported-feature constraints in its converter.  

OpenFOAM is good at generating production-ready meshes if your geometry is clean and you accept meshing tuning effort.

### Deep dive: can OpenFOAM meshing hit underfloor gaps + strakes + diffusers?

For ground-effect / bluff-body work, this is the hard part:

- very tight gaps (undertray clearance, diffuser side-wall gaps),
- small spanwise or streamwise features (strakes, fences, vortex generators),
- and first-cell control (`prismatic` growth, target `y+`, shear layer resolution) around those features.

OpenFOAM is not one thing here—it is one of four practical meshing paths depending on the geometry stress level.

- `blockMesh` gives exact topology and fully scripted control.
- `snappyHexMesh` gives surface-driven hex/split-hex automation and layered near-wall treatment.
- `cfMesh` gives a more geometry-robust OpenFOAM-native alternative with its own boundary layer pipeline.
- `Gmsh` / `Netgen` are more often best used as preprocessors when you need CAD-driven local size control before OpenFOAM.

#### The real comparison for underfloor + strakes + diffusers

For underfloor flows the quality bar is:

- Preserve the undertray channel and wheel-clearance topology (no unintended leaks/merges),
- keep fence/strake leading edges sharp,
- keep wall-normal spacing low enough for expected boundary-layer thickness,
- keep total cell quality in wake-relevant regions.

### Boundary-layer capability comparison (prismatic detail)

| Mesh stack | How prismatic/boundary layers are created | Layer control quality | Where it fails first |
|---|---|---|---|
| `[[snappyHexMesh]]` | Layer extrusion from wall faces during `addLayers` stage (`addLayersControls`) | Explicit controls (`nSurfaceLayers`, `firstLayerThickness`, `finalLayerThickness`, `expansionRatio`, `nGrow`, `featureAngle`, `maxFaceThicknessRatio`, `maxThicknessToMedialRatio`) and quality guards can drive strong y+-targeted workflows when geometry and base mesh are coherent | Tends to skip/reduce layers in thin gaps, around very sharp curvature transitions, or where feature extraction/topology is noisy |
| `[[blockMesh]]` | No automatic BL extrusion; you create wall-normal growth through block grading and manual near-wall block partitioning | Very stable and fully deterministic but not truly wall-adaptive unless every region is pre-encoded in block topology | Hard to represent curved fences/strakes and re-entrant detail without excessive manual geometry decomposition |
| `[[cfMesh]]` | Boundary layers in `boundaryLayers` dictionary (global and per-patch via `patchBoundaryLayers`) are extruded from boundary faces in the generated volume mesh; can use `optimiseLayer` and smoothing controls | Useful when thickness and smoothness need to track geometry complexity and patch continuity; controls like `thicknessRatio` and `nLayers` are explicit | Can saturate thickness to local cell-size constraints in complex edge transitions; thin, deep concavities still require iteration |
| `[[Gmsh]]` | Topological boundary layer extrusion via `extrudeBoundaryLayer` and/or boundary layer fields (`BoundaryLayer`, `setAsBoundaryLayer` in docs) | Strong where layer direction is controlled in CAD/scripting, especially when surface mesh is rebuilt from geometry before extrusion | Not always sufficient for complex automotive underfloor topology without a separate layer-quality pass |
| `[[Netgen]]` | Core docs are oriented around tetrahedral generation; there is no dedicated production boundary-layer workflow in the core API docs | Fast and robust for automatic tetra volume fill, but wall-layer specialization is limited in native workflow | Diffuser undertrays with strong anisotropy often need post-`convertToFoam` or external layering |
| HyperMesh (external preprocessor) | Dedicated surface + volume meshing pipeline with boundary layer mesher; layer growth, transition, and core interaction controlled during mesh generation in HyperMesh (`*blmesh_2d`, `*tetmesh`, CFD Tetramesh controls) | Very strong explicit local control for layer insertion, edge/corner handling, and transition strategy (`All Prism`, `All Tetra`, distributed BL thickness, quality filters) before export | Not OpenFOAM-native; must be maintained as a separate stage plus export/import (STEP/STL -> volume mesh -> OpenFOAM conversion) |
| `[[Ansys CFX]]` (reference class) | Commercial CAD-to-solver preprocessing generally includes dedicated inflation and wall-normal controls (external to this OpenFOAM stack) | Stronger integrated GUI-driven controls for layer assignment by face sets, growth, and continuity | Not script-first-friendly unless you own enterprise flow scripts/APIs |

### HyperMesh boundary-layer stack vs `snappyHexMesh`: why it is usually better for diffuser/undertray BL control

For this specific use case, HyperMesh is usually the better boundary-layer generator, because it gives you direct control points that OpenFOAM meshing currently exposes only indirectly.

- **More deterministic geometry targeting**
  - HyperMesh supports one model control + unlimited local controls, with local overrides and on/off toggles (`Mesh Controls`), and controls can be exported/reused as `.xml` templates.
  - In snappy, local wall control is mainly patch/region based and tied to the existing clipped background mesh plus STL regions; when patch edges drift due to snapping, BL settings react to what survived the surface snap step.
- **Better low-clearance behavior (diffusers, wheel wells, undertray throttles)**
  - OpenFOAM layer generation has explicit limits (`minThickness`, `maxThicknessToMedialRatio`, `maxFaceThicknessRatio`, `nGrow`, `nLayerIter`) and a back-off/retry cycle; it can stop inserting where constraints cannot be met.
  - HyperMesh exposes distributed thickness workflows (for example, distributed BL thickness ratio + minimum tetra-core / BL thickness ratio controls) that are explicitly designed to reduce layer size in tight opposing-wall regions before the tetra core is generated.
- **Strake/edge sharpness control**
  - HyperMesh gives explicit options for acceleration-based growth and corner control (`corner factor`, corner smoothing behavior in BL commands), while also allowing local remesh/fixed/float behavior on boundaries during BL imprinting.
  - In snappy, sharp-feature behavior depends heavily on `resolveFeatureAngle`, `featureAngle`, and refinement/surface extraction quality, and still operates inside a single add-layers iteration framework.
- **Cell-transition quality for mixed prismatic+tetrhedral topology**
  - HyperMesh gives explicit transition modes (`All Prism`, `All Tetra`) and explicit split behaviors (`All Prism` keeps prism layers, `All Tetra` transitions differently) and later provides split-to-tet utilities.
  - In OpenFOAM workflow, prisms are inserted directly as split hex layers and transition behavior is coupled to `addLayersControls` plus general mesh quality controls.
- **Why this matters in bluff-body ground-effect loops**
  - You need repeatable first-layer thickness and growth under high aspect-ratio near-ground flow, with thin boundary layer retention around fences/strakes and clean taper into the tetra core/wake region.
  - In practice, teams that are iterating undertray/fence geometry often get more stable convergence if BL generation is done in HyperMesh (or another mature CAD CFD preprocessor) and then fed to OpenFOAM.

### Practical mapping: `snappyHexMesh` knobs vs HyperMesh equivalents

| Requirement | OpenFOAM (`snappyHexMesh`) | HyperMesh |
|---|---|---|
| First wall height | `firstLayerThickness` | `First Layer Thickness` |
| Growth law | `expansionRatio` / `relativeSizes` in `addLayersControls` | First-layer + growth-rate options including constant/acceleration and aspect-ratio-based control |
| Sharp-feature protection | `featureAngle` + `maxThicknessToMedialRatio` + `maxFaceThicknessRatio` + `nLayerIter` | `cornerFactor`, corner handling parameters in BL generation, distributed BL thickness with corner-aware scaling |
| Local adaptation in tight gaps | Back-off behavior + quality thresholds; no dedicated “BL collision reduction field” in base workflow | Distributed BL thickness generation with minimum tetra-core/BL ratio constraints and local thickness reduction |
| Transition into core | Quality controls + implicit split decisions in add-layers and volume generation | Explicit hexa transition mode (`All Prism`, `All Tetra`) and explicit prism-to-tet split workflow |
| Reproducibility | Dict + command-line scripts; very strong for full case reproducibility | Mesh controls + command scripting (`Task Manager` and Tcl commands), but requires maintaining an additional preprocessor stage |

### Edge/feature and narrow-gap behavior

The decisive failure mode for cars is not global cell count; it is feature-scale under-resolution.

What matters on strakes/ground-effect geometry:

- `blockMesh`:
  - Pros: if strakes are parametrized analytically, edges stay exact.
  - Cons: any geometry edit (strake blend, sweep change, local clearance shift) becomes a topology rebuild task.
- `snappyHexMesh`:
  - Needs clean `triSurface`; noisy triangulation kills edge fidelity and layer continuity.
  - Narrow gaps are addressed via gap refinement mechanics (example family includes `gapLevelIncrement`/`gapLevel` controls in historical/future docs), but they still need base cell refinement and controlled patch assignment.
  - Feature edges must be preserved either directly in the source surface or via edge extraction for sharp-edge behavior.
- `cfMesh`:
  - In noisy CAD cases, it often survives where `snappyHexMesh` fails in early snapping.
  - Multi-patch layer continuity is available, but strong anisotropic requirements in small gaps can still destabilize layer quality.
- `Gmsh`:
  - Best with scripted local size fields; strakes and fences become explicit geometric entities first, then meshed with explicit field control.
  - Good strategy for parametric DOE pipelines where CAD is generated from code.
- `Netgen`:
  - Good cleanup+tetra fallback when STL conversion paths are poor.
  - Usually not the first choice for strict prismatic fidelity on underfloor strakes.

### Practical workflow map for this specific use case

Use this by geometry complexity:

1. **Geometry clean and regular**
   - `[[blockMesh]]` base (or scripted pre-blocking),
   - `[[snappyHexMesh]]` only for local adaptation near curved strake edges and underbody fillets.
2. **Moderately messy CAD or repeated tri-surface defects**
   - Preprocess CAD in `[[FreeCAD CFD Workflow]]`/`[[CadQuery CFD Workflow]]` style scripts to rebuild watertight topology,
   - generate controlled external mesh prep in `[[Gmsh]]` or `[[Salome]]` and import to OpenFOAM.
3. **Frequent STL defects / highly concave underbody details**
   - use `[[cfMesh]]` as fallback mesh generator,
   - keep `snappyHexMesh` for only final cases where edge extraction can be stabilized.

### What this means for gap-fitting around ground hardware

In a diffuser/undertray stack, a single `firstLayerThickness` and `expansionRatio` is never enough. You need coupling between:

- background refinement (`blockMesh` + local refinement regions),
- surface feature retention (`surfaceFeatureExtract` + `resolveFeatureAngle` / feature files),
- layer policy (`addLayersControls` and `maxThicknessToMedialRatio`),
- and geometry gates (check minimum gap and patch continuity before solve).

The practical test pattern is:

- if geometry has a gap, ensure `nSurfaceLayers` coverage does not truncate to near-zero on both walls,
- if strake edges show staircase behavior, increase edge density before raising layer count,
- if gap wake shows swapped cell orientation or broken transitions, reduce growth/expansion and simplify edge angles first.

### Moving-ground and ground-effect patterns for race-car studies

Yes, OpenFOAM can do moving-ground setups, but it is not automatic:

1. Define a moving ground patch on the floor and set a consistent `movingWallVelocity` direction convention.
2. For wheel effects, choose:
   - transient rotational model (`MRF`, AMI/AMI-like patch interfaces, or full moving-mesh path),
   - or a simplified proxy if wheel wake details are not a KPI.
3. Keep solver family and CFL settings consistent after switching from fixed ground to moving-ground.
4. Re-run mesh quality gates; thin gap layers are often the first thing to break when ground speed is active.

Pragmatic rollout:

- run fixed-ground candidates first for the baseline design-space screen,
- use moving-ground only on a filtered top set,
- enforce one ground-movement profile per campaign version (`U_vehicle`, `mu_wall`, turbulence constants, slip settings).

Common failure patterns:

- wrong sign convention in moving-wall velocity (flow direction reversed),
- interface mismatch where wheel patches/clearance are represented too coarsely,
- excessive near-wall stretching causing layer collapse at high relative speed,
- timestep and mesh Courant constraints tightening due to wall motion and transient coupling.

### Why snappy can still beat manual approaches in this loop

Even with its fragility, `snappyHexMesh` is still the strongest OpenFOAM-native path for "cleaner CAD + repeatable campaign" because it is:

- deterministic (dict-driven),
- MPI-capable in meshing stages,
- and able to combine CAD edge-sensitive snapping + split-layer insertion in a single run (`castellatedMesh`, `snap`, `addLayers`).

If your input STL is stable and your gap/feature taxonomy is consistent, this can outperform fully manual block rebuild loops once parameter ranges are bounded.

### Why snappy layers fail in the tightest features

OpenFOAM’s documented layer insertion logic is strict: it projects layers, relaxes, validates, and either accepts or backs off/retries until constraints are met; if constraints cannot be met for a wall face, it may not insert layers there.

The practical knobs (`maxThicknessToMedialRatio`, `featureAngle`, `maxFaceThicknessRatio`, `nGrow`, `nLayerIter`) are exactly where underfloor and sharp-edge cases are won or lost.

### Conversion reality check for OpenFOAM in this loop

- `fluentMeshToFoam` conversion drops some topology semantics (`multiple material` collapse, unsupported interfaces/trees, some internal patches).
- `star4ToFoam`/`cfx4ToFoam` style converters are useful but have explicit unsupported features like some baffles, partial boundaries, and sliding interfaces depending on source format.
- OpenFOAM enforces mesh validity before run-time; converted meshes that are valid enough in other tools can still fail on import.

If these are repeatedly failing, don’t keep pushing the same mesher blindly. Treat this as a stage-level choice in your loop:

1. Keep CAD parametrization clean and feature-sensible.
2. Generate geometry-adapted base size/edge growth externally (`[[Gmsh]]`/`[[Netgen]]` or equivalent).
3. Use `[[snappyHexMesh]]` or `[[cfMesh]]` only where geometry complexity requires.
4. Reject candidates via automated quality gates before solve.

### Solver coverage and numerical methods

OpenFOAM covers a broad solver portfolio across categories like:

- basic/incompressible
- compressible
- heat transfer
- multiphase
- particles/lagrangian
- combustion
- DNS
- electromechanics and stress extensions (in supporting docs trees)

The solver application index in OpenFOAM documentation shows these capability families and also explicit pressure-velocity coupling algorithms (`SIMPLE`, `PISO`, `PIMPLE`) for incompressible/transient flow families.

Examples from official solver docs:

- `simpleFoam` is steady-state incompressible, SIMPLE-based.
- `interFoam` is multiphase/transient incompressible with two-phase VOF-style topology.
- `fireFoam` / `reactingFoam` are combustion classes with chemistry, turbulence, and often PIMPLE coupling in documented flow setups.

`A.2 Standard utilities` enumerates utilities for partitioning, conversion, function objects, and post-processing entry points, reinforcing that solvers are one piece of a larger application ecosystem.

### Post-processing

OpenFOAM provides `paraFoam`, which launches ParaView. The docs note that ParaView with native OpenFOAM reader (`paraFoam -vtk`) can read decomposed cases for visual post-processing.

There is also data converter tooling (`foamToVTK`, `foamToEnsight`, etc.) in the utilities list for automation-friendly extraction steps.

### Compute model (parallel, MPI, and performance model)

OpenFOAM documents domain decomposition as its core parallel model; `decomposePar` creates per-processor decomposed cases and `reconstructPar` merges results for inspection/export.  

Parallel launch pattern in docs is MPI-based (`mpirun ... -np ... <foamExec> -parallel`).  

Main takeaway:

- Strong distributed CPU parallelism via MPI.
- Parallel execution is explicit and scriptable.
- No analogous native “GPU solver mode” API is shown in these core OpenFOAM distribution docs (contrast this with the explicit GPU sections in commercial Fluent docs below).

### Platform and releases

OpenFOAM Foundation release 14 notes include major features in modular solvers, meshing, modular Lagrangian, and utilities.  

Platform coverage in release/download materials includes Linux plus compatibility wrappers for Windows via WSL and macOS via Canonical Multipass + Ubuntu for local Linux-hosted usage.

---

## What OpenFOAM is weaker at (or explicitly harder)

OpenFOAM’s power is also its pain:

- Geometry-first workflows are external. You generally drive CAD-to-surface conversion before `snappyHexMesh`/`cfMesh` style meshing.
- Meshing and boundary naming correctness are highly sensitive to preprocessing quality. “Surface hygiene” issues (non-manifold/open edges) can derail snapping/layering.
- Converter paths from other systems are good but not lossless (for example embedded interfaces or complex multi-material assumptions can be downgraded depending on source).
- The stack is flexible but not “touch-to-run” for non-CFD teams; it needs strong convention for numerics, turbulence, and run controls.
- Debugging stack errors often requires understanding dictionary and case-level mechanics.

This is why teams usually split work into an explicit preprocessor chain before OpenFOAM:

- CAD/parameterization stage: build or mutate geometry in external modelers with parameter files (for example, `[[FreeCAD CFD Workflow]]`, `[[CadQuery CFD Workflow]]`, `[[Onshape CFD Workflow]]`, `[[SALOME CFD Workflow]]`, `[[OpenVSP CFD Workflow]]`, `[[OpenSCAD CFD Workflow]]`).
- Geometry repair/export stage: clean topology, stitch/repair watertight surfaces, create named regions, then export to neutral formats (`STEP`/`IGES`/`BREP` or STL/OBJ as needed). Common tools include `[[Blender CFD Workflow]]`, custom scripts, or dedicated model-healing in CAD tooling.
- Mesh stage:
  - Structured/controlled meshing: `[[blockMesh]]` or external structured tools feeding `[[blockMesh]]` templates.
  - Surface-hex/hex mesh generation: `[[snappyHexMesh]]` (OpenFOAM-internal).
  - Mesh conversion routes: `[[cfMesh]]` with surface/mesh formats.
  - External mesher routes: `[[Gmsh]]`, `[[Netgen]]`, then `convertToFoam`/`ideasUnvToFoam`/`fluentMeshToFoam`.
- Boundary/field mapping stage: create patch/field dictionaries (`0/`, `constant/`) and apply consistent naming conventions so case templates stay stable across parameter sweeps.
- Run stage: solve in `system/controlDict` + `system/fv*` dictionaries, then post-process with `[[paraFoam]]`/ParaView.

In short, OpenFOAM is the solver core plus automation layer; most teams keep CAD modeling, topology cleanup, and high-quality meshing in a dedicated preprocessor toolchain.

---

## OpenFOAM vs commercial Ansys solvers in depth (CFD loop mindset)

### Ansys Fluent (commercial)

Ansys Fluent command-line usage supports headless run modes, process counts, parallel, scheduler integration, and explicit GPU options such as `-gpu`, `-gpu_async`, and `-gpu_remap`.  

Fluent’s GPU documentation details what is supported in native GPU mode:

- mesh topologies (polyhedral/hexahedral/tet/pyramid/prism, hanging nodes),
- supported turbulence (e.g., laminar, several RANS variants, limited LES options),
- CHT support with caveats,
- solver settings (pressure-based, transient/steady modes),
- sequential parametric updates and boundary conditions support.

It also documents explicit unsupported items, including DO radiation edge cases, Workbench unavailability for GPU mode, and restart/monitor caveats.

### Ansys CFX (commercial)

Ansys CFX is documented as a general-purpose suite with a coupled solver focus, integrated pre/post process chain, and explicit model coverage for:

- steady/transient
- laminar/turbulent
- sub/trans/supersonic
- heat transfer/radiation
- buoyancy
- non-Newtonian
- multiphase
- combustion
- particle tracking
- multiple rotating/multiple-reference frames

Those points are from the official `Ansys CFX` documentation chapter overview.  

The docs also show CLI execution through `cfx5solve` and related components and direct parallel setup options (`-parallel`, `-partition`, host distribution flags).

CFX parallel docs in `v251/v252` describe the leader/follower SPMD execution pattern and MPI-based distributed parallel runs.

### OpenFOAM vs Ansys comparison (what matters for your loop)

| Dimension | OpenFOAM | Ansys Fluent | Ansys CFX |
|---|---|---|---|
| Workflow model | CLI-first case dictionaries + manual meshing + utility chain | CLI + GUI + integrated pre/post workflows | GUI + CLI (`cfx5pre`, `cfx5solve`) + integrated pre/post workflow |
| Meshing path | Internal meshing suite (`blockMesh`, `snappyHexMesh`) and converters | Internal mesh + import adapters + mature GUI meshing path | Internal mesher + converters + geometry/solver-integrated pipeline |
| Solver strategy | Many specialized solvers; user selects by class (`simpleFoam`, `pimpleFoam`, `interFoam`, combustion/multiphase families) | Commercial solvers and extensive model templates with stronger packaged defaults | Commercial solver family, coupled-flow emphasis, high-end production workflows |
| Parallel compute | MPI/domain decomposition (`decomposePar`, `reconstructPar`, openMPI `mpirun`) | MPI + scheduler options + native `-scheduler`, `-mpi`, etc. | MPI parallel with partitioning (`cfx5solve -parallel`, `-partition`/`-par-dist`) |
| GPU | Core docs focus on CPU/MPI model; no native GPU mode page shown in same docs family | Native GPU solver mode documented with explicit support and limitations list | No native GPU solve mode in the cited CFX references; source indicates documented parallel via MPI and command-line partitioning |
| Parametric/automation | Strong CLI + dict templating, scriptable case cloning and function-object reporting | Strong scripting and workflow automation via CLI + journaling + API channels | Strong for enterprise workflows via command line + solver manager and Workbench integration |
| Cost/perf tradeoff | Zero license, more engineering/setup burden, strong custom control | High license, strong vendor support, broad out-of-box templates | High license, strong robustness in industrial contexts, integrated ecosystem |

The practical split is usually:
- **OpenFOAM** for control-heavy custom loops (especially when you own the campaign orchestration).
- **Fluent/CFX** when you want robust prepackaged workflows and lower setup friction with vendor support.

---

## Can this be used for your bluff-body diffuser-angle optimization loop?

Yes. A robust OpenFOAM loop typically looks like:

1. `design_space.json` stores parameters (`diffuser_angle`, Reynolds target, turbulence model, turbulence intensity, wall roughness, etc.).
2. A generator script renders:
   - geometry parameters into a parametric CAD/mesh source (often external CAD API or surface script),
   - `system/blockMeshDict` and/or `system/snappyHexMeshDict`,
   - `system/controlDict`, `fvSchemes`, `fvSolution`.
3. Mesh stage:
   - `blockMesh` (base mesh)  
   - `surfaceFeatureExtract` (if needed)  
   - `snappyHexMesh [-parallel]` (castellated + snap + layers)
4. Mesh quality pass:
   - check constraints in `checkMesh`; reject bad candidates.
5. Solve stage:
   - decompose if needed (`decomposePar`)
   - run MPI solver in parallel (e.g., `mpirun ... <solver> -parallel`)
6. Post stage:
   - `reconstructPar` if run in parallel,
   - export fields (`foamToVTK` or direct ParaView pipeline),
   - collect KPI metrics from logs/functionObjects.
7. Update `design_space.json` and iterate.

This exact pattern is why OpenFOAM is a good base for agent-driven loops: the case is explicit data, and every stage is command-addressable.

---

## Useful automation pattern (OpenFOAM command surface)

```bash
# Example campaign loop structure (not a full script, skeleton only)
for params in $(cat design_space.json | jq -r '.cases[] | @base64'); do
  case_dir="run/$(echo $params | jq -r '.name')"
  mkdir -p "$case_dir"
  python3 scripts/render_case.py "$params" template_case/ "$case_dir"
  (cd "$case_dir" && blockMesh && snappyHexMesh -overwrite)
  (cd "$case_dir" && checkMesh || continue)
  (cd "$case_dir" && decomposePar && mpirun -np 16 interFoam -parallel > log && reconstructPar)
  (cd "$case_dir" && postProcess -func "forces")
done
```

Because OpenFOAM dictionaries are text and `case/system` + `case/constant` are deterministic, this loop is straightforward to make idempotent and reproducible.

---

## Comparison map (if you are choosing a CFD core)

| Tool | Best position | Where it wins | Biggest drawback |
|---|---|---|---|
| OpenFOAM | Open-source, highly customizable CFD pipelines | Custom workflows, script-first research and industrial-adjacent optimization | More setup friction, no single-turnkey industrial guardrails |
| Ansys Fluent | Broad commercial CFD production with strong model set | Native GPU option, strong parallel CLI options, mature workflow support | Commercial cost, explicit GPU mode limitations |
| Ansys CFX | High-confidence coupled and rotating-flow production | Robust coupled solver environment, integrated workflow stack | Commercial cost, less flexible than pure script-first ecosystem for deep custom automation |
| SU2 | Primarily aerodynamics/adjoint-driven optimization | Solver+optimization stack for shape optimization | Narrower multiphysics breadth than OpenFOAM |
| SimScale | Cloud-hosted CFD platform + collaboration | Fast start, low local-install burden | Cloud-run constraints and less deep source-level customizability |

---

## Boundary-layer engineering and speed roadmap for a bluff-body RL campaign

If your target is “generate design variants, run hundreds/thousands of cases, and converge toward a design objective,” the most effective sequence is:

1. Make meshing deterministic.
2. Make boundary-layer feasibility checks cheap.
3. Reduce full-fidelity solves with staged fidelity.
4. Then optimize compute hot spots (`solvers`, mesh quality checks, I/O) and only then tackle CUDA.

The first 3 steps usually give the biggest gains before raw GPU tuning.

### Why boundary layers are the bottleneck

For undertray/diffuser studies, wall resolution and gap integrity dominate convergence and design ranking. OpenFOAM’s `addLayers` implementation is a staged quality loop and will drop/reduce layer insertion when constraints fail (`nLayerIter`, thickness/medial-distance limits, layer iteration quality gates).  

That means there is a lot of room for upstream geometry-to-input discipline:

- if gap fields are under-resolved, layer insertion fails or throttles,
- if feature extraction is noisy, layers get dropped around sharp edges,
- if dicts are not version-locked, meshing changes between runs.

For automotive underbody geometry, this is often what hurts repeatability more than solver runtime.

### Deterministic workflow first (not optional)

OpenFOAM loops break when geometry edits are non-reproducible in mesh output. Build replayability into the control plane:

- deterministic CAD/geometry pipeline output (`STEP`+`STL` artifacts hashed),
- stable `snappyHexMeshDict` render with sorted lists (patch names, refinement regions, feature files),
- fixed decomposition and MPI layout,
- explicit mesh QA manifest (`checkMesh`, cell count, face skewness, wall patch coverage),
- fixed solver dictionaries + fixed wall time-step/control envelopes.

OpenFOAM can be made deterministic to a useful practical degree, but the first public GPU-offloading path is documented as a separate track and not the default determinism-first setting.
So for production RL sweeps, keep determinism mode as default and treat GPU changes as a second phase.

### Common preprocessor stage stack seen in production

The same stage graph shows up across campaign repositories:

1. **Parametric CAD stage**  
   Generate geometry from JSON/XLSX using a headless CAD API (`FreeCAD`, `CadQuery`, `Onshape`, `OpenSCAD`, `SALOME`, `Gmsh` scripting).
2. **Topology QA stage**  
   Mesh-heal/repair and patch semantic checks (`surfaceCheck`, custom manifold checks, volume/edge sanity, naming normalization).
3. **Mesher strategy stage**  
   Pick one of:
   - OpenFOAM-native (`blockMesh` + `snappyHexMesh`) when STL cleanliness is good.
   - OpenFOAM-native (`cfMesh`) when snap tolerance is poor.
   - External mesher (`Gmsh`/`Netgen` + `convertToFoam`) when feature fields are easier outside OpenFOAM.
   - External BL-focused preprocessor when diffusers/underbody require stronger layer governance.
4. **Mesh QA stage**  
   `checkMesh`, custom coverage checks (`nSurfaceLayers` retention, wall patch continuity), and failure-code extraction.
5. **Solve stage**  
   MPI launch and convergence control by solver family.
6. **Post/manifest stage**  
   deterministic KPI extraction to fixed schema (`forces`, probes, wake planes), plus metadata and commit-friendly manifest.

If you merge these stages into explicit commands and artifacts, agents become reliable at scale.

### PR idea track: where to get real stability gains quickly

#### Track 1: boundary-layer guardrails in OpenFOAM meshing

1. Add or standardize a pre-meshing gap/clearance scan (outside core or as utility extension).  
2. Compute minimum local clearance and feed per-surface `nSurfaceLayers`/`finalLayerThickness` caps.  
3. Skip or lower layer insertion where clearance is below a minimum ratio (instead of full case failure).  
4. Promote to full layer passes only for geometries that pass prechecks.

This path is aligned with how `snappyHexMesh` already behaves and is the lowest-risk way to reduce catastrophic meshing regressions across many candidates.

#### Track 2: deterministic stability in mesh candidates

1. Lock patch ordering and feature extraction lists before writing dicts.  
2. Add explicit quality assertions before full solve (`maxNonOrtho`, `maxBoundarySkewness`, layer count sanity checks).  
3. Persist mesh fingerprints so reruns can be compared automatically.

This is low compute risk and very high control-value for RL loops.

#### Track 3: controlled compute acceleration

This is where CUDA becomes realistic:

- Prefer the kernel areas where arithmetic density is high and data movement is regular (linear algebra / field transforms).  
- Don’t start with full meshing rewrite; start with targeted kernels and benchmark with fixed case sets.

OpenFOAM already has a public path indicating `std::execution`-based offloading, memory-location handling (UMPIRE), and `-stdpar`-style compilation direction, but this is documented as experimental/offloading work with CPU-threading parity caveats.

So for now, treat “rewrite in CUDA” as a staged project:

- Stage A: add optional GPU kernels for linear algebra-sensitive kernels.
- Stage B: measure speed + quality impact on representative candidate set.
- Stage C: evaluate if boundary-layer and orchestration changes dominate and only expand.

### Reality check on the 1000x target

One PR or one GPU rewrite does not usually deliver 1000x solver speed on its own. The practical route to ~1000x campaign throughput is:

- eliminate avoidable remesh/full-solve work via staged fidelity,
- cache geometry/mesh artifacts,
- use adaptive candidate pruning,
- and close the loop with a surrogate that requests only high-value full-order runs.

That stack (process + caching + selective full solves) is how teams usually get massive wall-clock gains in autonomous design optimization.

### Suggested upstream PR stack (highly practical for your loop)

If you want to turn this into code changes, here is the order that tends to get merged faster:

#### Track A: pre-layer geometry/quality gating (highest impact / low disruption)

- add a geometry gate utility before layer addition that computes:
  - local wall clearance,
  - local curvature/noise score,
  - minimum clear gap for boundary-layer insertion.
- make it fail-fast with explicit reason codes so the outer loop can either retry (lower target layers) or drop.
- keep full `snappyHexMesh` for cases that pass the gate.

This directly targets undertray/strake failure modes and avoids paying full meshing cost on broken candidates.

#### Track B: deterministic mesh provenance in core

- add deterministic output artifacts from `snappyHexMesh` (JSON summary per run):
  - per-patch `nSurfaceLayers` attempt/accepted,
  - failure reason map,
  - local quality stats (max skewness, non-orthogonality, invalid faces),
  - patch coverage metrics and cell counts.
- this gives your orchestrator stable post checks and supports campaign-level dedupe and ranking.

Implementation surfaces are in the `snappyHexMeshDriver` layer code path and `layerParameters` dictionaries.

#### Track C: staged performance work (not full CUDA rewrite)

- isolate one low-risk kernel class first (field-vector algebra or mesh-quality statistics),
- add optional offload toggle,
- keep strict parity checks on identical cases before widening scope,
- only then consider broader meshing kernels.

Do not do full CUDA for meshing+solver in one PR. In practice, this gets rejected or drifts in behavior.

### Can 1000x be real?

- one solver-level optimization: usually single-digit to low double-digit speedup in production solve time;
- real throughput gains usually come from reducing expensive cases and narrowing full-order evaluations.

Practical route to high aggregate gains:

1. deterministic candidate filtering before full mesh/solve,
2. staged fidelity/mesh coarsening,
3. surrogate model triage,
4. only escalate top candidates to fine `snappyHexMesh` + full-solve.

That is where teams generally hit two- to three-order-of-magnitude campaign throughput reduction in wall-clock.

### Boundary-layer + CFD stack comparison (in-practice)

| Stack | BL fidelity control | Mesh failure robustness | Reproducibility | Speed scaling path |
|---|---|---|---|---|
| `snappyHexMesh` (OpenFOAM-only) | high control over `addLayersControls`, but local fails around narrow gaps | medium; retries/backoffs are explicit | medium-high if dicts/manifests are locked | moderate; can be staged with external preprocess |
| `snappyHexMesh` + `addBufferLayers` (v2412+) + `nOuterIter` layering passes | better control over layer transitions around difficult edges | improved when tuned around difficult regions | same as above | additional passes increase cost but reduce late-stage failures |
| `cfMesh` + OpenFOAM | configurable layer logic can be more forgiving on some features | medium-high on difficult STL quality | medium; depends on external dict logic | moderate |
| External mesher + OpenFOAM import (`Gmsh`/`SALOME`) | good for scripted feature fields and size maps, then imported | high for cleaned CAD flows | high if CAD pipeline is deterministic | preprocess-heavy but can be stable at scale |
| External BL-focused environment (e.g., HyperMesh) | strongest explicit BL templating and edge/control knobs | high for industrial preprocess flows | high in enterprise workflow frameworks | not directly in OpenFOAM, but strong if stage partition is disciplined |

### What to do if you want to contribute upstream

OpenFOAM contribution flow is explicit about using community channels, planning in `OpenFOAM-dev`, and applying licensing steps before non-trivial submissions.

For code changes likely to be accepted:

1. Small scope PR (single path, minimal behavioral side-effects).  
2. Reproducer and tests included.  
3. Benchmarks with fixed versions and deterministic manifest.  
4. Clear risk note for deterministic mode and GPU mode if relevant.

For docs/CI-heavy workflows, there is an explicit expectation to run local CI checks before opening a merge request.

### Practical implementation order for your campaign stack (what to build first)

1. **Lock deterministic run manifest + failure contract**  
   This is the highest leverage item and does not require solver changes.
2. **Add pre-layer geometry gate + retries**  
   Run cheap checks before meshing and downshift to a lower layer recipe when geometry is near limits.
3. **Add fixed stage boundaries in pipeline**  
   e.g., "mesh only if candidate version >= X", "run medium fidelity only when residual trend qualifies".
4. **Introduce surrogate/filter loop**  
   Keep the majority of candidates from entering full-fidelity solves with cheap CFD surrogates or coarse filters.
5. **Only then optimize performance kernels**  
   Measure real campaign wall-clock before and after each micro-optimization.

A useful rule: build orchestration first, then PR risk. High-risk code changes without campaign hygiene create expensive churn and low reproducibility.

References used in this roadmap:

- OpenFOAM v2606 offloading announcement (std::execution, UMPIRE, stdpar direction): https://www.openfoam.com/news/main-news/openfoam-v2606/infrastructure
- OpenFOAM snappyHexMesh layer controls: https://doc.openfoam.com/2606/tools/pre-processing/mesh/generation/snappyhexmesh/layers/
- snappyHexMesh quick reference (v2606): https://doc.openfoam.com/2606/tools/pre-processing/mesh/generation/snappyhexmesh/quick-reference/
- Contribution process and licensing: https://openfoam.org/dev/how-to-contribute/ and https://openfoam.org/contributor-agreement/

## Pros / cons

### Pros

- Full-control workflow for parametric engineering loops.
- Huge set of solver families and utilities in one ecosystem.
- Strong CLI automation story (case cloning, scripted meshing, solver launch, postprocess).
- No license cost per solver.

### Cons

- Meshing and numerical setup need more discipline than turnkey packages.
- Geometry/mesh quality issues are a frequent source of failure.
- Missing turnkey CAD-to-CFD handoff unless you build the CAD/Python/API front end yourself.
- Enterprise support and certification are weaker than commercial suites.

---

## Related notes

- [[CFD]]
- [[blockMesh]]
- [[snappyHexMesh]]
- [[cfMesh]]
- [[paraFoam]]
- [[icoFoam]]
- [[simpleFoam]]
- [[pisoFoam]]
- [[pimpleFoam]]
- [[rhoSimpleFoam]]
- [[rhoPimpleFoam]]
- [[rhoCentralFoam]]
- [[interFoam]]
- [[compressibleInterFoam]]
- [[reactingFoam]]
- [[fireFoam]]
- [[buoyantSimpleFoam]]
- [[buoyantPimpleFoam]]
- [[chtMultiRegionFoam]]
- [[Ansys CFX]]
- [[SU2]]
- [[XFOIL]]
- [[Lattice Boltzmann Method]]
- [[OpenFOAM Projects]]

---

## Further reading (primary sources)

- OpenFOAM docs: user guide sections for case structure, parallel execution, meshing, and post-processing.  
- OpenFOAM v14 release notes (features and platform list).  
- Ansys Fluent command line and GPU docs (2026 R1 and v261/v252 docs).  
- Ansys CFX introduction (capabilities), solver manager docs, and cfx5solve command-line docs.

URLs:

- https://openfoam.org/version/14/
- https://openfoam.org/download/
- https://openfoam.org/download/windows/
- https://www.openfoam.com/documentation/user-guide/2-openfoam-cases/2.1-file-structure-of-openfoam-cases
- https://www.openfoam.com/documentation/user-guide/3-running-applications
- https://www.openfoam.com/documentation/user-guide/4-mesh-generation-and-conversion
- https://www.openfoam.com/documentation/user-guide/4-mesh-generation-and-conversion/4.3-mesh-generation-with-the-blockmesh-utility
- https://doc.openfoam.com/2306/tools/pre-processing/mesh/generation/blockMesh/blockmesh/
- https://www.openfoam.com/documentation/user-guide/4-mesh-generation-and-conversion/4.4-mesh-generation-with-the-snappyhexmesh-utility
- https://doc.openfoam.com/2606/tools/pre-processing/mesh/generation/snappyhexmesh/
- https://doc.openfoam.com/2606/tools/pre-processing/mesh/generation/snappyhexmesh/layers/
- https://doc.openfoam.com/2606/tools/pre-processing/mesh/generation/snappyhexmesh/castellation/
- https://openfoam.org/release/2-3-0/snappyhexmesh/
- https://openfoam.org/current-release/
- https://www.openfoam.com/news/main-news/openfoam-v20-12/pre-processing
- https://www.openfoam.com/news/main-news/openfoam-v2206/pre-processing
- https://www.openfoam.com/news/main-news/openfoam-v2306/pre-processing
- https://www.openfoam.com/news/main-news/openfoam-v2412/pre-processing
- https://www.openfoam.com/news/main-news/openfoam-v2512/infrastructure
- https://www.openfoam.com/news/main-news/openfoam-v2606/infrastructure
- https://ansyshelp.ansys.com/public/Views/Secured/corp/v252/en/flu_ug/tgd_user_workflow_guided_tasks_ftm_add_boundary_layer.html
- https://wiki.openfoam.com/High_Performance_Computing_(HPC)_Technical_Committee
- https://www.openfoam.com/documentation/user-guide/4-mesh-generation-and-conversion/4.5-mesh-conversion
- https://www.openfoam.com/documentation/user-guide/7-post-processing/7.1-parafoam
- https://www.openfoam.com/documentation/user-guide/a-reference/a.2-standard-utilities
- https://doc.openfoam.com/2312/fundamentals/command-line/
- https://doc.openfoam.com/2212/tools/processing/solvers/
- https://cfmesh.com/wp-content/uploads/2015/09/User_Guide-cfMesh_v1.1.pdf
- https://gmsh.info/doc/texinfo/
- https://gmsh.info/doc/texinfo/gmsh.pdf
- https://manpages.debian.org/trixie/netgen/netgen.1.en.html
- https://help.altair.com/hwdesktop/hwx/topics/pre_processing/meshing/mesh_controls_c.htm
- https://help.altair.com/hwdesktop/hwx/topics/pre_processing/meshing/mesh_controls_type_edge_layer_r.htm
- https://help.altair.com/hwdesktop/hwx/topics/panels/help399.htm
- https://help.altair.com/hwdesktop/hwd/topics/reference/hm/_tetmesh.htm
- https://help.altair.com/hwdesktop/hwd/topics/reference/hm/_blmesh_2d.htm
- https://help.altair.com/hwdesktop/hwd/topics/reference/hm/_blmesh_2d_computeblthickness.htm
- https://www.help.altair.com/2021/hwsolvers/acusolve/topics/tutorials/acu/acu_1000_intro_hm_r.htm
- https://2022.help.altair.com/2022.2/hwdesktop/engsol/topics/tutorials/cfd/tut_cfd_1100_creating_hybrid_grid_varying_layer_thickness_t.htm
- https://2022.help.altair.com/2022.3/hwdesktop/engsol/topics/tutorials/cfd/tut_cfd_1200_cfd_meshing_with_automatic_bl_t.htm
- https://doc.openfoam.com/2306/tools/processing/solvers/rtm/multiphase/interFoam/
- https://doc.openfoam.com/2306/tools/processing/solvers/rtm/combustion/fireFoam/
- https://doc.openfoam.com/2306/tools/processing/solvers/rtm/incompressible/simpleFoam/
- https://doc.openfoam.com/2606/contributing/how-to/
- https://openfoam.org/dev/how-to-contribute/
- https://openfoam.org/contributor-agreement/
- https://ansyshelp.ansys.com/public/Views/Secured/corp/v261/en/flu_ug/flu_ug_startramp.html
- https://ansyshelp.ansys.com/public/Views/Secured/corp/v261/en/flu_ug/flu_ug_sec_gpu_solver_supported_features.html
- https://ansyshelp.ansys.com/public/views/secured/corp/v251/en/flu_ug/flu_ug_sec_gpu_solver_limitations.html
- https://ansyshelp.ansys.com/public/Views/Secured/corp/v251/en/flu_ug/flu_ug_sec_gpu_solver_supported_features.html
- https://ansyshelp.ansys.com/public/Views/Secured/corp/v252/en/cfx_intr/i1302231.html
- https://ansyshelp.ansys.com/public/Views/Secured/corp/v252/en/cfx_solv/i1304917.html
- https://ansyshelp.ansys.com/public/Views/Secured/corp/v252/en/cfx_solv/i1304960.html
- https://ansyshelp.ansys.com/public/Views/Secured/corp/v251/en/cfx_solv/i1304960.html
- https://ansyshelp.ansys.com/public/Views/Secured/corp/v251/en/pdf/Ansys_CFX_Introduction.pdf
- https://ansyshelp.ansys.com/public/Views/Secured/corp/v252/en/cfx_mod/i1345898.html
