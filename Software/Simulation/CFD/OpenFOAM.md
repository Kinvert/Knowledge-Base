# OpenFOAM

OpenFOAM is an open-source CFD stack, not a single solver. It is a library+case framework that provides dozens of applications, utilities, and solvers through dictionaries and command line workflows. It covers meshing, solve, and post-processing in the same case-based system, which makes it a strong fit for scripted optimization loops and repeatable CFD pipelines.

---

## What it is (and what it is not)

OpenFOAM is used for more than “CFD only”:

- It includes pre-processing utilities (e.g. `blockMesh`, `snappyHexMesh`, converters).
- It includes solver applications (e.g. incompressible, compressible, multiphase, combustion, heat transfer, particle/lagrangian).
- It includes post-processing hooks (`reconstructPar`, `paraFoam`, data converters like `foamToVTK`).

What it does not provide:

- A single GUI-driven turnkey flow from CAD import to report like a managed CAD-CFD platform.
- A single “all-in-one” solver for every flow class (you choose solver by physics class).

The OpenFOAM user guide explicitly states there is no generic solver for all cases and directs users to class-specific solvers by category (incompressible, compressible, multiphase, turbulence class, etc.).

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
- Standard scalar/vector discretization controls, runtime settings, and linear solver choices are controlled from dictionaries; this is where reproducibility and automation depth come from.
- Dynamic mesh and mesh-motion support exists through dedicated utilities/solvers, but setup quality is heavily model- and case-specific and much less "black-box-safe".

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

This is why teams often pair OpenFOAM with external CAD and workflow tooling (or a preprocessor stage) rather than trying to do full feature CAD editing directly in the solver tree.

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
- [[Ansys CFX]]
- [[SU2]]
- [[XFOIL]]
- [[Lattice Boltzmann Method]]

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
- https://www.openfoam.com/documentation/user-guide/4-mesh-generation-and-conversion/4.4-mesh-generation-with-the-snappyhexmesh-utility
- https://www.openfoam.com/documentation/user-guide/4-mesh-generation-and-conversion/4.5-mesh-conversion
- https://www.openfoam.com/documentation/user-guide/7-post-processing/7.1-parafoam
- https://www.openfoam.com/documentation/user-guide/a-reference/a.2-standard-utilities
- https://doc.openfoam.com/2312/fundamentals/command-line/
- https://doc.openfoam.com/2212/tools/processing/solvers/
- https://doc.openfoam.com/2306/tools/processing/solvers/rtm/multiphase/interFoam/
- https://doc.openfoam.com/2306/tools/processing/solvers/rtm/combustion/fireFoam/
- https://doc.openfoam.com/2306/tools/processing/solvers/rtm/incompressible/simpleFoam/
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
