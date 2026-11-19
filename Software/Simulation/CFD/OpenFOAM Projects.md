# OpenFOAM Projects

This note is a practical index of real GitHub/OpenFOAM workflows you can reuse for a parameter-sweep CFD pipeline, not a theory file.  

If you are building a bluff-body workflow where CAD and mesh vary per iteration (diffuser angle, ground clearance, side geometry), this is where you start.

---

## Why study OpenFOAM projects

OpenFOAM itself is not a turnkey product. For iterative CAD-to-CFD loops, the bottleneck is usually project scaffolding, not solver primitives:

- how cases are generated from parameter files,
- how meshing is encoded and guarded,
- how failures are normalized into a “retry with fix” loop,
- how KPI extraction is made deterministic,
- how runs are organized so you can resume and compare thousands of candidates.

Project repositories are how teams encode those things.

---

## Common control structure used across successful projects

The same skeleton appears in almost every project:

1. `design_space` read
2. case/template rendering
3. mesh stage
4. run stage (serial or MPI/HPC)
5. logs + quality checks
6. post-processing and KPI export
7. registry/manifest update

Most failures happen in steps 3–5, so automation quality depends mostly on:

- explicit checks (`checkMesh`, parser checks, residual sanity),
- consistent naming across patches/cases,
- deterministic directory naming,
- stable mesh repair and layer controls,
- hard output schema.

---

## High-value projects (and what to steal)

## 1) DLR-RY/caseFoam

Core idea: OpenFOAM case manipulation layer built on PyFoam for large parametric trees.

- It explicitly says it is "automatically setup OpenFOAM cases with varying conditions" and is installed as a Python package.
- It shows parameter studies by combining case dimensions, e.g. `caseStructure` and `caseData` dictionaries, then `casefoam.mkCases(..., hierarchy='tree', writeDir='Cases')`.
- Post-processing is first-class and returns pandas data (`time_series`, `positional_field`, `posField_to_timeSeries`), with examples using `holoviews` for visualization.

What is useful for a bluff-body loop:
- deterministic nested case trees (e.g., `angle -> clearance -> mesh -> run_id`),
- explicit manipulation options (`replace string`, execute bash, dict substitution),
- easy migration to DOE-style generation.

Tradeoffs:
- it is less “single-command” and needs solid Python/case structure discipline,
- still strong when you want a mature param sweep generator.

Project links:
- [caseFoam README](https://raw.githubusercontent.com/DLR-RY/caseFoam/master/README.rst)
- [casefoam docs index](https://casefoam.readthedocs.io/)
- [casefoam API docs](https://casefoam.readthedocs.io/en/latest/getting_started.html)

---

## 2) merv1n34k/foamflow

Core idea: lightweight shell/Python pipeline wrapper with explicit `new -> pre -> run -> post`.

- Uses `case.template` + `Flowfile`; `flow new` populates case files; `flow pre` substitutes placeholders and runs mesh steps (including `blockMesh` example); `flow run` executes the case; `flow post` exports `*_result.csv`.
- Supports custom mesh generation through OpenSCAD when needed.
- Explicitly states “using out-of-the-box is not recommended,” which is a signal: it is flexible but expects you to own conventions.

What is useful for a bluff-body loop:
- a simple command surface for onboarding non-experts,
- straightforward case-by-case rendering and post CSV export,
- easy to wire into a GitHub Actions matrix or Python loop.

Tradeoffs:
- less opinionated/less resilient than richer frameworks,
- best when your team already owns domain-specific conventions.

Project link:
- [foamflow README](https://raw.githubusercontent.com/merv1n34k/foamflow/master/README.md)

---

## 3) gerlero/foamlib

Core idea: Python object API for OpenFOAM case editing and execution.

- Provides `FoamFile` and `FoamFieldFile` dict-like APIs,
- `FoamCase` for run/clean/result access,
- `AsyncFoamCase` for local concurrent solves,
- `AsyncSlurmFoamCase` for cluster concurrency.
- Supports both openfoam.com and openfoam.org runtimes in examples and docs.

What is useful for a bluff-body loop:
- clean abstraction boundary for your own agentic control loop,
- direct editing of `controlDict`, `fvSchemes`, `boundaryField` with Python dict semantics,
- easy integration with optimization loops and external schedulers.

Tradeoffs:
- you still need to own geometry/mesh generation,
- performance-critical loops depend on how you tune solver + post pipeline.

Project links:
- [foamlib README](https://raw.githubusercontent.com/gerlero/foamlib/main/README.md)
- [foamlib docs](https://foamlib.readthedocs.io/en/stable/)

---

## 4) csml-rpi/Foam-Agent

Core idea: end-to-end multi-agent automation system with built-in error correction.

- Claims full workflow from prompt to results with meshing, setup, execution, error correction, and visualization.
- Exposes LangGraph-orchestrated nodes (plan/input-writer/run/reviewer/visualization),
- adds FAISS RAG over tutorial corpus,
- supports up to 25 automatic correction loops,
- ships Docker flow and MCP integration (`/foam` style agent path, HTTP/stdio MCP tools).

What is useful for a bluff-body loop:
- if you want non-human-in-the-loop campaign progression,
- automatic retry/rewrite cycle for simulation errors,
- strong starting point when your process is still young.

Tradeoffs:
- heavy abstraction layer; less transparency than fully scripted stacks,
- better for rapid iteration than strict deterministic reproducibility unless you lock versions and configs.

Project links:
- [Foam-Agent README](https://raw.githubusercontent.com/csml-rpi/Foam-Agent/main/README.md)
- [Foam-Agent DeepWiki overview](https://deepwiki.com/csml-rpi/Foam-Agent/1-overview)

---

## 5) OpenFOAMCaseGenerator

Core idea: single-purpose case generation from a constrained DSL-like policy input.

- Generates complete case setup from a single Python class per case,
- policy-driven parameterization (`discretisation_policy`, mesh intent, RANS/LES intent),
- supports `post_processing` and multiple probe/surface outputs,
- checks common wrong combinations before runtime (e.g., LES with steady solver).

What is useful for a bluff-body loop:
- reduces setup-copy errors by replacing scattered tutorial edits with intent-driven specs,
- strong for creating standardized templates across many geometry families.

Tradeoffs:
- harder to override quickly for unstructured team conventions,
- you must map your team’s naming and KPI requirements into its generator vocabulary.

Project links:
- [OpenFOAMCaseGenerator README](https://raw.githubusercontent.com/tomrobin-teschner/OpenFOAMCaseGenerator/main/README.md)
- [OpenFOAMCaseGenerator docs](https://openfoamcasegenerator.readthedocs.io/en/latest/)
- [Usage docs section](https://openfoamcasegenerator.readthedocs.io/en/latest/usage.html)

---

## 6) OpenFOAM/ICengines (AATE-style engine CFD stack)

Core idea: an industrial-style repository layout with `Allwmake`, tutorial scripts (`Allmesh`, `Allrun`), and mesh+solver coupling.

- Current upstream description of this path indicates snappyHexMesh-only mesh prep for engine cases and notes mesh sizes can exceed ~1.5M cells and take significant serial time.
- Includes repository-style structure (`applications/utilities`, `tutorials`, `src`, `bin`) that mirrors larger OpenFOAM teams.

What is useful for a bluff-body loop:
- useful to copy the discipline of explicit tutorial entrypoint scripts and structured command flow,
- good template for long-running, higher-complexity project organization.

Tradeoffs:
- engine-specific content and heavier compile/build expectations,
- less directly portable for lightweight geometry-driven campaigns.

Project link:
- [ICengines/AATE README](https://raw.githubusercontent.com/OpenFOAM/ICengines/master/README.md)

---

## 7) yushan-lab/openfoam-cfd-minilab

Core idea: reproducible validation repository with strict output manifests and versioned workflows.

- Demonstrates explicit smoke and full validation workflows with scripts for:
  - running cases with deterministic scripts,
  - exporting artifacts,
  - quality checks via `checkOutputs`-style scripts,
  - GitHub Actions reproduction.
- Uses multiple mesh levels and formal quality gates for grid studies.

What is useful for a bluff-body loop:
- best example of governance for engineering quality (manifest, quality gates, artifact registries),
- easy pattern to avoid “this result looks right” drift during long campaigns.

Tradeoffs:
- narrow case scope in repo content (not a generic meshing framework),
- best as QA scaffold, not as general param generator.

Project link:
- [OpenFOAM CFD validation lab README](https://raw.githubusercontent.com/yushan-lab/openfoam-cfd-minilab/main/README.md)

---

## 8) petebachant/NACAFoil-OpenFOAM + Calkit

Core idea: reproducible case-space with `calkit` CLI and profile execution.

- Provides project-level command model: `calkit pull`, `calkit run`, and profile-driven execution via `calkit xenv`.
- Useful for turning one geometry/param pair into repeatable command sets.
- Example pattern passes explicit profile and angle at run time.

What is useful for a bluff-body loop:
- if your team wants strict command governance and environment reproducibility,
- useful when geometry families are fixed and you are validating variants with strong baseline repeatability.

Tradeoffs:
- not as rich in meshing orchestration as the other frameworks,
- less flexible for large project expansion without custom glue.

Project link:
- [NACA foil OpenFOAM README](https://raw.githubusercontent.com/petebachant/NACAFoil-OpenFOAM/master/README.md)

---

## 9) gerlero/setup-openfoam (workflow support, not full solver project)

Core idea: CI bootstrap layer for version-stable OpenFOAM environments.

- GitHub Action that installs and activates selected OpenFOAM versions in CI.
- Demonstrates matrix patterns across versions and runner OSes.
- Includes a `cache` flag for repeated runs.

What is useful for a bluff-body loop:
- if your optimization loop is in GitHub Actions and version drift is breaking convergence,
- useful to keep solver versions explicit (`openfoam-version` matrix).

Project link:
- [setup-openfoam README](https://github.com/gerlero/setup-openfoam/blob/main/README.md)

## 10) airshaper/adaptive-mesh-refinement

Core idea: a parameterized external-flow case that uses multi-stage snappyHexMesh refinement with solution-driven feedback and `Allrun` orchestration.

- The repository workflow has two named case trees (`primal_refinement`, `primal_run`) and a deliberate two-phase meshing pattern:
  - Stage 1: castellated-only mesh pass with baffles for geometry-safe initial cell topology.
  - Stage 2: snapping + first solve to generate a refinement scalar field.
  - Stage 3: adaptive refinement from computed fields (`grad(p)` or `curl(U)`) and a re-snap phase.
- It explicitly demonstrates a practical trick for bluff-body loops: **do not run adaptive refinement blind**; derive the field (`refVal`) from a finished first solve.
- This pattern is useful for diffusers/underbodies when wake gradients are highly localized and fixed-layer meshing over-refines clean regions.

Tradeoffs:
- Strong for cases where wake-adaptive cost/perf tradeoff matters.
- Not a general DOE engine; you still need external parameterization to sweep geometry cleanly.

Project links:
- [adaptive-mesh-refinement README](https://github.com/airshaper/adaptive-mesh-refinement/blob/main/README.md)
- [OpenFOAM foundation motorBike reference context (tutorial pattern)](https://github.com/openfoam/OpenFOAM/blob/master/tutorials/multiphase/interFoam/RAS/motorBike)

## 11) joslorgom/shapeOptimizationFoam

Core idea: OpenFOAM-native shape optimization loop with custom adjoint-style workflow (`shapeOptimizationFoam`) and explicit compile/run stages.

- The project compiles a custom solver, applies `Allprepare`, and runs optimization iterations that alternate:
  - primal solve,
  - adjoint solve,
  - sensitivity/shape update,
  - mesh motion and repeat.
- It is a real example of pushing optimization logic into the OpenFOAM application layer rather than only outer Python orchestration.

Why it matters for agentic workflows:
- If your long-term goal is "automate geometry evolution + CFD" this project gives a reference for where to implement shape-update kernels safely.
- It shows how to encode optimizer lifecycle inside a solver framework rather than ad hoc scripts.

Tradeoffs:
- Heavy C++ coupling and recompilation requirements.
- Geometry is not driven from CAD APIs directly in this repo; you'd still need an upstream parametric modeler + mesh update stage.

Project links:
- [shapeOptimizationFoam README](https://raw.githubusercontent.com/joslorgom/shapeOptimizationFoam/main/README.md)
- [shapeOptimizationFoam in OpenFOAM ecosystem listing via GitHub search](https://github.com/joslorgom/shapeOptimizationFoam)

---

## Comparing the project families against a bluff-body diffuser loop

This table is from practical constraints, not marketing.

| Project family | Best role in loop | Mesh strategy | Sweep complexity handling | Failure recovery | Best deployment style |
|---|---|---|---|---|---|
| `caseFoam` | Parameter tree + post-processing | any external mesh/mesher input into OpenFOAM case | high | manual review with scripted callbacks | local + HPC |
| `foamflow` | quick template pipeline + CSV KPI export | template + command stage (`blockMesh` via pre step) | medium | weak/explicit only | CI or shell-heavy teams |
| `foamlib` | Python-native orchestration | any mesh path, your own mesher choice | high | script-defined (retry in external driver) | local Python service + cluster |
| `Foam-Agent` | autonomous agentic loop | supports external `.msh`, built-in mesh stages | very high | automatic review/fix loops | MCP/agent platforms + Docker |
| `OpenFOAMCaseGenerator` | standardized case intent and guardrails | `file_properties` plus mesher handling in generator | high for repeated family classes | built-in consistency checks | code-first teams with strict templates |
| `setup-openfoam` | environment reproducibility | N/A | N/A | N/A | GitHub Actions CI |
| `NACAFoil` | command-reproducibility and profile execution | OpenFOAM default + profile-specific case generation | low | command-level | smaller controlled DOE |
| `adaptive-mesh-refinement` | wake-aware meshing branch for non-uniform flow localization | snappyHexMesh refinement + two-stage re-snap strategy | medium | explicit quality gates between passes | HPC/HPC-ish loops where solve-driven remeshing is worth cost |
| `shapeOptimizationFoam` | shape-update + solve loop in solver layer | any mesh workflow that supports mesh motion | high (geometry-optimizer coupling) | solver-level iteration state + external orchestration for stability checks | specialist teams with C++ stack ownership |

---

## Common “clever tricks” that consistently matter

### A. Keep your own case graph schema

Use a nested naming convention at least as deep as:

- `campaign/run_###/angle_###/clearance_###/mesh_###`

The schema should map both geometry generation and mesh policy.  

### B. Separate geometry from numerics

Do not let CAD variables directly rewrite solver dictionaries in one step. Use:

- geometry manifest (`geo.json`),
- mesh manifest (`mesh.yaml`),
- solve manifest (`solve.yaml`),
- KPI manifest (`kpi.yaml`).

This is the easiest path to audit and rollback.

### C. Persist quality gates

If you only log end time and drag, you do not have an optimization process—you have roulette.

Minimum per-case gate set:
- `checkMesh` pass,
- expected patch count / patch types,
- y+/first-layer acceptance rule,
- residual trend sanity (not just final residual values),
- force-convergence or KPI plateau check.

### D. Keep “post” deterministic

Every project above works better when post-processing is deterministic:
- fixed sample points,
- fixed naming for forces, field exports, and timing windows,
- fixed CSV schema (not ad hoc parsing per case).

### E. Use manifest-driven loops, not folder-scan loops

Your orchestrator should read one manifest and then emit jobs. Folder enumeration as logic source leads to path-collision errors and stale states.

---

## Suggested bluff-body diffuser workflow (practical blueprint)

Use this if you are trying to run `angle` sweeps and need geometry from JSON/XLSX:

1. **Design file**
   - `campaign.json` or `design.xlsx` defines geometry variables:
     `diffuser_angle`, `ledge_height`, `ramp_radius`, `mesh_level`, `target_Re`, etc.
2. **Geometry stage**
   - Use a CAD API to generate STEP/STL for each sampled row,
   - ensure closed surfaces and named patches.
3. **Project runner**
   - feed row into one project family:
     - `foamlib` for Python-native control,
     - `foamflow` for template-based teams,
     - `Foam-Agent` for autonomous fallback.
4. **Meshing stage**
   - enforce gap-aware controls before solve:
     for `snappyHexMesh`, check background size and surface edge quality,
     tune `addLayers` thresholds when undertray clearance is tight.
5. **Run stage**
   - run with OpenFOAM command set from project template,
   - capture logs and failure mode tags.
6. **Post stage**
   - compute drag/lift split by component,
   - compute wake momentum loss, pressure recovery, and underbody recirculation proxies,
   - export fixed CSV schema.
7. **Loop engine**
   - rank candidates by objective,
   - apply next sampling step,
   - keep top-N candidates and mutate sampling radius.

Useful pattern:
- `cfd_runner.py` pulls design rows → renders geometry + case → calls OpenFOAM runner → writes `case_manifest.json` + `kpi.csv` + `paraview_export` paths.

---

## Important caveat for OpenFOAM projects

Most "end-to-end" projects help you move from geometry to results faster, but they do not replace:
- geometric patching discipline,
- mesh strategy for difficult underbody strakes,
- numerical strategy review (stability/accuracy tradeoffs).

If your target includes tight diffusers and very narrow strake gaps, you still need explicit geometry-aware mesh logic in the outer loop.

---

## Relationship to other CFD ecosystem options

These projects are usually strongest when paired with:

- [[OpenFOAM]] for solver and core utilities,
- [[blockMesh]] or external mesher front-end for stable topology,
- [[snappyHexMesh]] for fast structured-to-unstructured transition in many geometry classes,
- [[paraFoam]] and custom post-processing for report generation.

If your team is asking for turnkey GUI work, this family is not the answer by itself.  
If your team wants scalable deterministic loops and can invest in conventions, this is exactly the right family.

---

## Suggested next step

The most practical stack for your use case is:

1. `OpenFOAMCaseGenerator` (for strict templated case integrity),
2. `foamlib` (for Python orchestration),
3. `setup-openfoam` in CI (to pin solver versions),
4. `openfoam-cfd-minilab` style quality manifests (for repeatability confidence).

Then decide whether to add `Foam-Agent` for autonomous recovery after each run.

---

## Project-scan summary (what this yielded)

This was a useful amount of material, not a shallow sample:

- **Most strong finds were orchestration patterns** rather than one-turnkey CFD products.
- **High actionability (immediate use):** templating + manifest-driven case graphs + quality gates.
- **Medium actionability:** adaptive and solver-level loops (need more integration work).
- **Low actionability:** direct undertray/ground-effect-specific automotive repositories (few direct analogs).

Most directly similar examples for your bluff-body idea:

- `airshaper/adaptive-mesh-refinement` (adaptive mesh loops that use field feedback),
- `caseFoam` + `foamlib` (deterministic campaign scaffolding),
- `shapeOptimizationFoam` (solver-level optimization loop if you later migrate into C++).

From a campaign engineering perspective, the highest-ROI next step is:

1. lock deterministic manifests/CLI contracts first;
2. add explicit pre-mesh failure reason coding;
3. then integrate surrogate-guided candidate pruning to reduce full-order volume.

---

## External resources

- https://github.com/DLR-RY/caseFoam
- https://github.com/merv1n34k/foamflow
- https://github.com/gerlero/foamlib
- https://github.com/csml-rpi/Foam-Agent
- https://github.com/tomrobin-teschner/OpenFOAMCaseGenerator
- https://github.com/OpenFOAM/ICengines
- https://github.com/yushan-lab/openfoam-cfd-minilab
- https://github.com/petebachant/NACAFoil-OpenFOAM
- https://github.com/gerlero/setup-openfoam
- https://github.com/airshaper/adaptive-mesh-refinement
- https://github.com/joslorgom/shapeOptimizationFoam

---

## Related notes

- [[OpenFOAM]]
- [[blockMesh]]
- [[snappyHexMesh]]
- [[paraFoam]]
