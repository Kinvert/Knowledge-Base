# snappyHexMesh

`snappyHexMesh` is OpenFOAM’s flagship automated meshing utility for surface-conforming CFD meshes. It is designed for geometry-driven campaigns where you want a **reproducible parametric mesh stage** that can be controlled from a dictionary and reused inside DOE/agent loops.

---

## What it is and how it fits in a loop

`snappyHexMesh` solves this practical problem:

- Your geometry workflow is external (`STEP`/`IGES` -> STL/OBJ/tri surface).
- You need cell control near complex walls, leading edges, and wakes.
- You want one pipeline that can run dozens-to-thousands of candidates.

A typical flow is:

1. CAD export to STL/obj and (ideally) triangulated surface QA.
2. Optional feature extraction (`surfaceFeatureExtract`).
3. Coarse background mesh generation (`[[blockMesh]]` or equivalent).
4. `snappyHexMesh` refinement + snapping + optional boundary layers.
5. Quality gate (`checkMesh`, patch checks, boundary ownership checks).
6. OpenFOAM solve stage (`simpleFoam`, `pimpleFoam`, etc.).

It works because the mesh definition is a plain text control file, so geometry and quality policy can be regenerated from script outputs each campaign iteration.

---

## What classes of geometry it is strong on

- External and internal bluff-body-like shapes with separations and wake-sensitive outer boundaries.
- Surfaces that are mostly closed and cleanly manifold.
- Flows where wall treatment (layer quality + patch fidelity) is a major KPI.
- Parametric sweeps where mesh adaptation is repeated with small geometry deltas.

It struggles when:

- Geometry has severe non-manifold or broken shell topology.
- Very thin slits/gaps are below your smallest practical tetra/hex scale.
- You require strict mesh topology reproducibility across large topology edits without retuning.

---

## The 3-stage meshing model

`snappyHexMesh` is normally staged through three operational phases controlled by dict switches:

- `castellatedMesh`: build initial cell population around surface envelopes.
- `snap`: move near-surface vertices/points onto CAD features.
- `addLayers`: generate near-wall prism layers for boundary layers.

You can run only one part during debugging by setting these flags.

### 1) Castellated mesh

Goal: convert background mesh into refined cells around chosen geometry features.

It uses local refinement rules and feature capture to control where mesh is fine vs coarse.

What you configure:

- surface/box/channel-like refinements,
- global growth constraints,
- refinement caps and balancing budgets,
- `locationInMesh` seed point that must lie in the intended fluid region.

Operational consequence:

- too little refinement = staircase-ish wall capture;
- too much refinement = runaway cell count/iteration time and memory pressure.

### 2) Snapping

Goal: project cell faces toward triangulated geometry so wall and interface representation better reflects curvature and edges.

You control:

- snap iteration budget,
- patch projection tolerances,
- feature snap aggressiveness,
- relaxation and fallback behavior.

Operational consequence:

- low snap tolerance = robust for dirty surfaces but poor surface fidelity;
- high tolerance/aggressiveness = better fidelity, more risk of inverted/bad cells.

### 3) Layer addition

Goal: build prism/hexa-like boundary layers for wall-resolution-dependent models (RANS/SST etc.).

You control:

- first-layer thickness and growth,
- target final thickness,
- wall angle constraints,
- where to skip layers when topology cannot accept growth.

Operational consequence:

- good layers improve near-wall behavior for turbulence models;
- bad settings can collapse layers and fail the entire mesh.

---

## Deep control knobs in `system/snappyHexMeshDict`

OpenFOAM naming differs slightly by major version, but the same structure is consistent.

### `castellatedMeshControls`

Typical controls and what they mean:

- `maxLocalCells`, `maxGlobalCells`: hard memory/time safety caps.
- `minRefinementCells`: prevents over-refining tiny isolated islands.
- `nCellsBetweenLevels`: smoothness between refinement levels.
- `maxRefinementThickness`: (or equivalents depending on release) controls how far refinement propagates.
- `resolveFeatureAngle`: decides what is considered a sharp geometry feature.
- `features`: references `.eMesh` edge features from `surfaceFeatureExtract`.
- `refinementSurfaces`: per-surface target refinement range plus patch behavior.
- `refinementRegions`: box/cylinder/ring-like local refinements not tied to a surface.
- `locationInMesh`: seed point used to classify solid vs fluid side.

Practical playbook:

- Keep growth ratio and surface refinement conservative on first pass.
- Validate feature extraction before meshing when geometry has fillets/edges.
- Raise region/cell caps slowly; do not jump from 2M to 80M cells in one iteration.

### `snapControls`

- `nSmoothPatch`, `nSmoothNorm`, `nSmoothInternal`: influence convergence of vertex movement.
- `nSolveIter`: how many projection solves run per cycle.
- `nRelaxIter`: stability damping.
- `snapTolerance`, `implicitFeatureSnap`, `explicitFeatureSnap`: fidelity vs robustness trade.

### `addLayersControls`

- `relativeSizes`: layer thickness interpreted relative to local base cell size.
- `expansionRatio`: growth between layers (too high causes collapse).
- `finalLayerThickness`: target last layer thickness.
- `minThickness`, `nGrow`: minimum admissible and face growth controls.
- `featureAngle`: where layers are suppressed on sharp corners.
- `slipFeatureAngle`: helps suppress layer failure near sharp convex/concave transitions.
- `nSmoothSurfaceNormals` / `nSmoothNormals` / `nSmoothThickness`: how heavily surface normals are regularized.

For production: start with fewer layers + conservative growth; then increase target count only after quality gates pass.

### `meshQualityControls`

This section is your “blast wall” before solver:

- `maxNonOrtho`, `maxBoundarySkewness`, `maxInternalSkewness`
- `maxConcave` and `minVol`, `minTetQuality` (or equivalent family-specific names)
- `minFaceWeight`, `minDeterminant`, `minOrthogonality`-class controls

Use this to force rejection before solve. If you are doing batch campaigns, do not run long solver jobs on meshes that failed quality gates.

---

## SnappyHexMesh-specific failure taxonomy

### Geometry-stage failures

- Non-manifold shell / open edge: snapping can fail or generate unstable boundary layers.
- Inconsistent patch names: patches compile but BC mapping in downstream case becomes wrong.
- Wrong normals orientation: wall/front/side flips and inverted regions.

Fix order:

1. `surfaceCheck` on every imported surface.
2. rebuild feature extraction.
3. compare triangulation density against curvature scale.

### Seed / domain failures

- `locationInMesh` in wrong region can tag geometry as solid.
- Background mesh too coarse to represent internal channels.

Fix order:

1. quick point-in-mesh smoke test with known inside point,
2. reduce base cell size around complex openings,
3. regenerate before running snap/layers.

### Layer failures

- Layering collapses near sharp corners.
- Missing layers around thin or rapidly changing edges.
- Excessive cell non-orthogonality after layer growth.

Fix order:

1. lower growth ratio;
2. reduce layer count and increase first layer conservatively;
3. allow skipLayering on problematic patches if physics allows.

---

## Deterministic agent loop pattern

The useful part for agentic optimization is deterministic generation. A minimal stage contract usually includes:

- geometry file + patch semantic map,
- dict template + variable overrides,
- required validation gates,
- solver selection based on family.

A simple loop pattern:

- read design row from JSON/CSV/XLSX,
- render `snappyHexMeshDict`, patch map, base mesh settings,
- run meshing,
- reject candidate on checks,
- persist mesh stats + KPI.

Example command skeleton:

```bash
# pre: geometry already validated in triSurface/
surfaceFeatureExtract -case "$case"  # optional but recommended
blockMesh -case "$case"
snappyHexMesh -case "$case" -overwrite > logs/snappy_$run_id.log
checkMesh -case "$case" > logs/checkMesh_$run_id.log
```

Use campaign-level logic to fail fast when:

- `checkMesh` reports severe non-orthogonality/spikes,
- wall patch counts changed unexpectedly,
- volume differs from expected domain target by more than tolerance.

---

## Comparison matrix (where snappyHexMesh stands)

| Method | Geometry input | Topology control | Wall-layer behavior | Parametric repeatability | Good in loop when |
|---|---|---|---|---|---|
| [[snappyHexMesh]] | STL/tri surfaces + optional features | Surface-feature + region driven | Built-in boundary layer generation | Very high | surface complexity is moderate-high and wall resolution matters |
| [[blockMesh]] | Manual block definitions | Full manual control, explicit topology | Manual only | Highest (same dict = same mesh) | geometry is mostly boxes, ducts, and regular passages |
| [[cfMesh]] | STL/tri surfaces | Adaptive unstructured strategy | Some pipelines support near-wall controls | Medium-High | robustness is more important than advanced layering fidelity |
| [[Gmsh]] | CAD/parametric + scripts | Strong CAD topology and meshing ops | Needs export conversion to OpenFOAM-ready mesh path | Medium | broader multiphysics preprocessing is needed before OpenFOAM |
| [[Netgen]] | CAD/tri surfaces | Robust tetra fallback | Limited OpenFOAM-native layering | Medium | you need a fast tetra fallback for degraded geometry |
| Ansys Meshing | CAD native formats | Industrial parametric and repair workflows | Strong boundary-layer and template presets | Medium-High | you want GUI-quality mesh governance and enterprise standards |

---

## What it cannot do alone

- It is not a CAD system: you still need external preprocessing for boolean cleanup, feature suppression, and design-tree edits.
- It will not “fix bad topology” reliably; you still need upstream geometry hygiene.
- It does not replace turbulence-wall model selection; those live in solver dictionaries (`system/fvSchemes`, `system/fvSolution`, and `constant/turbulenceProperties`).
- Geometry changes that alter topology dramatically often require manual control retuning.

---

## For bluff-body diffuser studies

For diffuser-angle campaigns (your target use-case), teams usually structure as:

- small angle sweep range in geometry generator,
- `snappyHexMesh` with fixed `refinementSurfaces` topology and only geometric scale parameters as variables,
- separate “convergence-safe” profile for first screening.

Pragmatic defaults:

- start with 1 layer for first 20% of candidates,
- promote to 2–4 layers only after stable baseline,
- lock background block topology unless geometry family changes materially.

---

## Related notes

- [[OpenFOAM]]
- [[CFD]]
- [[blockMesh]]
- [[cfMesh]]
- [[paraFoam]]
- [[ParaView]]
- `checkMesh`
- `surfaceFeatureExtract`

## Further reading

- OpenFOAM user guide: meshing → snappyHexMesh
- OpenFOAM `snappyHexMesh` tutorial trees and command-line examples
- `snappyHexMesh` validation workflows in release docs
