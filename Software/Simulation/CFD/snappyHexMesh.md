# snappyHexMesh

**snappyHexMesh** is the core **OpenFOAM** mesh utility for generating high-quality, hex-dominant unstructured meshes from CAD/triangulated geometry. It is designed for complex external and internal flow domains where structured mesh generation is too difficult to do by hand.

---

## 🧠 Overview

The tool works by starting from a background mesh (typically generated with `[[blockMesh]]`) and then:

- carving the surface-conforming domain (`castellation`),
- snapping boundary points onto the geometry,
- and optionally adding prismatic boundary layers.

The result is usually a mostly hexahedral (`hex`) cell set with prisms near walls and optional local refinements, which is a practical compromise for CFD stability and solver performance.

---

## 🧰 Workflow in an OpenFOAM case

Typical sequence:

1. Prepare the geometry as watertight surface files (`.stl`, `.obj`, sometimes `.ply` via conversion) in `constant/triSurface/`.
2. Optional preprocessing with `surfaceFeatureExtract` to capture sharp edges.
3. Generate a coarse background mesh (`[[blockMesh]]` is common).
4. Configure `system/snappyHexMeshDict`.
5. Run:

```bash
snappyHexMesh
```

Use `-overwrite` to replace the input mesh with the snapped/layered result in-place, and `-parallel` when the case has been decomposed for distributed execution.

6. Run quality checks (`checkMesh`) and boundary audits before solver launch.

Typical case structure:

- `constant/triSurface/` → surface geometry and optional feature data
- `system/snappyHexMeshDict` → meshing recipe
- `system/fvSchemes`, `system/fvSolution` → post-generation numerical setup

---

## ⚙️ How snappyHexMesh controls map to outcomes

`snappyHexMeshDict` is usually split into sections:

- `castellatedMeshControls`: surface snapping/refinement strategy (target cell sizes, feature refinement, max cell counts, `locationInMesh` seed rules)
- `snapControls`: how aggressively patch points move onto the geometry and when snapping falls back
- `addLayersControls`: boundary-layer thickness, growth, and layer growth failure handling
- `meshQualityControls`: minimum quality constraints and auto-fixes for skewness/angles/non-orthogonality
- `mergeTolerance` and `castellatedMesh`/`snap`/`addLayers` toggles for pipeline control

Practical tuning pattern:

- increase surface refinement where curvature or small gaps exist,
- keep layer count and final thickness conservative at first,
- reduce `nRelaxIter`/`nSmoothPatch` if convergence stalls,
- and turn on stricter layer controls after the coarse shape is stable.

---

## ✅ Why teams use it

- Strong balance of geometry fidelity and computational efficiency for turbulent wall-bounded flow.
- Good integration with `OpenFOAM` solver ecosystems and case directories.
- Reproducible parameterized meshing through dictionary files, which suits batch campaigns.

Common use cases:

- External aerodynamics with curved bodies and inlets/outlets
- Internal flow passages where boundary layers are critical
- Geometry-heavy optimization loops (where automation beats manual meshing)

---

## ⚠️ Gotchas and failure modes

These are the first problems to check when a run fails:

- **Geometry quality problems**: non-manifold triangles, open edges, and inconsistent normals often break snapping earlier than solver convergence.
- **Layer failure near sharp features**: too-high growth ratio / too-thin initial layer -> layer collapse or skipped cells.
- **Excessive base refinement**: `maxGlobalCells` reached without useful geometric benefit.
- **Wrong `locationInMesh`**: points in void instead of fluid region causes inversion/void errors.
- **Boundary patch mismatches**: `geometry` patch names not matching desired surface names leads to mis-assigned BC regions.

General debugging order:

1. Validate geometry files in a viewer.
2. Run with conservative castellation first (smaller target sizes).
3. Enable snapping with higher tolerances for hard cases.
4. Add layers only after first mesh shape is stable.
5. Recheck with mesh quality tools and patch IDs before solver start.

---

## 🔀 Comparison chart (mesh strategy context)

| Approach | Core geometry handling | Cell topology | Boundary layers | Best fit |
|---|---|---|---|---|
| [[snappyHexMesh]] | Surface-driven STL workflows | Hex-dominant + prisms | Optional, built-in | Complex CFD geometries with automation |
| [[blockMesh]] | Topology-first, structured | Structured hexahedra | Manual / none | Simple boxes and controlled domains |
| [[cfMesh]] | OpenFOAM-compatible mesher | Mixed/poly elements | Optional depending on pipeline | Simpler OpenFOAM workflows without full snappy tuning |
| [[Gmsh]] | CAD/boolean-first mesher | Triangle/tet dominant with options | Not OpenFOAM-native | Multi-physics pre-processing, broader geometry operations |
| [[Netgen]] | Robust unstructured tetra generation | Tetrahedral | Limited OpenFOAM coupling | Rapid fallback when surface input is messy |
| Ansys Meshing | GUI-driven enterprise meshing | Mixed, tuned templates | Strong industrial presets | Teams with Ansys license and strict enterprise validation |

---

## 🔗 Related notes

- [[OpenFOAM]]
- [[CFD]]
- [[Mesh Generation]]
- [[Mesh Generation Software]]
- [[FEM]]
- [[FEniCSx]]
- [[paraFoam]]
- [[ParaView]]

## 🌐 Further reading

- OpenFOAM user guide: Meshing → snappyHexMesh
- OpenFOAM foundation docs and sample tutorials
- `snappyHexMesh` case templates in your local OpenFOAM installation
