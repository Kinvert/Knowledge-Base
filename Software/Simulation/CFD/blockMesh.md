# blockMesh

**blockMesh** is a core OpenFOAM mesher that generates a structured hexahedral background mesh from a dictionary description (`system/blockMeshDict`).

---

## 🧠 Overview

`blockMesh` is a fully scripted meshing utility: you define vertices, edges, blocks, patch labels, and grading, and OpenFOAM constructs a base Cartesian/structured mesh.

It is widely used as:

- the entire meshing method for simple CFD boxes/ducts;
- a first-stage mesh before advanced meshing (`snappyHexMesh`, `cfMesh`, etc.);
- a deterministic baseline for comparison studies.

---

## 🧰 workflow

1. Define geometry and topology in `system/blockMeshDict`.
2. Define block edges/vertices and boundary patches.
3. Run:

```bash
blockMesh
```

4. Inspect with:

- `checkMesh` for topology and quality,
- `paraFoam` or `paraView` for visual confirmation.

Quality controls are largely geometric via:

- grading (`simpleGrading`, `edgeGrading`),
- cell distribution per direction,
- and boundary face assignments.

---

## ✅ Why teams use it

- Fully deterministic: same dict gives same mesh.
- Fast and lightweight for simple domains.
- Good starting point for parametric sweeps.

Common use:

- canonical channels/pipes/boxes,
- coarse base mesh for refinement workflows,
- and quick geometry sanity checks before a more expensive mesher.

---

## ⚠️ Common failure modes

- bad patch naming causes wrong boundary condition application,
- excessive skewness from wrong grading,
- non-physical cell shapes from vertex ordering mistakes,
- accidental unit mismatch in dimensions (especially CAD-import-free workflows).

Mitigation:

1. Start with coarse dimensions.
2. Keep patch naming explicit from day one.
3. Validate with `checkMesh` before solver start.

---

## 🔀 Comparison table

| Tool | Topology basis | Geometry handling | Boundary-layer support | Typical strength |
|---|---|---|---|---|
| blockMesh | Structured (`hex`) blocks | Manual dict-only topology | Not built-in | Deterministic simple domains |
| snappyHexMesh | Surface-driven snapping | STL/triangulated surfaces | Optional via `addLayersControls` | Hex-dominant CFD with complex geometry |
| cfMesh | Algorithmic poly/hex pre/post | STL + background workflows | Optional pipeline-dependent | OpenFOAM-oriented alternative |
| Gmsh | CAD + parametric + FEM-style meshing | CAD/implicit geometry + script | Limited for OpenFOAM-native wall stacks | Broad preprocessing and meshing |
| Salome | GUI + CAD + mesher modules | STEP/IGES CAD strong | Multi-module workflows | Full CAD-driven geometry prep |
| Netgen | Unstructured tetrahedral generation | CAD/Triangulated input | Limited | Fast fallback for rough domains |

---

## 🔗 Related notes

- [[OpenFOAM]]
- [[CFD]]
- [[snappyHexMesh]]
- [[Mesh Generation]]
- [[Mesh Generation Software]]

## 🌐 Further reading

- OpenFOAM blockMesh user guide
- OpenFOAM tutorial dictionaries and examples
- OpenFOAM case setup references for patch and cell-zone design
