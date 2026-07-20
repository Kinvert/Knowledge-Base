# cfMesh

**cfMesh** is an OpenFOAM-oriented meshing framework aimed at generating quality meshes for finite-volume CFD workflows, often used as an alternative to `snappyHexMesh` for unstructured hex/tet-like preparations.

---

## 🧠 Overview

`cfMesh` is typically used in geometry-driven meshing workflows where users need:

- robust boundary capture,
- automated layer-like boundary treatment in some pipelines,
- and stable handling of STL-based CAD domains.

It sits in the same OpenFOAM meshing ecosystem as `blockMesh` and `snappyHexMesh`, but with its own meshing strategy.

---

## 🧰 Typical workflow

1. Clean and export geometry surfaces (often STL/triangulated form).
2. Configure mesh controls and refinement strategy in OpenFOAM case dictionaries.
3. Run cfMesh meshing stage.
4. Run `checkMesh` and mesh inspection before solver launch.

This is usually used either:

- directly for production-ready meshes in challenging CAD domains, or
- as a backup path when `snappyHexMesh` requires extensive tuning.

---

## ✅ Why teams choose it

- Useful when surface complexity or topology quality makes automated meshing expensive.
- Can reduce setup overhead for some CAD cases versus heavily-tuned snapping workflows.
- Fits OpenFOAM case structures and automated campaign scripts.

Use cases:

- industrial geometry with many face features,
- robustness-first meshing in geometry-heavy design studies,
- cross-check meshes alongside `snappyHexMesh` results.

---

## ⚠️ Pitfalls

- behavior differs by OpenFOAM version/build and package source.
- boundary patch interpretation can surprise teams used to `snappyHexMesh`.
- quality defaults should be validated for your turbulence/BC stack.

Recommended debugging order:

1. Verify surface cleanliness and unit consistency first.
2. Start coarse and validate patch topology.
3. Add local refinements only after base mesh is stable.
4. Compare wall distances / first-layer quality against expected Y+ or wall-resolution goals.

---

## 🔀 Comparison table

| Tool | Primary fit in OpenFOAM | Geometry input | Setup style | Best first choice when |
|---|---|---|---|---|
| cfMesh | OpenFOAM-native alternative mesher | STL/triangulated surfaces | Dictionary-driven | You want an alternative to snappy with similar workflow |
| snappyHexMesh | OpenFOAM + Open-source snapping | STL/tri surfaces + features | Dictionary-driven | You need hex-dominant mesh + layer workflow |
| blockMesh | Structured base meshes | Manual block dictionary | Explicit `blockMeshDict` | Geometry is simple/axis-aligned |
| Gmsh | Multi-domain CAD meshing | CAD/geo + scripts | Script + GUI | You need CAD pre-processing and general meshing |
| Salome | Full CAD preprocessing platform | STEP/IGES native | GUI + scripted modules | You need integrated geometry cleaning + meshing |
| Netgen | Unstructured tetrahedra | CAD/surfaces | CLI/GUI | Rapid fallback or lower-overhead tetrahedralization |

---

## 🔗 Related notes

- [[OpenFOAM]]
- [[CFD]]
- [[snappyHexMesh]]
- [[blockMesh]]
- [[Mesh Generation]]
- [[Mesh Generation Software]]

## 📚 Further reading

- OpenFOAM mesher ecosystem notes and case docs
- cfMesh community examples in OpenFOAM workflows
- Comparative mesher notes in this vault
