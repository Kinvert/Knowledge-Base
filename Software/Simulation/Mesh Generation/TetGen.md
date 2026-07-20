# TetGen

**TetGen** is an open-source library and command-line tool for creating tetrahedral meshes, often based on Delaunay-based refinement and constrained triangulation approaches.

---

## 🧠 Overview

TetGen is commonly used when teams want high-quality tetrahedral meshes for FEM/CFD preprocessing, especially in pipeline stages where triangle/tet structure control is important and geometry is already in a compatible preprocessed form.

It appears as:

- a standalone meshing library,
- a CLI tool in scripted toolchains,
- and an element generator feeding solver-facing preprocess flows.

---

## 🧰 Core usage pattern

1. Prepare an input definition (`.poly`, `.smesh`, or similar depending on interface).
2. Set tetra refinement constraints and region/quality preferences.
3. Run mesher generation.
4. Convert/export into downstream format expected by solver toolchains.

The practical advantage is repeatable tetra generation with consistent quality controls in scripted contexts.

---

## ✅ Why teams use it

- fast, deterministic tetra generation for many workflows,
- compact CLI path for batch pipelines,
- good balance of quality and simplicity in simulation preprocessing.

Common use cases:

- FEM meshing with tetra constraints,
- tetra fallback when full CAD mesher tuning is too slow,
- solver input generation in repeated design batches.

---

## ⚠️ Limitations

- mesh output ecosystem often needs conversion plumbing to target simulators,
- not as feature-complete for hex-dominant boundary-layer CFD as OpenFOAM-native tools,
- quality tuning can become sensitive with poor input geometry conditioning.

---

## 🔀 Comparison chart

| Tool | Output focus | Strength | Common ecosystem | Best use case |
|---|---|---|---|---|
| TetGen | Tetrahedral meshes | Strong constrained/Delaunay behavior | FEM preprocessing pipelines | High-repeatability tetra generation |
| Netgen | Fast robust tetra | Lightweight and forgiving | FEM/quick CFD prep | Rapid fallback meshing |
| Gmsh | General CAD + meshing | Full geometry-to-mesh toolchain | FEA/FEM + CAD integration | Complex scripted + CAD workflows |
| snappyHexMesh | Hex-dominant + layers | OpenFOAM CFD-specific | OpenFOAM solver stacks | External/internal flow with snapping |
| cfMesh | OpenFOAM meshing alternative | Robust OpenFOAM preprocessing | OpenFOAM environments | Geometry-driven OpenFOAM workflows |
| blockMesh | Structured base mesh | Deterministic structured grids | OpenFOAM | Simple canonical geometries |

---

## 🔗 Related notes

- [[Mesh Generation]]
- [[Mesh Generation Software]]
- [[CFD]]
- [[FEM]]
- [[OpenFOAM]]
- [[Netgen]]
- [[Gmsh]]

## 📚 Further reading

- TetGen library interfaces and CLI references
- Surface-to-tetra conversion strategies
- Solver interface notes for tetrahedral pipelines
