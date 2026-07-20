# Netgen

**Netgen** is an open-source unstructured mesh generator and geometry meshing utility commonly used in finite-element style workflows. It is frequently paired with CAD import or surface input when a fast, robust tetrahedral mesh is needed.

---

## 🧠 Overview

Netgen is useful when teams need dependable tetrahedral meshing without a heavy full-CAE UI overhead. It supports rapid retries, reasonable quality defaults, and straightforward export patterns for downstream solvers.

---

## 🧰 Core concepts

- **Geometry handling**: import-oriented preprocessing with basic cleanup and tolerance setup.
- **Tetrahedral generation**: primary engine behavior is robust volumetric tetra meshing.
- **Automation**: practical for scripted and repeated mesh regenerations.

Typical pipeline:

1. Prepare geometry (surface or CAD-derived input).
2. Generate mesh with target quality parameters.
3. Inspect element quality.
4. Export for solver handoff.

---

## ✅ Why teams use it

- strong lightweight alternative when meshing must be fast and stable,
- good for preprocessing and parameter sweeps,
- useful as a fallback when other meshing methods over-refine or fail.

Common uses:

- FEM pre-processing,
- simple CFD-ready STL cleanup-to-tet fallback,
- mesh quality stress testing in repeated geometry campaigns.

---

## ⚠️ Limitations

- less explicit geometry CAD authoring vs full systems (e.g., Salome, Gmsh),
- limited native wall-layer workflow compared with dedicated CFD boundary-layer workflows,
- may need tuning for highly anisotropic feature capture.

---

## 🔀 Comparison chart

| Tool | Typical output | Geometry workflow | Complexity handling | Best fit |
|---|---|---|---|---|
| Netgen | Tetrahedral | Geometry/surface intake | Moderate | Quick robust meshing for FEM/CFD prep |
| TetGen | Tetrahedral (Delaunay) | Domain/constraint based | Good for robust local tetra | High-quality tetra for simulation |
| Gmsh | CAD + flexible element types | Script/CAD-first | Moderate-high | Full preprocess + mesh generation |
| Salome | Multi-module CAE | CAD-heavy pipeline | High | Geometry-driven multiphysics prep |
| snappyHexMesh | OpenFOAM surface-based | STL + OpenFOAM dictionaries | High | Hex-dominant CFD from OpenFOAM |
| blockMesh | Structured block | Manual dict | Low | Canonical structured domains |

---

## 🔗 Related notes

- [[Mesh Generation]]
- [[Mesh Generation Software]]
- [[CFD]]
- [[FEM]]
- [[Gmsh]]
- [[TetGen]]
- [[snappyHexMesh]]

## 📚 Further reading

- Netgen mesh quality guides
- Surface-to-volume preprocessing workflows
- Solver compatibility notes for tetra outputs
