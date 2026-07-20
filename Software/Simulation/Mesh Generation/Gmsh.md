# Gmsh

**Gmsh** is a general-purpose open-source mesh generator and preprocessor used across FEM, CFD, and general geometry pipelines. It supports GUI and scripting (`.geo`) workflows for building geometry and generating meshes.

---

## 🧠 Overview

Gmsh has three practical pillars:

- geometry kernel,
- meshing,
- and post-mesh export.

It is often used where teams need reusable geometric templates plus deterministic meshing for larger automation pipelines.

---

## 🧰 Core concepts

- `Point`, `Line`, `Surface`, `Volume` objects for constructive modeling.
- structured/unstructured mesh controls via options and size fields.
- support for tetrahedral/hexahedral/prismatic elements depending on pipeline and version constraints.
- command/script-driven automation via `.geo` files.

Because it is scriptable, teams often use Gmsh for reproducible mesh generation in CI or parametric studies.

---

## ✅ Practical use cases

- CAD pre-processing for FEM/CFD candidate geometry
- fast parametric geometry + mesh experiments before solver handoff
- mesh generation for thermal/structural multiphysics (with external solvers)
- converting and exporting to formats consumed by OpenFOAM/FEM ecosystems

---

## ⚠️ What can go wrong

- meshing quality depends heavily on sizing fields and transfinite constraints,
- geometry tolerance mistakes can produce sliver-quality artifacts,
- converter pipeline sometimes needs format tuning per target solver.

Mitigation:

1. Keep geometry unit conventions strict across CAD/import/export steps.
2. Use scripts for repeatability.
3. Validate element quality before heavy solver runs.

---

## 🔀 Comparison chart: Mesh tools for general workflows

| Tool | Primary style | Open-source | Strength | Best with |
|---|---|---|---|---|
| Gmsh | CAD + meshing + scripts | ✅ | Balanced geometry+mesh workflow | Open-source CAD-to-mesh chains |
| FreeCAD | CAD + mesh plugins | ✅ | Strong parametric modeling | Teams wanting full CAD modeling + mesh handoff |
| Salome | Multi-module CAE platform | ✅ | CAD cleanup + solver modules | Full preprocessor + meshing in one stack |
| Netgen | Fast robust tetrahedral mesher | ✅ | Lightweight/unstructured speed | Simple fallback meshing |
| TetGen | Delaunay tetrahedral engine | ✅ | High-quality 3D tet output | FEM-heavy tetra pipelines |
| snappyHexMesh | OpenFOAM surface-to-hex tool | ✅ | CFD-specific hex-dominant path | Complex external flows with OpenFOAM |
| blockMesh | Structured OpenFOAM mesher | ✅ | Deterministic block topology | Canonical/simple domains |

---

## 🔗 Related notes

- [[Mesh Generation]]
- [[Mesh Generation Software]]
- [[CFD]]
- [[OpenFOAM]]
- [[snappyHexMesh]]
- [[blockMesh]]
- [[Netgen]]
- [[TetGen]]

## 📚 Further reading

- Gmsh tutorials and `.geo` reference examples
- Geometry-to-mesh conversion guides
- Interoperability with OpenFOAM and FEM toolchains
