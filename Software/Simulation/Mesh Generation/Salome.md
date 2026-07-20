# Salome

**Salome** is an open-source, modular CAE platform with integrated CAD, meshing, and simulation-preparation capabilities.

---

## 🧠 Overview

Salome is often used when a workflow needs:

- CAD import (STEP, IGES, BREP),
- geometry cleanup and topology checks,
- and solver-ready mesh export without switching platforms.

Its workflow is typically module-based and extends well into multiphysics preprocessing where geometry + mesh + case organization are tightly coupled.

---

## 🧰 Core concepts

- CAD modeling and geometry repair modules.
- Meshing module with configurable algorithms and size controls.
- Post-processing and visualization modules for intermediate validation.
- Platform-wide project organization for large campaigns.

It is a practical fit for teams where geometry handling is the bottleneck.

---

## ✅ Why teams use it

- integrated workflow from CAD to mesh,
- strong support for industrial model formats,
- reproducible exports for engineering solvers and HPC setups.

Common use cases:

- mechanical/CFD pre-processing in mixed-domain projects,
- multi-step geometry cleanup before OpenFOAM/Salome-compatible export,
- teams needing GUI-driven yet scriptable preprocessor chains.

---

## ⚠️ Common caveats

- can feel heavyweight for tiny studies,
- interface is larger than single-purpose mesh tools,
- large models benefit from strict naming/metadata conventions.

Use strategy:

1. Keep all geometry naming consistent at import.
2. Apply cleanup early, then freeze shape.
3. Meshing and solver handoff should be version-controlled.

---

## 🔀 Comparison chart

| Tool | Focus | Geometry support | Mesh strategy | Best use |
|---|---|---|---|---|
| Salome | Full CAE preprocessing | Strong CAD (STEP/IGES) | Structured/unstructured module-driven | Complex geometry preprocessing |
| Gmsh | Lightweight CAD+mesh scripting | Good CAD/script pipeline | General-purpose mesh controls | Reproducible geometry meshing | 
| FreeCAD | CAD-first parametric stack | Strong native CAD modeling | Basic-to-moderate meshing | Parametric CAD + quick export |
| Netgen | Fast unstructured tetrahedral | Mesh-focused geometry intake | Tetra-heavy automation | Fast tetra fallback |
| OpenFOAM snappyHexMesh | CFD surface-to-volume workflow | STL/tri surfaces | Hex-dominant + layers | CFD with OpenFOAM stacks |
| blockMesh | Structured block meshing | Dictionary-driven geometry | Structured hex | Canonical base meshes |

## 🔗 Related notes

- [[Mesh Generation]]
- [[Mesh Generation Software]]
- [[CFD]]
- [[OpenFOAM]]
- [[snappyHexMesh]]
- [[Gmsh]]
- [[Netgen]]

## 📚 Further reading

- Salome platform modules and geometry cleanup workflows
- CAD format conversion and meshing strategies
- Open-source CAE case setup patterns
