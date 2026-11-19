# NURBS

**NURBS** = *Non-Uniform Rational B-Spline*.
In geometry software, it is a compact spline representation for curves and surfaces that gives local control, exact conics, and smoothness control.

In practice, NURBS is a standard “design-grade” representation for CAD bodies, automotive/aircraft skin continuity, and reverse-engineering workflows.

---

## Overview

NURBS geometry is defined by:

- degree (order - 1)
- control points
- knot vector
- optional weights (this is the “R” in NURBS)

Why these matter:

- control points shape the curve/surface
- knots define where and how basis functions blend
- weights let you represent circles/arcs exactly with low-degree curves

Compared with simple interpolating splines, NURBS are usually better for manufacturing-like CAD geometry work where continuity and exactness matter.

---

## Where NURBS sits in CAD

NURBS is not usually user-visible as raw math in high-level CAD UI:

- you edit handles/cross-sections, and the kernel stores a NURBS/parametric form
- many export formats preserve parametric intent differently (`.step`, `.iges` behavior varies)
- downstream tools often convert to native B-rep and then to tessellated mesh for simulation/visualization

Common CAD contexts:

- wings/airfoils and fuselage-like surfaces
- freeform car/aircraft skins
- toolpath/curvature-based manufacturing prep
- sculpted Class-A surfaces where continuity is a design constraint

---

## Strengths vs limitations

### Strengths

- Smooth continuity control (`C0` positional and typically higher continuity options)
- Great for exact analytic geometry via weights (circles, cylinders, cones, spheres)
- Excellent local editing behavior for large curves/surfaces
- Good foundation for CAM and downstream meshing pipelines when preserved well

### Limits

- Higher complexity than low-order polygon or polyline models
- Requires proper knot/parameter management for predictable edits
- File translators can degrade precision or continuity in legacy toolchains
- Not always a good fit for purely discrete workflows (e.g., topology optimization fields)

---

## How people use it in a workflow

Typical practical pipeline:

1. Build or import geometry with NURBS-capable CAD
2. Keep NURBS during styling and surfacing stages
3. Export to neutral exchange (`STEP`) or mesh format depending on downstream solver
4. Validate geometry after transfer:
   - continuity
   - face orientation
   - edge gaps and near-closed seams

For CFD pipelines that eventually need `STL`/volume mesh, NURBS is usually converted to trimmed B-rep + triangulation, so your quality checks should happen after export.

---

## NURBS vs related curve/surface families (comparison)

| Representation | Exact conic support | Local control | Continuity control | Runtime complexity | Best fit |
|---|---|---|---|---|---|
| NURBS | Yes (with weights) | High | High (`C0/C1/C2` options) | High | Parametric CAD, automotive/aerospace surfacing |
| [[Bezier]] | No rational exactness by default | Moderate | Good but global for segment edits | Moderate | Font/UI curves, design handles, simple CAD patches |
| B-Spline | No rational weights in core variant | High | High | High | Data approximation and smooth interpolation |
| Polynomial spline (uniform) | No | Low-moderate | Moderate | Low | Quick engineering approximations |
| Mesh (STL) | No | Low | None | Low | Simulation/printing where topology only |
| NURBS-to-STEP/B-Rep chain | Dependent on translator | Medium | Medium | Varies | Exchange-heavy projects |

---

## Related notes

- [[Bezier]]
- [[STEP]]
- [[Parasolid]]
- [[Gmsh CFD Workflow]]
- [[OpenVSP CFD Workflow]]
- [[Mesh Generation]]

## External resources

- [Wikipedia — NURBS](https://en.wikipedia.org/wiki/Non-uniform_rational_B-spline) (background and math family relations)
- [NURBS book and Open CASCADE references (OpenCASCADE modeling docs)](https://dev.opencascade.org/doc/overview/html/index.html)
- CAD/geometry kernel docs for your target tool (Rhino/FreeCAD/SOLIDWORKS/Onshape/Blender)
