# Bézier Curves and Surfaces

**Bézier** curves/surfaces are polynomial spline primitives commonly used in graphics, font systems, CAD UI curves, and CAD kernels as a building block.

You probably know them from vector editors and UI paths; in math terms they are parameterized polynomials using Bernstein basis functions and a fixed control polygon.

---

## Overview

Characteristics:

- Built from a small set of control points + polynomial degree
- Interpolates end points
- Interior shape is pulled by control points (not interpolated directly except ends)
- Smoothness is guaranteed by continuity settings between segments, not by automatic global continuity

In CAD/CAM tools, Bézier is often used as:

- primitive segment inside broader spline systems
- readable geometry representation in fonts and design scripts
- UI-level segment for shape editing before downstream conversion

---

## Strengths and limitations

### Strengths

- Intuitive handle/editing model
- Numerically simple and stable for many interactive modeling tasks
- Easy to evaluate and tessellate into render/mesh pipelines
- Exact for low-order geometric primitives in constrained cases

### Limits

- Global control: moving a point can affect the whole curve segment
- Not as compact as NURBS for long complex surfaces needing varying parameter behavior
- No rational weights (so circles/arcs are approximated unless higher degree or exact rational extension)
- Usually not as interchange-friendly for large production CAD surfacing as full NURBS systems

---

## How Bézier relates to NURBS (important mapping)

From a pipeline perspective:

- A rational cubic Bézier can be represented as a constrained NURBS segment
- Long “designer curves” are often assembled from multiple Bézier pieces with continuity constraints
- For large multi-curve networks, NURBS/B-splines are usually preferred for local control and edit stability

Practical rule:

- Use Bézier when the user-facing control model is simple and local segment count is low
- Use NURBS when you need long smooth class surfaces with stronger continuity and rational/weight control at scale

---

## Comparison with common representations (5+ items)

| Representation | Local control | Continuity behavior | Rational exact shapes | Typical use case | Edit predictability |
|---|---|---|---|---|---|
| [[Bezier]] | Moderate | Per segment, continuity managed at joins | No (non-rational) | Vector/animation curves, font outlines, interactive editing | High for short segments |
| [[NURBS]] | High | High (knot + degree controlled) | Yes | Parametric CAD, airframe/automotive surfacing, CAM workflows | High once knots are managed |
| B-Spline | High | High | Optional via NURBS variant | Data fitting, smooth transitions over many points | High |
| Catmull-Rom spline | Low-Moderate | Moderate | No | Interpolation through points, motion paths, simple animation | High |
| Subdivision surface | Indirect control | Implicit at mesh level | No | Character/hero assets, organic modeling | Moderate |
| STL mesh | Low | None | No | Fast visualization, 3D print prep, CFD meshing ingress | Low |

---

## Practical use in CAD-like stacks

When you’re scripting or modeling:

- Keep curves editable at high level where possible (avoid immediate triangulation)
- For export to neutral formats, validate that control segments are preserved (or re-interpolated as expected)
- For downstream CFD/FEA, apply robust remesh checks after mesh export
- If using in surfacing-heavy stacks, lock endpoint tangents before subdividing or reconstructing segment chains

---

## Related notes

- [[NURBS]]
- [[STEP]]
- [[OpenVSP CFD Workflow]]
- [[Mesh Generation]]
- [[Blender]]

## External resources

- [Wikipedia — Bézier curve](https://en.wikipedia.org/wiki/B%C3%A9zier_curve)
- [Wikipedia — B-spline](https://en.wikipedia.org/wiki/B-spline)
- CAD/graphics tutorials from target toolchain docs (Rhino, FreeCAD, Blender, OpenCascade)
