---
title: CAESES
aliases:
  - CAESES Pump Design
  - CAESES
---
# CAESES

## Overview

CAESES is a parametric geometry and variation platform often used for pump design studies. Its strength in this space is controlled model generation plus variation automation feeding CFD-based optimization workflows.

---

## Core concepts

- Geometry is built from reusable parameters and rules.
- Large variant sets can be scripted and exported consistently.
- Works best when connected to external solver and optimization engines.

---

## Pros

- Strong geometry automation for pump families.
- Good support for design-space exploration and variation management.
- Fits organizations that want explicit geometry-to-performance pipelines.

## Cons

- Not a full closed-loop analysis stack by itself.
- Solver validation and error handling remain external.
- Requires workflow engineering around downstream CFD and KPI logic.

---

## Use cases

- Parametric sweeps across impeller, splitter, and passage design knobs.
- Generating stable geometry libraries for repeated CFD campaigns.
- Reducing manual CAD repetition before campaign closure.

---

## Comparison table

| Tool | Geometry generation depth | Design automation | Solver dependence | Best use case | Typical drawback |
|---|---|---|---|---|---|
| CAESES | Very high for pump geometry parametrization | High | External solver required | Pump optimization loops with many design variables | Requires separate CFD evaluation stack |
| TURBOdesign Suite | High for inverse pump synthesis | High | Export-heavy | Rapid target-driven geometry generation | Less control over arbitrary scripted constraints |
| Ansys BladeModeler | High for blade-specific parametrics | Medium-High | Ansys flow stack | Detailed blade family design | More focused on blade families than broad variation frameworks |
| AxSTREAM | Medium-High, multi-module design system | High | Internal modules for many stages | Full-cycle turbomachinery programs | Broader and heavier governance |
| SimFlow | Medium for geometry prep | Medium | Internal CFD workflows | Small teams needing practical pump-case workflow | Not primarily a geometry-automation core |
| OpenFOAM | Indirect via mesh/pre-process pipelines | Low-Medium in CAE context | Open solver infrastructure | Fully custom optimization scripts | Front-loaded development effort |

---

## Related notes

- [[TURBOdesign Suite]]
- [[Ansys CFX]]
- [[OpenFOAM]]

## External resources

- https://www.caeses.com/products/caeses/geometry-modeling-for-variation/pumps/
- https://www.caeses.com/products/caeses/
- Commercial platform; source code is proprietary and not publicly hosted.
