---
title: AFT Fathom
tags:
  - Hydraulic Simulation
  - Pump Systems
---
# AFT Fathom

## Overview

AFT Fathom is a hydraulic-system simulation tool for incompressible flow networks with pump and valve models, including centrifugal and positive-displacement equipment models.

---

## Core concepts

- Component-level network equations (head, flow, power, pressure losses).
- Scenario and control handling for transient and steady operating conditions.
- Uses pump curves and component data as primary physical inputs.

---

## Pros

- Very effective for system-level pump network decisions.
- Supports control strategy exploration and operational sensitivity.
- Fast to evaluate topology and operating-point changes compared with full CFD.

## Cons

- Does not replace 3D internal machine CFD for detailed blade phenomena.
- Quality depends on input data and pump curve completeness.
- Model assumptions should be reviewed for extreme transient events.

---

## Use cases

- System-level hydraulic sizing before geometry-level redesign.
- Network and control strategy comparison across operating modes.
- Early transient behavior checks for plant-level pump installations.

---

## Comparison table

| Tool | Primary analysis level | Typical input | Strength |
|---|---|---|---|
| AFT Fathom | Component/network | Pump curves, topology, control logic | System-level hydraulics and pump/valve interactions |
| PIPE-FLO | Hydraulic + operating network | Similar plant topology inputs | Strong operational and piping network workflow |
| PUMP-FLO | Selection-focused | Manufacturer catalog and sizing data | Fast front-end candidate narrowing |
| SimScale | 3D CFD | Detailed geometry + meshing | Internal flow and local loss resolution |
| Ansys CFX | 3D CFD | Geometry + meshes + solver settings | High-fidelity internal-machine validation |
| OpenFOAM | 3D CFD framework | Case setup + scripts | Custom research and optimization loops |

---

## Related notes

- [[PIPE-FLO]]
- [[PUMP-FLO]]
- [[Ansys CFX]]

## External resources

- https://docs.aft.com/fathom/OverviewofAFTFathom.html
- https://docs.aft.com/fathom/PumpPropertiesWindow.html
