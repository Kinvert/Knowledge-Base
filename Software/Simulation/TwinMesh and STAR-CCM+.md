---
title: TwinMesh and STAR-CCM+
aliases:
  - TwinMesh
  - TwinMesh plus STAR-CCM+
---
# TwinMesh and STAR-CCM+

## Overview

TwinMesh is a Siemens-focused approach for rotary positive-displacement (PD) machinery CFD workflows when combined with STAR-CCM+. It targets clearance-sensitive machines such as scroll, screw, vane, and gear pump geometries.

---

## Core concepts

- Emphasizes mesh and motion handling for rotary PD internals.
- Uses Siemens workflow integration rather than being a generic standalone solver family.
- Often chosen when standard rotating-fluid setups are difficult for general CFD.

---

## Pros

- Reduces friction for difficult rotary PD internal-gap flow setups.
- Adds practical value for leakage, recirculation, and pulsation analyses.
- Good for teams already standardized on Simcenter toolchain.

## Cons

- Not a general-purpose replacement for all pump CFD.
- Best ROI is concentrated in rotary PD programs.
- Requires Siemens ecosystem knowledge and maintenance.

---

## Use cases

- Rotary positive-displacement pump internals with small clearances and leakage paths.
- Scroll, screw, vane, gear, and vacuum pump geometry families.
- Campaigns where standard rotating setups are unstable or fragile.

---

## Comparison table

| Option | Target domain | Geometry/motion handling | Solver dependence | Ideal use |
|---|---|---|---|---|
| TwinMesh + STAR-CCM+ | Rotary positive-displacement pumps | High for clearance and rotating-contact geometry | High (Siemens stack) | Internal detail on scroll/screw/vane/gear/vacuum machines |
| Simcenter STAR-CCM+ | Broad rotodynamic machinery | High | High | Wide industrial rotating-machinery portfolio |
| Ansys CFX | Broad turbomachinery CFD | High | High | General centrifugal/compressor/centrifugal pump campaigns |
| SimScale | Cloud CFD + templates | Medium | Medium | Team-based template studies with lower local ops burden |
| PIPE-FLO | System/network simulation | Low | Low | Pump network performance rather than internal rotor flow |
| OpenFOAM | Custom simulation framework | Depends on user setup | Medium-High | Research and custom solver experimentation |

---

## Related notes

- [[Simcenter STAR-CCM+]]
- [[SimScale]]
- [[Ansys CFX]]

## External resources

- https://xcelerator.siemens.com/global/en/all-offerings/products/twinmesh----reliable--cfd-simulation-for-rotary-pd-machines.html
- Siemens commercial solution; source code is proprietary.
