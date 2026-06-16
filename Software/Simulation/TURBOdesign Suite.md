---
title: TURBOdesign Suite
tags:
  - Pump Design
  - Inverse Design
---
# TURBOdesign Suite

## Overview

TURBOdesign Suite is an inverse-design pump platform for centrifugal, mixed-flow, and axial machines. It is typically used to generate geometry from performance targets and then pass candidates into dedicated flow solvers for verification.

---

## Core concepts

- Target-driven synthesis: geometry responds to duty-point or head/flow goals.
- Workflow focuses on geometry generation and campaign handoff, not standalone 3D validation.
- Designed around production use for many variants and fast model evolution.

---

## Pros

- Fast geometry creation when performance requirements are stable.
- Useful for repetitive pump-family concept exploration.
- Strong when teams treat CFD as validation rather than starting point.

## Cons

- Needs downstream solver validation for trustable final performance.
- Can be constrained by assumptions in inverse workflow templates.
- Governance needed to avoid overfitting to one operating envelope.

---

## Use cases

- Inverse geometry synthesis for centrifugal, mixed-flow, and axial pump families.
- Concept-to-candidate generation from pressure/flow targets.
- Early pump campaigns before detailed 3D CFD verification.

---

## Comparison table

| Tool | Primary design direction | Strength | Best fit | Where weaker |
|---|---|---|---|---|
| TURBOdesign Suite | Inverse design for pump geometry | Quick geometry synthesis | Early-to-mid pump development | Full 3D flow validation depth |
| CAESES | Parametric variation + automation | Broad geometry design-space exploration | Multi-DOF optimization sweeps | Solver loop setup still external |
| Ansys BladeModeler | Blade geometry craftsmanship | Detailed blade refinement | Precision geometry definition | Not as direct for target-driven synthesis |
| SimScale | Cloud CFD + setup templates | Quick validated campaign execution | Team collaboration and moderate-size studies | Geometry generation is not its primary differentiator |
| AxSTREAM | Integrated design-to-system workflow | End-to-end turbomachinery stack | Large programs with long design chains | Heavy deployment and process overhead |
| OpenFOAM | Open simulation framework | Custom automation and high flexibility | Research and scripted optimization | Requires strong software engineering |

---

## Related notes

- [[CAESES]]
- [[Ansys BladeModeler]]
- [[OpenFOAM]]

## External resources

- https://www.adtechnology.com/applications/pump-design
