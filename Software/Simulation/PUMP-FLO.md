---
title: PUMP-FLO
tags:
  - Pump Selection
  - Pump Sizing
---
# PUMP-FLO

## Overview

PUMP-FLO is an online pump-selection environment connected to manufacturer and distributor data. It supports fast shortlist and sizing workflows before detailed simulation work.

---

## Core concepts

- Data-driven pump matching from operating requirements.
- Uses manufacturer-specific performance and fit metadata.
- Supports early-stage engineering and procurement communication.

---

## Pros

- Very fast at generating candidate lists.
- Useful for communication between sales, procurement, and design.
- Lower effort than building full simulation campaigns for early preselection.

## Cons

- Limited as a final performance validation method.
- Coverage and quality depend on catalog availability.
- Needs downstream validation and network-level checks.

---

## Use cases

- Pre-selection from catalog data during early procurement discussions.
- Fast narrowing of candidates before detailed CFD or system simulation.
- Supporting rapid engineering alignment between design and purchasing.

---

## Comparison table

| Tool | Core output | Input requirement | Best stage |
|---|---|---|---|
| PUMP-FLO | Candidate shortlists | Moderate (requirements and constraints) | Front-end pump selection |
| PIPE-FLO | Network behavior | Topology and operating data | Later operational/system sizing |
| AFT Fathom | System simulation with transients | Network + pump curve sets | Detailed hydraulic analysis |
| Ansys CFX | Internal CFD performance maps | Geometry + mesh + solver setup | Final high-fidelity validation |
| OpenFOAM | Full solver-driven validation | Detailed case and script setup | Deep customization |
| TURBOdesign Suite | Geometry synthesis | Performance targets + constraints | Pump concept generation |

---

## Related notes

- [[Ansys CFX]]
- [[PIPE-FLO]]
- [[AFT Fathom]]

## External resources

- https://pump-flo.com/
- Proprietary pump-data platform; no public source-code repository.
