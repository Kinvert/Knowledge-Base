---
title: OpenFOAM
tags:
  - CFD
  - Open Source
---
# OpenFOAM

## Overview

OpenFOAM is an open-source C++ CFD toolkit with a modular, case-based structure. It is popular in pump and turbomachinery work when teams want full control over meshing, solvers, and automation.

---

## Core concepts

- Finite-volume discretization with broad open-source solver libraries.
- Case directories (`0`, `constant`, `system`) define reproducible workflows.
- Custom scripting enables large campaign automation and CI integration.

---

## Pros

- No software license cost and no single-vendor lock-in.
- Highly customizable for custom geometry, turbulence, and automation patterns.
- Strong ecosystem for academic and applied R&D collaboration.

## Cons

- Higher engineering effort for robust industrial deployment.
- Requires significant setup, maintenance, and validation discipline.
- Not as turnkey for non-experts as some commercial tools.

---

## Use cases

- Custom pump optimization pipelines.
- Research and novel solver extension work.
- Organizations wanting full stack ownership and flexible automation.

---

## Comparison table

| Tool | Access model | Typical strengths | Best fit |
|---|---|---|---|
| OpenFOAM | Open-source | Maximum customization and extensibility | Teams owning the full simulation stack |
| Ansys CFX | Commercial | Enterprise robustness and support | Industrial high-fidelity validation |
| Simcenter STAR-CCM+ | Commercial | Large-scale industrial rotating studies | Regulated enterprise pipelines |
| SimScale | Cloud platform | Easy collaboration and managed compute | Distributed teams needing low setup overhead |
| SimFlow | GUI-driven practical CFD | Speedy practical pump projects | Smaller teams with lower scripting budget |
| AFT Fathom | System-level hydraulics | Plant network and transient behavior |

---

## Related notes

- [[Ansys CFX]]
- [[SimScale]]
- [[Simcenter STAR-CCM+]]

## Further reading

- https://www.openfoam.com/
- https://openfoam.org/
