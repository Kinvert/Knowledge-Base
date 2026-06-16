---
title: SimScale
aliases:
  - SimScale Turbomachinery
---
# SimScale

## Overview

SimScale is a cloud platform for CFD/FEA workflows with dedicated templates including pump and turbomachinery flows. The turbomachinery page is commonly used for cases like pump performance, cavitation, and NPSH-related studies.

---

## Core concepts

- Project-centric cloud workspaces with web-based pre/post.
- Supports rotating machinery and pump templates for faster project startup.
- Team collaboration is a key design principle in its workflow.

---

## Pros

- Fast deployment without local CAE infrastructure.
- Easy collaboration across teams and revisions.
- Useful for quick NPSH, head-loss, and rough cavitation scenario testing.

## Cons

- Cost model and throughput depend on compute usage patterns.
- Deep low-level solver customization is less immediate than open source stacks.
- For very high complexity, process governance can become cloud-admin heavy.

---

## Use cases

- Useful when teams want a clean path from CAD to repeatable cloud studies.
- Commonly sits before expensive full-scale enterprise solver campaigns.
- Good fit for distributed review cycles and template reuse.

---

## Comparison table

| Tool | Deployment model | Pump specialization | Workflow speed | Best for |
|---|---|---|---|---|
| SimScale | Cloud web platform | Medium-High (template-driven) | Fast for standardized studies | Collaborative cloud-first pump screening |
| Ansys CFX | Enterprise on-prem/hybrid | High | Medium | High-fidelity industrial validation |
| Simcenter STAR-CCM+ | Enterprise desktop/cloud mix | High | Medium | Large rotating-machinery industrial programs |
| OpenFOAM | On-prem / self-hosted | Depends on case setup | Variable | Customizable research pipelines |
| SimFlow | Desktop GUI platform | Medium-High | High | Teams preferring guided interfaces |
| TwinMesh and STAR-CCM+ | Specialized rotary PD CFD | Niche-high | Medium | Rotary positive-displacement pump detail |

---

## Related notes

- [[Ansys CFX]]
- [[Simcenter STAR-CCM+]]
- [[OpenFOAM]]

## External resources

- https://www.simscale.com/simulations/turbomachinery/
