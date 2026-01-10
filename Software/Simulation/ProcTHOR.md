# ProcTHOR

**ProcTHOR** is a procedural generation framework for creating fully-interactive 3D house environments for embodied AI research. Developed by Allen Institute for AI (AI2), it builds on [[AI2-THOR]] to generate unlimited diverse houses with realistic layouts, furniture placement, and lighting. ProcTHOR won the NeurIPS 2022 Outstanding Paper Award and has become a standard benchmark for training generalizable robot policies.

---

## Overview

ProcTHOR solves a key bottleneck in embodied AI: the need for diverse training environments. Instead of hand-crafting scenes, ProcTHOR procedurally generates houses that are physically plausible, semantically realistic, and fully interactive. Models trained on ProcTHOR-generated environments achieve state-of-the-art results across multiple embodied AI benchmarks.

GitHub: https://github.com/allenai/procthor

---

## Core Concepts

- **Room Specification**: Define house requirements (e.g., 3 bedrooms, 2 baths, 1 kitchen)
- **Floorplan Generation**: Procedurally create diverse layouts meeting specs
- **Asset Library**: 108 object types, 1633 fully interactable instances
- **Material Randomization**: 40 wall colors, 122 wall textures, 55 floor materials
- **Lighting Variation**: Artificial lights + skybox simulation for time-of-day
- **Physics-Enabled**: All objects support interaction (pick, place, open, etc.)

---

## Comparison: Procedural Environment Generators

| Tool | Domain | Assets | Physics | Rendering | Integration |
|------|--------|--------|---------|-----------|-------------|
| **ProcTHOR** | Indoor houses | 1633 objects | AI2-THOR | Unity | AI2-THOR API |
| [[Infinigen]] | Nature + indoor | Procedural | Blender | Cycles | USD export |
| [[Marble]] | Any (from text) | Generated | Isaac Sim | RTX | Isaac Sim |
| [[Isaac Replicator]] | Industrial | SimReady | PhysX | RTX | Omniverse |
| Habitat-Sim | Indoor scans | Matterport | Bullet | Custom | Habitat API |

---

## Key Features

- **Unlimited Diversity**: Generate arbitrarily large datasets of unique houses
- **Semantic Realism**: Object placement follows real-world patterns
- **Full Interactivity**: Objects can be manipulated, opened, picked up
- **Configurable Complexity**: Control room count, object density, clutter
- **Benchmarking**: Standard evaluation on navigation, rearrangement, manipulation

---

## Research Results

Models trained on 10,000 ProcTHOR houses (RGB only, no explicit mapping, no human supervision) achieved SOTA on:
- Habitat 2022 ObjectNav Challenge
- AI2-THOR Rearrangement 2022 Challenge
- RoboTHOR Navigation Challenge
- Multiple arm manipulation benchmarks

---

## Use Cases

- Training navigation policies that generalize to new environments
- Rearrangement tasks (move objects to goal configurations)
- Object search and retrieval
- Instruction following in household settings
- [[Sim2Real]] transfer for mobile manipulators

---

## Strengths

- Massive scale (generate thousands of houses)
- Proven generalization to real environments
- Active maintenance by AI2
- Easy Python API (`pip install procthor`)
- Compatible with AI2-THOR ecosystem

---

## Weaknesses

- Unity-based (not directly compatible with Isaac Sim)
- Indoor focus (no outdoor/industrial environments)
- Asset library fixed (can't easily add custom objects)
- Rendering less photorealistic than Unreal/RTX

---

## Related Notes

- [[AI2-THOR]] (Base simulator)
- [[Infinigen]] (Alternative procedural generator)
- [[Sim2Real]] (End goal of training)
- [[Domain Randomization]] (Related technique)
- [[Isaac Lab]] (Alternative training platform)
- [[Reinforcement Learning]]

---

## External Resources

- [ProcTHOR Website](https://procthor.allenai.org/)
- [GitHub Repository](https://github.com/allenai/procthor)
- [NeurIPS 2022 Paper](https://arxiv.org/abs/2206.06994)
- [AI2-THOR Documentation](https://ai2thor.allenai.org/)

---
