# Marble

**Marble** is a generative world model from World Labs (founded by Fei-Fei Li) that creates 3D environments from text prompts, images, or video. Unlike traditional 3D modeling that takes weeks, Marble generates simulation-ready environments in minutes. Its integration with [[Isaac Sim]] enables rapid creation of diverse training scenarios for robotics, reducing environment curation time by over 90%.

---

## Overview

Marble represents a shift from hand-crafted to AI-generated simulation environments. Given a text description like "modern kitchen with marble countertops" or a reference photo, Marble generates a persistent, editable, downloadable 3D world with geometry, lighting, and collision meshes ready for physics simulation.

- Beta: September 2025
- Commercial launch: November 2025
- Website: https://www.worldlabs.ai/

---

## Core Concepts

- **Text-to-3D**: Generate worlds from natural language descriptions
- **Image-to-3D**: Convert photos/renders into 3D environments
- **Persistent Worlds**: Unlike streaming generation, worlds are downloadable
- **Collision Meshes**: Exported scenes include physics-ready geometry
- **Gaussian Splats**: Alternative export format for neural rendering
- **Isaac Sim Integration**: Direct import into NVIDIA's robotics simulator

---

## Comparison: 3D Generation Methods

| Method | Speed | Diversity | Physics-Ready | Quality |
|--------|-------|-----------|---------------|---------|
| **Marble** | Minutes | High (generative) | Yes | Photorealistic |
| Manual modeling | Weeks | Low | Manual setup | Variable |
| [[ProcTHOR]] | Seconds | Medium (rules) | Yes | Game-quality |
| [[Infinigen]] | Hours | High (procedural) | Export needed | Photorealistic |
| 3D scanning | Hours | Low (real only) | Cleanup needed | Photorealistic |

---

## NVIDIA Partnership

December 2025: NVIDIA integrated Marble with [[Isaac Sim]], enabling:
- Text prompt → simulation-ready environment in hours (vs weeks)
- Automatic collision mesh generation
- Direct USD export for Isaac Lab
- Thousands of environment variations for [[Domain Randomization]]

---

## Workflow: Robotics Use Case

1. **Describe**: "cluttered kitchen with dishes in sink, open cabinet"
2. **Generate**: Marble creates 3D environment
3. **Export**: Download USD with collision meshes
4. **Simulate**: Import into [[Isaac Sim]] / [[Isaac Lab]]
5. **Train**: Run RL with procedural variations
6. **Deploy**: [[Sim2Real]] transfer to physical robot

---

## Key Features

- **Rapid Iteration**: Test new scenarios same-day
- **Photorealistic**: RTX-quality rendering
- **Editable**: Modify, expand, combine generated worlds
- **Multiple Exports**: USD, Gaussian splats, mesh, video
- **API Access**: Programmatic generation for dataset creation

---

## Use Cases

- Generating diverse kitchen/warehouse environments for robot training
- Rapid prototyping of simulation scenarios
- Creating [[Domain Randomization]] variations at scale
- VR/AR environment creation
- Film/game pre-visualization

---

## Strengths

- Dramatically faster than manual 3D modeling
- High visual quality
- Direct Isaac Sim integration
- Freemium access available
- Backed by AI pioneer (Fei-Fei Li)

---

## Weaknesses

- Generated geometry may need cleanup for precise physics
- Dependent on model's training data for accuracy
- Commercial pricing for high-volume use
- Newer tool, less battle-tested than procedural alternatives

---

## Related Notes

- [[Isaac Sim]] (Primary integration target)
- [[Isaac Lab]] (Training platform)
- [[ProcTHOR]] (Rule-based alternative)
- [[Infinigen]] (Procedural alternative)
- [[Domain Randomization]] (Use case)
- [[Sim2Real]] (End goal)
- [[OpenUSD]] (Export format)

---

## External Resources

- [World Labs Website](https://www.worldlabs.ai/)
- [Marble Announcement](https://www.worldlabs.ai/blog/marble-world-model)
- [NVIDIA Isaac Sim Integration Blog](https://developer.nvidia.com/blog/simulate-robotic-environments-faster-with-nvidia-isaac-sim-and-world-labs-marble/)
- [Robotics Case Study](https://www.worldlabs.ai/case-studies/1-robotics)

---
