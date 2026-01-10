# Infinigen

**Infinigen** is an open-source procedural generator for creating photorealistic 3D environments, developed by Princeton University. Built on [[Blender]], it can generate unlimited unique natural and indoor scenes—forests, mountains, underwater environments, and rooms—with perfect ground-truth labels for training computer vision and robotics models. Infinigen exports to USD for integration with [[Isaac Lab]] and other simulators.

---

## Overview

Unlike asset libraries with fixed 3D models, Infinigen generates everything procedurally using mathematical rules. This means infinite variety: no two trees, rocks, or furniture pieces are identical. The system produces photorealistic renders via Blender's Cycles engine and automatically generates dense annotations (depth, normals, segmentation, optical flow).

GitHub: https://github.com/princeton-vl/infinigen

---

## Core Concepts

- **Procedural Generation**: All geometry created from code, not stored assets
- **Infinite Variety**: Every scene element is uniquely generated
- **Ground Truth Labels**: Automatic depth, segmentation, flow, 3D annotations
- **USD Export**: Compatible with Isaac Lab, Omniverse, other USD pipelines
- **Nature + Indoor**: Terrain, vegetation, creatures, furniture, rooms

---

## Supported Environments

| Category | Examples |
|----------|----------|
| **Terrain** | Mountains, valleys, caves, cliffs |
| **Vegetation** | Trees, bushes, grass, flowers, coral |
| **Water** | Rivers, lakes, oceans, underwater |
| **Weather** | Rain, snow, fog, clouds |
| **Indoor** | Rooms, furniture, kitchens, offices |
| **Creatures** | Fish, insects, birds (procedural) |

---

## Comparison: Procedural Generators

| Tool | Strength | Rendering | USD Export | Physics |
|------|----------|-----------|------------|---------|
| **Infinigen** | Nature + indoor diversity | Blender Cycles | Yes | Via export |
| [[ProcTHOR]] | Interactive houses | Unity | No | AI2-THOR |
| [[Marble]] | Text-to-3D speed | Isaac Sim RTX | Native | PhysX |
| Habitat-Sim | Real scans | Custom | No | Bullet |

---

## Isaac Lab Integration

Infinigen scenes can be exported to USD and imported into [[Isaac Lab]] for robot training:
1. Generate diverse environments in Infinigen
2. Export as USD with collision meshes
3. Import into Isaac Lab scene
4. Train locomotion/manipulation policies with [[Domain Randomization]]

Research shows training on procedural Infinigen scenes improves sim2real generalization.

---

## Key Features

- **Photorealistic Output**: Production-quality renders
- **Automatic Annotation**: No manual labeling needed
- **Scalable**: Generate millions of images/scenes
- **Customizable**: Modify generation parameters or add new generators
- **Open Source**: Apache 2.0 license

---

## Use Cases

- Training vision models on diverse synthetic data
- [[Sim2Real]] for outdoor/nature robotics
- Generating cluttered scenes for manipulation training
- Computer vision benchmarks (depth, segmentation, flow)
- Film/VFX pre-visualization

---

## Strengths

- True infinite variety (procedural, not assets)
- Photorealistic quality
- Comprehensive ground truth
- USD export for modern pipelines
- Active Princeton research team

---

## Weaknesses

- Blender-based (slower than real-time engines)
- Generation can be compute-intensive
- Indoor generation less mature than outdoor
- No built-in physics simulation (export required)

---

## Related Notes

- [[Blender]] (Underlying 3D software)
- [[Isaac Lab]] (Training platform with USD import)
- [[ProcTHOR]] (Alternative for indoor)
- [[Domain Randomization]] (Related technique)
- [[Sim2Real]] (End goal)
- [[OpenUSD]] (Export format)

---

## External Resources

- [Infinigen GitHub](https://github.com/princeton-vl/infinigen)
- [Project Website](https://infinigen.org/)
- [CVPR 2023 Paper](https://arxiv.org/abs/2306.09310)
- [Isaac Lab Infinigen Tutorial](https://docs.isaacsim.omniverse.nvidia.com/)

---
