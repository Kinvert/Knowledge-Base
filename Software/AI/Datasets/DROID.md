# DROID

**DROID** (Distributed Robot Interaction Dataset) is a large-scale robot manipulation dataset with 76K demonstration trajectories across 564 diverse scenes. Unlike datasets collected in controlled lab settings, DROID captures real-world diversity—kitchens, offices, homes, workshops—making it valuable for training policies that generalize beyond laboratory environments.

---

## Overview

DROID was collected over 12 months by 50 data collectors across 13 institutions in North America, Asia, and Europe. The dataset emphasizes scene diversity over task complexity, capturing everyday manipulation in varied real-world settings. Policies co-trained with DROID outperform those trained on [[Open X-Embodiment]] alone.

Website: https://droid-dataset.github.io/

---

## Dataset Statistics

| Metric | Value |
|--------|-------|
| **Trajectories** | 76,000 |
| **Scenes** | 564 |
| **Tasks** | 86 |
| **Data Collectors** | 50 |
| **Institutions** | 13 |
| **Collection Time** | 12 months |
| **Interaction Hours** | 350+ |

---

## Key Differentiator: Scene Diversity

| Dataset | Trajectories | Scenes | Scenes/Traj |
|---------|--------------|--------|-------------|
| **DROID** | 76K | 564 | High diversity |
| Open X-Embodiment | 1M+ | ~300 | Low diversity |
| Bridge | 60K | ~10 | Very low |
| RT-1 | 130K | ~5 | Very low |

DROID prioritizes varied environments over raw trajectory count.

---

## Example Tasks

| Category | Tasks |
|----------|-------|
| **Kitchen** | Coffee making, dish handling, cabinet interaction |
| **Office** | Drawer organization, paper handling |
| **Workshop** | Tool manipulation, assembly |
| **Home** | Object rearrangement, cleaning |

---

## Hardware Setup

- **Robot**: Franka Emika Panda arm
- **Gripper**: Franka parallel gripper
- **Cameras**: Wrist-mounted + external views
- **Teleoperation**: SpaceMouse or VR controller
- **Portable**: Setup designed for wheeling into new environments

---

## Results

Policies co-trained with DROID show:
- **Better generalization** to new scenes than single-dataset training
- **Improved performance** vs Open X-Embodiment alone
- **Robust manipulation** across diverse backgrounds and lighting

---

## Comparison: Manipulation Datasets

| Dataset | Focus | Robot | Real-World Diversity |
|---------|-------|-------|---------------------|
| **DROID** | Scene diversity | Franka | Very high (564 scenes) |
| [[Open X-Embodiment]] | Robot diversity | Many | Medium (~300 scenes) |
| Bridge | Kitchen tasks | WidowX | Low |
| RoboTurk | Crowd-sourced | Sawyer | Medium |
| RoboNet | Video prediction | Multiple | Medium |

---

## Accessing the Data

- Full dataset and training code open-sourced
- Hardware setup guide for reproduction
- Compatible with diffusion policy training

```python
# Example: Load DROID for training
from droid import DROIDDataset
dataset = DROIDDataset(path="/path/to/droid")
```

---

## Strengths

- Highest scene diversity of any manipulation dataset
- Real-world environments (not just labs)
- Proven generalization benefits
- Open source (data + code + hardware guide)
- Multi-institution validation

---

## Weaknesses

- Single robot type (Franka)
- Smaller than OXE in trajectory count
- Manipulation only (no locomotion)
- Requires significant storage (~TB scale)

---

## Related Notes

- [[Open X-Embodiment]] (Complementary dataset)
- [[Imitation Learning]] (Primary use case)
- [[Sim2Real]] (DROID is real-world data)
- [[Reinforcement Learning]]
- [[Domain Randomization]] (DROID provides natural variation)

---

## External Resources

- [Project Website](https://droid-dataset.github.io/)
- [Paper (RSS 2024)](https://arxiv.org/abs/2403.12945)
- [GitHub Repository](https://github.com/droid-dataset/droid)
- [Hardware Setup Guide](https://droid-dataset.github.io/hardware)

---
