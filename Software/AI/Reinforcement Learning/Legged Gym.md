# Legged Gym

**Legged Gym** is an open-source reinforcement learning framework for training locomotion policies on legged robots. Developed by ETH Zurich's Robotic Systems Lab, it's built on [[Isaac Gym]] and uses [[RSL-RL]] for training. Legged Gym has become the de facto starting point for quadruped and humanoid locomotion research, with proven [[Sim2Real]] transfer to robots like ANYmal, Unitree Go1/Go2, and Boston Dynamics Spot.

---

## Overview

Legged Gym provides GPU-accelerated parallel simulation for training locomotion controllers. It features a config-driven architecture where environments and training hyperparameters are defined in Python classes. The base `legged_robot` environment implements rough terrain locomotion, which can be extended for specific robots or tasks.

GitHub: https://github.com/leggedrobotics/legged_gym

---

## Core Concepts

- **LeggedRobotCfg**: Environment configuration (terrain, rewards, observations, domain randomization)
- **LeggedRobotCfgPPO**: Training configuration (PPO hyperparameters, network architecture)
- **Rough Terrain Task**: Default environment with procedural terrain generation
- **URDF/MJCF Loading**: Import robot models from standard formats
- **Privileged Information**: Teacher policies can access ground-truth state for distillation

---

## Architecture

```
legged_gym/
├── envs/
│   ├── base/
│   │   └── legged_robot.py      # Base environment
│   └── anymal_c/
│       └── anymal_c_config.py   # Robot-specific config
├── scripts/
│   ├── train.py                 # Training entry point
│   └── play.py                  # Visualization/evaluation
└── resources/
    └── robots/                  # URDF files
```

---

## Comparison: Locomotion Frameworks

| Framework | Base Simulator | Robots Supported | Sim2Real Proven | Maintained |
|-----------|---------------|------------------|-----------------|------------|
| **Legged Gym** | Isaac Gym | ANYmal, Unitree, Spot, custom | Yes | Community |
| **Isaac Lab** | Isaac Sim | Same + humanoids | Yes | NVIDIA |
| **Walk-These-Ways** | Isaac Gym | Unitree Go1 | Yes | MIT |
| **Humanoid-Gym** | Isaac Gym | Humanoids | Yes | Community |
| **MuJoCo Playground** | MuJoCo/MJX | Various | Limited | DeepMind |

---

## Supported Robots

| Robot | Manufacturer | Type | Notes |
|-------|-------------|------|-------|
| ANYmal B/C/D | ANYbotics | Quadruped | Primary development platform |
| Unitree A1/Go1/Go2 | Unitree | Quadruped | Popular low-cost option |
| Spot | Boston Dynamics | Quadruped | Via community configs |
| Mini Cheetah | MIT | Quadruped | Research platform |
| Custom URDF | Any | Any | Import your own |

---

## Key Features

- **Parallel Training**: Thousands of robots training simultaneously on GPU
- **Procedural Terrain**: Stairs, slopes, rough ground, stepping stones
- **Domain Randomization**: Mass, friction, motor strength, observation noise
- **Reward Shaping**: Configurable reward terms for velocity tracking, smoothness, energy
- **Teacher-Student**: Train privileged teacher, distill to deployable student

---

## Strengths

- Battle-tested sim2real on real quadrupeds
- Clean, readable codebase
- Strong community and research adoption
- Direct path to [[RSL-RL]] and [[Isaac Lab]]
- Extensive documentation and examples

---

## Weaknesses

- Isaac Gym is deprecated (transitioning to Isaac Lab)
- Linux + NVIDIA GPU required
- Less suited for manipulation tasks
- Config system can be verbose for complex setups

---

## Migration Path

Legged Gym → **Isaac Lab** is the recommended upgrade path. Isaac Lab provides:
- Modern Isaac Sim backend (better rendering, sensors)
- Active NVIDIA support
- Broader task coverage (manipulation, navigation)
- Same RSL-RL integration

---

## Related Notes

- [[Isaac Gym]] (Underlying simulator)
- [[Isaac Lab]] (Modern successor)
- [[RSL-RL]] (Training library)
- [[PPO]] (Algorithm used)
- [[Sim2Real]] (End goal)
- [[Domain Randomization]] (Key technique)
- [[PufferLib]] (Alternative training infrastructure)
- [[Reinforcement Learning]]

---

## External Resources

- [Legged Gym GitHub](https://github.com/leggedrobotics/legged_gym)
- [RSL-RL Documentation](https://leggedrobotics.github.io/rsl_rl/)
- [ETH RSL Research](https://rsl.ethz.ch/research/researchtopics/rl-robotics.html)
- [Walk-These-Ways (MIT)](https://github.com/Improbable-AI/walk-these-ways)
- [Isaac Lab Migration Guide](https://isaac-sim.github.io/IsaacLab/)

---
