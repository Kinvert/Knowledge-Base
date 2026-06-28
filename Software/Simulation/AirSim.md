# AirSim

`AirSim` is an open-source simulator from Microsoft for autonomy experiments on vehicles (multirotors, fixed-wing, and cars), built on Unreal/Unity. It is used to build safe DRL and control pipelines before touching real hardware.

---

## Overview

- Built for AI research with APIs for perception and control in a single loop.
- Explicitly targeted at both aerial and ground vehicle use-cases.
- Useful for mixed hardware/software loops because it can run software-in-the-loop with common autopilot stacks and supports HIL paths for PX4.

The project README states support for drones/cars and APIs for data/control access across simulation modes.[^airsim-readme]

---

## UGV behavior model

AirSim’s ground-vehicle mode is typically used for:
- lane-following policy research
- obstacle-avoidance and perception-to-control loops
- sensor-conditioned policy evaluation (lidar/rgb/camera telemetry)

Common pattern:
1. Build world + vehicle + sensor configuration
2. Use `car` API to issue control commands and read state
3. Log rollouts for imitation/RL training
4. Validate in sim, then move policy to ROS2 or embedded controllers

---

## Comparison chart (UGV workflow)

| Criterion | AirSim | CARLA | LGSVL/SVL | Webots | Gazebo Garden | CoppeliaSim |
|---|---|---|---|---|---|---|
| Vehicle domain focus | multi-modal (drone + car) | autonomous driving specialist | autonomous driving specialist | broad mobile robot | broad robotics + ROS | broad robotics + scripting |
| Ground-vehicle fidelity | High-visual; API-driven | High-urban + benchmark suites | High visual + AD stack integration | Medium | Flexible modular world models | Plugin-driven scene-based |
| Control API | yes | yes (Python/C++ workflows) | Python API + ROS bridges | yes (via controllers and plugins) | yes (via transport + plugins) | yes (embedded scripts + remote API) |
| Simulator style | game-engine + API bridge | city-scale autonomous driving env | map/vehicle plugin stack | robotics platform | middleware-first robotics | robotics-first scene + component |
| RL onboarding | strong (scripted loops) | strong with scenario tools | moderate setup overhead | lightweight for quick loops | strongest with ROS ecosystems | custom but flexible |

---

## Pros and cons

### What it can do

| Capability | Status |
|---|---|
| Multimodal autonomy experiments | Good |
| Cross-platform development | Good |
| Ground-vehicle DRL workflows | Good |
| Direct PX4/Hardware workflow entry points | Good |

### What it can’t do well

| Limitation | Impact |
|---|---|
| Setup complexity is non-trivial | Higher onboarding cost than tiny mobile robot simulators |
| Full-stack AD stack can require custom glue | More engineering on top of default scenes |
| Resource overhead of UE/Unity stacks | Higher runtime footprint for large batched sweeps |

---

## Practical notes

- `AirSim` historically tracks both flight and ground use-cases, so check the branch and environment preset before committing benchmark settings.
- If your core objective is UGV-only, `CARLA` and `LGSVL` often give cleaner AD-specific tooling; if you need shared aero + UGV tooling in one stack, AirSim can stay.

---

## Related notes

- [[CARLA]]
- [[Gazebo]]
- [[Webots]]
- [[PufferLib]]
- [[PX4]]
- [[ArduPilot]]

---

## Sources

- https://github.com/microsoft/AirSim
- https://github.com/microsoft/AirSim/wiki
- https://github.com/microsoft/AirSim/blob/master/README.md

[^airsim-readme]: Microsoft AirSim README includes claims for drone/car support and API-driven control loops for autonomy simulation.

