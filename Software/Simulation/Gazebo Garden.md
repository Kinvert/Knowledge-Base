# Gazebo Garden

`Gazebo Garden` is part of the modern Gazebo ecosystem and a practical choice for UGV workflows because it exposes mobile robot primitives such as differential drive, odometry output, and ROS transport integration.

---

## Core positioning

The Garden docs include concrete mobile-robot configuration patterns (for example `DiffDrive`) with velocity topic and odometry setup that map well into autonomous ground vehicle control loops.[^gazebo-doc]

---

## Why use this for ground robots

- Strong overlap with ROS2 workflows.
- Good fit for custom robot URDF/SDF model workflows.
- Appropriate when you need a controllable baseline with transparent simulation internals rather than strictly AD-specific scenario stacks.

---

## Comparison chart (Gazebo Garden in UGV context)

| Category | Gazebo Garden | CARLA | AirSim | LGSVL/SVL | Webots | CoppeliaSim |
|---|---|---|---|---|---|---|
| ROS integration | Native/strong | tool-specific | possible | strong | possible | strong |
| Diff-drive support | explicit topic interface | scenario-level | limited by AD world | scenario-level | yes via controllers | yes via scripts |
| Vehicle dynamics | customizable via plugins/models | scenario tuned | moderate | moderate | moderate | high flexibility |
| Learning stack setup | moderate | scenario focused | script heavy | stack heavy | simple | flexible |
| Best use case | robotic mobility and custom controllers | urban AD validation | mixed aerial/ground | AD stack integration | educational/prototyping | manipulator + mobile mixed studies |

---

## Pros and cons

### Pros
- Open ecosystem with broad middleware compatibility.
- Good for controlled robotics experiments where you define your own assumptions.
- Strongly scriptable for reproducible RL pipelines.

### Cons
- Not as AD-scene complete as CARLA/LGSVL by default.
- Requires model and plugin curation to match real-world rover/framerate behavior.
- Batch performance depends on scene complexity and bridge architecture.

---

## Practical notes

- Start with `DiffDrive` for differential rover baselines and upgrade to full vehicle plugins when dynamic complexity rises.
- Keep command topic names and frame conventions fixed across training and HIL stages to avoid sim-to-real drift.

---

## Related notes

- [[CARLA]]
- [[AirSim]]
- [[LGSVL Simulator]]
- [[Webots]]
- [[CoppeliaSim]]

---

## External links

- https://gazebosim.org/docs/garden/moving_robot
- https://gazebosim.org/
- https://github.com/gazebosim
- https://github.com/gazebosim/gz-sim
- https://github.com/gazebosim/gz-sim/tree/gz-garden

[^gazebo-doc]: Gazebo Garden moving robot docs explicitly show command topic and odometry path for differential drive vehicle primitives.

