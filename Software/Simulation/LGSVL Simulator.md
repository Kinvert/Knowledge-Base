# LGSVL / SVL Simulator

`LGSVL` (often branded as `SVL Simulator`) is a ROS/ROS2-focused autonomous vehicle simulator built on Unity, with map, ego-vehicle, and bridge/plugin tooling for autonomy stack testing.

---

## Core positioning

LGSVL is positioned as a multi-robot, AD-oriented simulator where the main workflow is integration with vehicle software stacks (Autoware, Apollo, ROS2 bridges, and custom message schemas). The docs site includes plugin categories for sensors, bridges, controllable vehicles, traffic behavior, and map workflows.[^lgsvl-docs]

---

## What it brings to UGV work

- Strong bridge-first mindset for driving stacks.
- Clear separation between map/sensor/vehicle configuration.
- Practical vehicle-level plugin architecture for adding your own ego dynamics or bridge channels.

---

## Comparison chart (UGV and mobile robotics orientation)

| Use axis | LGSVL / SVL | CARLA | AirSim | Gazebo Garden | Webots | CoppeliaSim |
|---|---|---|---|---|---|---|
| AD stack integration | Very strong | Strong | Good | Moderate via ROS | Moderate | Moderate |
| Plugin/custom vehicle path | Explicit plugin model | Scenario runner strong, plugin model different | Lower-level API control | ROS plugins + transport | Scripted controllers | Embedded scripts + plugins |
| Tooling for maps/worlds | Strong | Strong | Moderate | Strong | Moderate | Strong |
| ROS2 workflow | Native focus | Works via tooling | Possible | Strong, common | Strong | Strong |
| Real-time policy debugging | Good | Good | Good | Good | Good | Moderate |

---

## Pros / cons for RL workflows

| Pros | Cons |
|---|---|
| Easy to connect to ROS2 and AD tools | Documentation quality varies by version |
| Multi-vehicle testing is first-class | Not as universal for non-road ground robots |
| Clear vehicle definition path in docs | Heavier runtime profile than stripped simulator loops |

---

## Deployment notes

- If your project is AD-first, this one usually beats generic robot sim stacks for communication fidelity.
- For UGV variants with heavy wheel dynamics (mud/off-road) and non-AD controllers, test whether the default physics and tire behavior are enough or if you need custom calibration layers.

---

## Related notes

- [[CARLA]]
- [[Gazebo]]
- [[ROS2]]
- [[Webots]]
- [[CoppeliaSim]]
- [[PX4]]

---

## External links

- https://github.com/lgsvl/simulator
- https://unity-proj.github.io/lgsvl/
- https://github.com/lgsvl/simulator/releases/
- https://github.com/lgsvl/Autoware
- https://github.com/lgsvl/Autoware/tree/lgsvl_develop

[^lgsvl-docs]: The public documentation index and GitHub repository list bridge/sensor/vehicle plugin architecture and map + simulation workflows.

