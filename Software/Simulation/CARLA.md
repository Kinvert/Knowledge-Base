# CARLA

`CARLA` is an open-source simulator for autonomous driving research with dedicated driving scenes, traffic behavior, and extensible sensor setups. It is commonly used for training, testing, and validating autonomous road-vehicle stacks.

---

## Core positioning

CARLA is explicitly built for autonomous driving and published with open assets (city maps, urban actors, and vehicles). It is designed to expose realistic data and scenario interfaces so that perception/planning/control stacks can be evaluated against repeatable routes.[^carla-repo]

---

## Why CARLA is a strong UGV option

- Road-domain focus: intersections, traffic participants, and urban/rural scenario classes.
- Strong ecosystem links for scenario-runner and RL/IL pipelines.
- Suitable for full-sensor policy pipelines (camera/radar/LiDAR-like abstractions in many branches).

---

## Comparison chart (UGV sims by ground-vehicle posture)

| Dimension | CARLA | AirSim | LGSVL/SVL | Webots | Gazebo Garden | CoppeliaSim |
|---|---|---|---|---|---|---|
| Primary domain | Autonomous driving | Ground + air | Autonomous driving | General mobile robotics | General robotics + ROS | General robotics |
| Scenario support | Strong | Moderate | Moderate/strong | Moderate | Strong via plugins | Strong via scene modeling |
| Built-in sensors | Broad AD-oriented sensor suite | Multi-modal | AD + map-aware | Task-dependent | Extensible | Extensible |
| Benchmark tooling | Yes (official leaderboard ecosystem) | Limited road benchmarks | Scenario-focused | General RL/control | General + ROS | General |
| Best use | AD policy, planner validation | cross-domain autonomy | AD stack + ROS2 integrations | lightweight prototyping | ROS-driven and custom robot tasks | custom mechanics experiments |

---

## Pros and cons

### Can do well

| Strength | Why it matters |
|---|---|
| Repeatable scenario structure | Easier policy comparability |
| AD ecosystem alignment | Better fit for road-vehicle tasks |
| Open and scriptable workflows | Good for reproducible research |

### Weak spots

| Limitation | Why it matters |
|---|---|
| Focused on road autonomy | Less convenient for non-road or farm/industrial rovers |
| Environment-heavy rendering | Lower batch density than stripped C++ simulators |
| Integration path complexity | Need disciplined API wrapper design for RL loops |

---

## Practical workflow

1. Choose map + route style + traffic density.
2. Enable sensor stack and logging schema.
3. Drive scenario generation through Scenario Runner/launcher.
4. Train policy loop via your chosen adapter.
5. Validate with held-out maps and new seeds.

---

## Related notes

- [[AirSim]]
- [[LGSVL Simulator]]
- [[Gazebo]]
- [[PufferLib]]
- [[ROS2]]
- [[Isaac Sim]]

---

## External links

- https://github.com/carla-simulator/carla
- https://github.com/carla-simulator
- https://carla.org/
- https://github.com/carla-simulator/carla/tree/master/Docs
- https://github.com/carla-simulator/leaderboard

[^carla-repo]: CARLA repository description and project pages confirm open-source autonomous-driving design and scenario tooling.

