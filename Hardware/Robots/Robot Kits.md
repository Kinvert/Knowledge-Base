# Robot Kits to Write Next (Small, Specific Dossiers)

You asked for bite-sized topics. This is a shortlist of specific robots to write in small, practical notes next (one platform per note) with concrete reasons tied to your current RL/Sim2Real direction.

## Why this list exists

- Keep focus on **sim2real workflow** and **PufferLib compatibility**.
- Prefer projects with public code/docs plus some hardware + API exposure.
- One note per platform, each scoped to:
  - mechanism + actuation
  - software/SDK stack
  - what an RL harness would likely need
  - realistic expectations on SPS/headless deployment

## Robot Arms (small, practical targets)

These are low-complexity starts for manipulator-focused notes.

| Platform | Why it fits your next write | What to watch |
|---|---|---|
| [[reBot Arm B601]] | Already partially covered in your KB; good bridge to LeRobot + ROS2 and a concrete RL path with wrappers. | Keep the next note centered on `step/reset` adapter patterns for policy training, not broad vendor marketing. |
| [[SO-ARM100]] | Existing entry already exists and is lightweight; useful for comparing open-source arm toolchains. | Expand on `ROS2` topic/action contract and what breaks if you push PPO at 100+ Hz. |
| [Interbotix X-Series (WidowX-250)](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_interface/ros2.html) | Mature ROS2 docs + many users in real environments. | Capture the controller model (position/effort/trajectory modes) and simulator parity strategy. |
| [Poppy Ergo Jr](https://github.com/poppy-project/poppy-ergo-jr) | 6-DOF, open hardware/software, ROS+Python compatibility, cheap build path. | Verify firmware/driver maturity before promising high-rate loops. |
| [OpenMANIPULATOR-X](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/specification/) | Low-cost 5-/6-DOF arm with clear open e-manual and ROS2-friendly package structure. | Keep wrappers at joint-level to avoid controller namespace drift. |
| [Niryo One](https://robots.ros.org/niryo-one/) | Fully documented ROS-oriented educational arm with open repo history. | Use one note for driver expectations vs real hardware reliability. |
| [Franka Panda / FR3](https://frankaemika.github.io/docs/franka_ros2.html) | Strong research benchmark; standard manipulation stack with many references. | ROS2 launch and FCI stability requirements can add integration overhead. |
| [SMARTmBOT](https://arxiv.org/abs/2203.08903) | ROS2-based mobile platform with open-source framing; useful because it mixes perception + control stack in one stack. | Mostly mobile-focused, but includes manipulator-relevant planning + control plumbing patterns. |

## Two-Wheeled / Wheeled-Biped / Balancing Robots (closest to your current plan)

These are better for your 2DOF-like balancing bot direction and transfer ideas (IMU + line-lidar state structure, simple action spaces).

| Platform | Why it fits | RL relevance |
|---|---|---|
| [Upkie](https://github.com/upkie/upkie) | Open-source wheeled-biped platform with wheels + legs and built-in `Gymnasium` envs (PyBullet and Spine). | Directly matches your inverted-pendulum + line-lidar experiments; strong option for one-note "small env contract". |
| [MABEL](https://grizzle.robotics.umich.edu/files/ACC_MABEL_2008.pdf) | Historical high-performance 5-link biped rig with model-based control focus and public paper details. | Useful as a legacy control benchmark and comparison baseline. |
| [Segway RMP 220 (legacy)](https://index.ros.org/r/segwayrmp/) | Public ROS interface exists (`segwayrmp`) plus legacy C/C++ API references; known two-wheel balancing class. | Good for comparing classic production platform integration risk vs modern open projects. |
| [HyperDog](https://github.com/NDHANA94/hyperdog_ros2) | Open-source quadruped with ROS2 + micro-ROS and Jetson/Nano + STM32 split. | Good counterpoint on when wheeled-balancer ideas transfer poorly to legged gaits. |
| [OpenRoACH](https://www.emergentmind.com/articles/1903.00131) | Open-source legged (hexapedal) platform with onboard ROS; strong lesson for fast, low-cost mechanics + contact-rich behavior. | Useful for understanding sensor layout and low-cost estimation burden. |
| [Biped Open Platform Project](https://biped-open-platform-project.github.io/) | Actively maintained open-source research biped concept using mostly off-the-shelf / 3D-printable pieces. | Useful for "what data to log" and "what interface minimalism actually means" in early prototypes. |
| [OpenPodcar2](https://arxiv.org/abs/2604.24242) | ROS2-interfaced open self-driving base from mobility-scooter donor vehicle, with simulated and real common interface. | Helpful for sim-real tooling patterns even if behavior class differs from balancing. |

## Ultra-Bite Notes You Can Add Next (1-page each)

If you want only tiny topics, these are good first targets:

- `Upkie Env + Adapter`
- `Upkie vs ReBot B601: control-telemetry bridge`
- `Segway RMP + Puffer wrapper feasibility`
- `Interbotix WidowX-250: ROS2 action map for PPO`
- `Poppy Ergo Jr: 6-DOF action simplification for Puffer`
- `HyperDog: when to use/avoid legged dogs for your compute budget`

## Already in the KB (for sequencing)

- [[SO-ARM100]]
- [[reBot Arm B601]]
- [[Ascento]]
- [[RAI Ultra Mobility Vehicle]]
- [[Unitree]]
- [[Bite-Sized Robot Targets]]

## Sources

- Upkie repository: https://github.com/upkie/upkie
- Upkie docs/ROS/MuJoCo tooling references: https://github.com/upkie/upkie
- MABEL technical paper: https://grizzle.robotics.umich.edu/files/ACC_MABEL_2008.pdf
- SegwayRMP ROS index: https://index.ros.org/r/segwayrmp/
- Segway user manuals archive: https://robotics.segway.com/wp-content/uploads/2023/10/RMP220-User-Manual-v1.0.pdf
- HyperDog paper and implementation: https://arxiv.org/abs/2209.09171
- HyperDog repo: https://github.com/NDHANA94/hyperdog_ros2
- Biped Open Platform project: https://biped-open-platform-project.github.io/
- OpenRoACH reference: https://people.eecs.berkeley.edu/~ronf/PAPERS/lwang-openroach-icra19.pdf
- OpenRoACH summary: https://arxiv.org/abs/1903.00131
- Interbotix X-ARM docs: https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_interface/ros2.html
- Poppy Ergo Jr: https://github.com/poppy-project/poppy-ergo-jr
- OpenPodcar2 preprint: https://arxiv.org/abs/2604.24242
- SMARTmBOT ROS2 platform: https://arxiv.org/abs/2203.08903
