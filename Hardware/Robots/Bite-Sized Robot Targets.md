---
title: Bite-Sized Robot Targets
tags:
  - robotics
  - reinforcement-learning
  - simulation
  - pufferlib
  - hardware
---

# Bite-Sized Robot Targets

These are concrete robot candidates you can write about one-by-one, with sources and a direct path to a PufferLib harness.

---

## 1) Upkie (wheeled biped)

**Why this is high-value now:** It is exactly your domain: balancing + simple observation streams + ROS2/embedded actuation + RL examples already available.

- **What it is:** Open-source wheeled biped using wheels plus articulated legs for uneven terrain.
- **Control stack:** Python/C++ library `upkie` and Gymnasium environments. Reals support comes via the *spine* process.
- **Core integration path:** Use `Upkie-PyBullet-*` for training and `Upkie-Spine-*` for real-robot/bridge.
- **Key docs claims:**
  - Upkies are buildable with off-the-shelf components (mjbots actuators).
  - Supports switching environments via name (`Upkie-PyBullet-*` ↔ `Upkie-Spine-*`), so policy code can remain unchanged across sim/real.
  - Spine options include real-time `pi3hat`, mock, and read-only.

**PufferLib fit:**
- Good for a stable **custom C/Python env wrapper** if you want deterministic `obs/reward/reset` and vectorization.
- The existing environment package is already useful for behavior prototyping, but throughput comes from custom vectorized envs, not raw hardware.

**Quick note for your 2D balance project:**
Use this as a *template for your own step contract*: fixed-rate obs, minimal action channels, and explicit reset states.

---

## 2) Segway RMP-220 (legacy 2-wheel balancer)

**Why this is practical:** Classic balancing dynamics, known industrial class, explicit C/C++ + ROS APIs.

- **What it is:** Balancing mobile platform with 48V 24Ah battery and CAN/UART interfaces.
- **Control stack:** User manuals include a `ROS Interface`, `C/C++ Interface`, and explicit interface callbacks/structures.
- **Signals:** API exposes speed, odom, IMU, battery, state/mode, and command-source fields.
- **Reality check:** Public docs are old and community maintenance is uneven; many examples are for legacy ROS versions.

**PufferLib fit:**
- Useful as a **hardware control case study** for low-level wrappers and transport latency handling.
- Not ideal as the first full-stack paper-to-policy environment because you’ll spend effort adapting legacy APIs.

---

## 3) HyperDog

**Why this is useful:** If you want a *non-biped, non-wheel* baseline for transfer ideas.

- **What it is:** Fully ROS2 + micro-ROS based quadruped in open-source repos.
- **Control stack (from repo):**
  - `hyperdog_msgs`, `hyperdog_ctrl`, `hyperdog_launch`, `hyperdog_teleop`, `micro_ros_agent`, Gazebo sim modules.
  - Includes gamepad command topics, gait generation, body-motion planning, IK node flow.
- **Strengths:** Explicitly designed for ROS2 package split and micro-controllers.
- **Weaknesses:** Older micro-ROS/foxy-oriented stack; you’ll likely do more integration work for modern toolchains.

**PufferLib fit:**
- Good for **mid-size sim2real harness patterns** with ROS graph boundaries.
- Not as low-friction for high SPS as native custom simulators.

---

## 4) WidowX-250 (Interbotix)

**Why this is useful:** Strong low-cost arm platform with concrete ROS2 launch patterns.

- **What it is:** 6DOF industrial-grade hobby/manufacturing crossover arm based on DYNAMIXEL X/XL series servos.
- **Docs strength:** Detailed arm-control package docs with launch arguments, mode config, register config, and simulation toggle.
- **Key points from docs:**
  - Launch command supports real/sim switching (`use_sim:=true`).
  - Motor config and mode config are explicit YAML-driven parameters.
  - ROS package structure exposes URDF + control/description stack.

**PufferLib fit:**
- Very good for a deterministic **action-bridge harness** study.
- Start with joint-space wrappers (`servo`/`joint` targets) and keep observation lean (joint + gripper + safety flags).

---

## 5) Poppy Ergo Jr

**Why this is useful:** Cheap and modular 6DOF arm with fast build and swap-able tool heads.

- **What it is:** Open-source, 6-DOF arm with simple 3D printed structure and Raspberry Pi control baseline.
- **Build/ops:** Assembly docs explicitly state short build time targets; simple mechanical swap options (gripper/pen/other).
- **Community tooling:** docs include Python/Jupyter, ROS entry points, and simulation-to-real pathways.

**PufferLib fit:**
- Great for **low-risk prototyping**: build repeatable command/state wrappers and test reward shaping at low cost.
- Throughput best handled by simulator/surrogate, not direct hardware loop.

---

## 6) OpenRoACH

**Why this is useful:** Ultra-low-cost, tiny legged baseline with ROS onboard and fully open source.

- **What it is:** 15 cm, 200 g hexapod with onboard ROS and long run-time durability claims.
- **Capabilities from source:** multi-surface walking/running, onboard BeagleBone, low-cost BOM, simple assembly.
- **Value for you:** Great for “small-footprint legged control” experiments if you want contact-rich behavior data.

**PufferLib fit:**
- Useful for **observation design experiments** (IMU/line sensors/vision variants), then transfer to higher-fidelity simulators.

---

## 7) Biped Open Platform Project

**Why this is useful:** Explicit research-facing open-source biped design direction.

- **What it is:** Open-source walking robot concept with 3D-printable/off-the-shelf parts and hybrid serial-parallel leg architecture.
- **Notable constraints:** Project currently described as early/untested in official pages.
- **Why include it anyway:** Useful as design reference for actuator placement and lightweight link architecture.

**PufferLib fit:**
- Primarily for **architecture scouting** unless you actively implement your own controllers.

---

## Suggested first 3 short notes to write next

1. **Upkie** (wheeled biped + gymnasium + spine architecture)
2. **Segway RMP-220** (legacy balancing platform with C/C++/ROS API details)
3. **HyperDog** (ROS2 + micro-ROS quadruped multi-node control graph)

These three give you a clean contrast:
- 1 is modern RL-friendly by design,
- 2 is legacy but low-level concrete,
- 3 is complex-legged, graph-heavy ROS integration.

---

## Tiny write targets for **robot arms** (next up)

Use these as one-note slices, not big surveys.

| Arm | Why it is bite-sized |
|---|---|
| [[reBot Arm B601]] | Existing source stack + known ROS2 package map; use it for one tight control-loop note |
| [[SO-ARM100]] | Lean LeRobot/ROS2 path; high signal for sim2real wrapper patterns |
| [Interbotix WidowX-250](https://docs.trossenrobotics.com/interbotix_xsarms_docs/ros2_interface/ros2.html) | Clear ROS2 launch flow and service model; easy to frame as action/state contract |
| [Poppy Ergo Jr](https://github.com/poppy-project/poppy-ergo-jr) | Cheap hardware, ROS/Python entry, and fast prototype loop |
| [OpenMANIPULATOR-X](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide/) | Explicit standalone-vs-ROS install path and package split |
| [Niryo One](https://robots.ros.org/niryo-one/) | Open education-grade arm with ROS package stack for simple baseline tasks |
| [Kinova Gen3 / Gen3 Lite](https://github.com/Kinovarobotics/ros2_kortex) | Research-grade arm; `ros2_kortex` gives clean launch + interface surface |
| [Franka Emika Panda / FR3](https://frankaemika.github.io/docs/franka_ros2.html) | Stable benchmark robot with known MoveIt2 + FCI patterns |

---

## Mini comparison (goal alignment)

| Platform | Best for | RL sim/real friction | Harness style |
|---|---|---:|---|
| Upkie | wheeled-biped balancing + simple action/obs | Medium | `env -> spine` bridge |
| Segway RMP-220 | classic balance baseline + C/C++ API | Medium-low | C/C++ DLL + ROS bridge |
| HyperDog | quadruped graph + micro-ROS | Medium | Topic graph + control node boundaries |
| WidowX-250 | budget 6DOF arm RL/IL tasks | Medium | Joint/pose topic bridge |
| Poppy Ergo Jr | fast, cheap arm prototype | Medium | ROS/Python wrappers |
| OpenRoACH | tiny legged locomotion exploration | Medium-high (if sim-first) | Custom wrapper + custom sim |
| Biped Open Platform | architecture + actuators + leg layout | Low currently | project-specific

## Tiny write targets for **balancer / similar platforms** (next)

| Platform | Why this is bite-sized |
|---|---|
| [OpenDog V3](https://github.com/XRobots/openDogV3) | Open-source quadruped with ROS-flavored package layout; useful for ROS graph notes |
| [MABEL](https://mabel.biped.solutions) | Historical but concrete biped control references and a clean baseline in literature |
| [OpenRoACH](https://wiki.eecs.berkeley.edu/biomimetics/Main/OpenRoACH) | Already in this list; still useful for very small-footprint legged observation design |
| [Segway RMP-220](https://robotics.segway.com/wp-content/uploads/2021/04/Segway-RMP-220Lite-Specs.pdf) | Legacy balancer with public API tables and transport modes |

---

## Source notes

- Upkie: open-source wheeled biped project with Python/C++ stack and Gymnasium environments, plus Bullet/Pi3hat spines with mock/readonly modes. `https://github.com/upkie/upkie`
- Segway RMP-220: manual and C/C++ API tables include ROS introduction and callback structures over CAN/UART.
- HyperDog: ROS2 + micro-ROS architecture with controller, teleop, launch, and Gazebo components.
- Interbotix WidowX-250: ARM launch + motor config + mode config + ROS2 package structure in official docs.
- Poppy Ergo Jr: 6DOF, 3D-printable structure, Raspberry Pi control, and docs/assembly + ROS integration paths.
- OpenRoACH: tiny ROS-capable hexapod, open design, payload and runtime characteristics in paper and project page.
- Biped Open Platform Project: open biped research direction and leg design strategy.
- OpenMANIPULATOR-X docs: ROBOTIS ROS-capable 5-DOF/6-DOF platform and clear setup flow.
- Kinova ros2_kortex GitHub: ROS2 launch/package split and supported Kortex arm models.
- Franka ros2 docs: official repository and launch surface for FR3/Panda.
- OpenDog V3 and Segway API references are for architecture/scaffold mapping, not fresh ecosystem.

---

## Next action

If you want, next pass I can convert each of the above into a separate one-page markdown dossier with
- consistent frontmatter,
- direct action-observation contract,
- one comparison row against Ascento/UMV/Unitree,
- and concrete PufferLib training patterns.
