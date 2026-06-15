---
title: Ascento
aliases:
  - Ascento Research
  - Ascento
  - ETH Ascento
tags:
  - robotics
  - simulation
  - wheeled-biped
  - reinforcement-learning
  - pufferlib
  - ros2
  - control
  - whole-body-control
---

# Ascento

**Ascento** is a Swiss wheeled-biped research platform that has both academic roots (ETH Zürich) and a commercialized ROS2 fielded stack from MyBotShop. The key reason it still matters for your workflow is that it keeps a realistic hardware+control stack (not just a toy simulator) while exposing ROS2 telemetry/control surfaces.

---

## Research Status

| What | Signal | Confidence |
|---|---|---:|
| Core control architecture | Four actuators total, **C++ + ROS** software, Kalman-state estimation + LQR stabilization | High |
| Legacy prototype detail | ROS/C++ + IMU+encoder Kalman filter + LQR + jump/fall controllers | High |
| Current docs | ROS2 topics/services documented and runnable with direct `ros2 topic` / `ros2 service` commands | High |
| Current product specs | ROS2 + Jetson Orin NX + Lidar/vision payload, 50.4V 17.5Ah battery (12S5P), 4G/WiFi | High |
| Sim availability | Papers + docs mention Gazebo/Fortress path, but public-facing interface for deterministic RL-style step/reset still weak | Medium |
| Direct PufferLib fit (native) | Not exposed as a native `step/reset` API in public docs; needs adapter/bridge | High |

---

## Core papers and what they imply

### 1) Ascento: A Two-Wheeled Jumping Robot (ICRA 2019)
- Mechanical design uses **four actuators**: two wheel drives and two leg actuators; 2-wheel inverted-pendulum-style simplified control model is used for baseline balancing.[^ascento-arxiv]
- The leg mechanism is a **three-bar linkage**, with each leg height adjusted by hip actuation (~31–66 cm).
- Control stack is written in **C++** and uses ROS for communication; IMU + encoder measurements feed a **Kalman filter**.
- LQR stabilizes the robot; jump and fall recovery include heuristic phase sequencing + feed-forward/feedback layers.[^ascento-arxiv]

### 2) LQR-Assisted Whole-Body Control with Kinematic Loops (IEEE RA-L 2020)
- Confirms Ascento only needs four actuators; adds full dynamics with leg kinematic loops and a WBC hierarchy around **LQR-assisted balancing**.[^ascento-wbc-pdf]
- Onboard compute note: controller run at **400 Hz** with average control period **1.56 ms** (about 1.20 ms WBC optimization, 0.11 ms DARE); a strong indicator of the control stack’s real-time orientation.[^ascento-wbc-pdf]
- This is still an academic control stack, not a direct off-the-shelf RL API; it is valuable as a model and behavior source.

### 3) RL follow-up (2024, stair climbing)
- Uses Ascento in learning context; uses a **position-based RL formulation** and **asymmetric actor-critic** for stair-climbing control with a boolean stair mode feature, achieving 15 cm stair climbing in real-world tests for Ascento.[^ascento-rl]

---

## EKF and LQR notes (practical reading)

- The papers use the term **Kalman filter** for state estimation. In a wheeled-biped context with nonlinear IMU kinematics, teams typically promote this to EKF in implementation if model nonlinearity is significant. That is a reasonable inference for controller replication work, but it is still an inference unless validated in your exact firmware branch.
- The documented controllers are unambiguous about **LQR as a stabilization backbone** for low-level balancing and pitch correction, with gains scheduled across leg-height states in the original prototype controller.
- For a PufferLib stack, the clean split is:
  - let a simulator provide `state + obs` for learning.
  - keep LQR (and any EKF/EKF-like estimator) inside a host-side guard or robot-side safety loop that tracks commands from RL.
- This pattern gives you the RL upside of fast policy updates while preserving deterministic stabilization logic on the hardware side.

---

## Morphology and DoF (where joints are)

From the papers and docs, Ascento’s mechanical chain is compact and explicitly link-based:

- Two wheel hubs with wheel actuators.
- Each leg has a non-trivial linkage including hip/inner/outer joints in the three-bar chain and knee/torsion spring elements; practical control exposes higher-level leg-height/pose behavior via ROS2 publish topics.
- Published controller state space uses generalized states for pitch, wheel speed, and heading-like terms and interpolates LQR gains across leg heights.

Practical interpretation:
- The strict mechanical DoF is more than four joints in the CAD mechanism due to linkage internals, but control abstraction is strongly reduced to 2 wheel torques + leg-height family + body states.
- This makes it suitable as a **reduced-state RL target** if you encode actuator/height dynamics as bounded setpoints rather than direct low-level spring/joint torque commands.

---

## Ascento hardware lineage and compute

### Paper-era hardware (2019 prototype)
- Wheel motors: Maxon EC90 frameless hub motors.
- Leg motors: ANYdrive series-elastic actuators.
- IMU: ADIS16460 + ToF / Hall/auxiliary sensing for jump triggering and state estimation.
- Compute in paper demo: Intel NUC-style platform + MCU bridge for low-level motor comms.[^ascento-pdf]

### Current Ascento Research / Guard profile (R&D docs)
- Compute: **NVIDIA Jetson Orin NX** (or derivative platform references in docs).
- Battery/system: 50.4V, 17.5Ah (12S5P), realsense/LiDAR/camera payload options, 4G/WiFi, acoustic I/O.[^ascento-docs]
- docs flag that some spec/feature fields are **subject to change** per delivered unit and ask not to assume static parity with every unit.[^ascento-docs]

---

## ROS2 integration reality (what is actually usable)

The ROS2 docs are concrete enough for adapter engineering:

- **Network setup**
  - Host side LAN static IP `10.42.0.51/24` and robot at `10.42.0.50` for SSH and network ping checks.
  - Explicit SSH and component IP map includes Ascento Nvidia, MCU, and sensor hosts.

- **Status topics**
  - `/ascento/imu`, `/ascento/joint_states`, `/ascento/battery_state`, `/ascento/estimator/base_height`, `/ascento/estimator/speed_bias`, etc.

- **Action topics/services**
  - `/ascento/cmd/twist` (velocity)
  - `/ascento/cmd/base_height_meter_set`
  - `/ascento/cmd/clear_error`
  - `/ascento/cmd/hard_estop_set`, `/ascento/cmd/soft_estop_set`
  - services: `/ascento/cmd/standup`, `/ascento/cmd/homing_start`, `/ascento/cmd/speed_bias_increase/decrease`, plus light/reboot controls.
- **Transport stack hints**
  - docs explicitly discuss Zenoh bridge setup and ROS2 workspace build workflow.

This is enough for a deterministic adapter with explicit polling and controlled action-rate gating, but it is still not a native RL env API.

---

## How to hook Ascento into PufferLib

### Path A (recommended first): custom deterministic surrogate
- Re-implement reduced dynamics with
  - action space: velocity command + leg height target + behavior mode + safety toggles
  - obs space: IMU, joint kinematics, base height/speed bias, battery+diagnostic flags.
- This gives high SPS and deterministic reset semantics.

### Path B (intermediate): ROS2 adapter around documented topics
- Wrap ROS2 command/status topics above in a **fixed-frequency adapter process**.
- Never make the adapter the training loop. Keep simulation step at fixed dt inside PufferLib and stream action/obs at controlled cadence.

### Path C (deployment): on-hardware inference service
- Run policy inference + safety checker in host process, send commands at bounded rates.
- Keep hard low-level loops (estimation, motor safety, e-stop handling) on robot side.

Suggested obs/action contract template:
- obs: `gyro, accel, joint_pos, joint_vel, base_height, speed_bias, battery_soc, estimator_flags`
- act: `vx, wz, base_height_target, mode_id`
- reward: forward progress - stability penalty - height violation - effort proxy
- done: safety fault/tilt fail/time limit

---

## Headless / throughput / sim2real outlook

| Layer | Headless | Deterministic reset | Likely SPS |
|---|---|---:|---:|
| Pure local custom Ascento model | Yes | High | High |
| ROS2 live adapter | Yes (headless host) | Medium | Medium-Low |
| Native docs-side Gazebo path | Limited/unclear in public docs | Medium-Low | Medium-Low |
| Full commercial image | Host-driven | Medium | Medium |

Interpretation:
- If SPS is the bottleneck, the productive route is **custom C/C++ or C99 env core + ROS2 bridge**.
- On-hardware tests should be treated as sparse policy validation, not primary sample efficiency source.

---

## Comparable systems (5+ style comparison)

| Platform | Core class | Control/API posture | Headless vectorized potential | Why relevant |
|---|---|---|---|---|
| **Ascento Research** | wheeled-biped (4 actuation concept) | ROS2 command/topic API documented | Medium | closest analog to your balance + wheel-leg target |
| **RAI UMV** | 2D-like self-balancing robotic vehicle | ROS + Isaac Lab/RL reported in papers | Medium | RL-first culture + sim2real precedent |
| **Unitree Go2 / A1 lineage** | quadrupedal 3D locomotion | SDK + ROS2 + ecosystem tools | Medium-High | strongest software ecosystem and sim support |
| **Unitree G1** | bipedal humanoid | deeper SDK/ROS2 stack | Medium | high-DoF scaling target if you jump complexity |
| **ANYmal / Spot** | quadrupeds | mature enterprise APIs | Medium | strong for industrial-grade sensing/ops tradeoff |
| **Segway-style self-balancing UGVs** | 2-wheel balancing dynamics | legacy/SDK mix | Medium-Low | good for simple balance benchmark |

---

## Gaps to close before production scale

1. Confirm exact software revision on your specific unit (the docs explicitly warn field versions can drift).[^ascento-docs]
2. Confirm estimator internals for your branch: docs list topic-level outputs, not always internal filter model details.
3. Decide reset semantics early (pose jitter bounds, initial tilt, floor height range).
4. Validate whether current docs version still claims Gazebo/Fortress support for your stack revision.

---

## Related notes

- [[PufferLib]]
- [[PufferLib C99 Environment Authoring]]
- [[PufferLib Robotics Fit and Limits]]
- [[ROS2]]
- [[Gazebo]]
- [[Sim2Real]]
- [[Unitree]]
- [[UMV]]

---

## Source Notes

[^ascento-arxiv]: Ascento paper: C++, ROS communication, Kalman estimation, 4 actuators, controller architecture. https://ar5iv.labs.arxiv.org/html/2005.11435
[^ascento-pdf]: Ascento prototype hardware and specification excerpted from mirrored paper PDF (e.g., actuator set and 10.4 kg class mass context). https://www.interregnorthsea.eu/sites/default/files/2026-01/Ascento.pdf
[^ascento-wbc-pdf]: LQR-assisted whole-body-control paper (IEEE RA-L) with closed-form kinematic-loop dynamics and timing/compute stats (400 Hz, 1.56 ms control period). https://www.research-collection.ethz.ch/server/api/core/bitstreams/c0497fd4-e406-40d8-a1a2-49e556f96433/content
[^ascento-rl]: Blind stair climbing RL follow-up; Ascento reaches 15 cm stair tasks with asymmetric actor-critic and privileged info. https://arxiv.org/abs/2402.06143
[^ascento-docs]: Ascento Research docs/specs and static integration pages (Orin NX, battery, network ranges, payload, features). https://docs.mybotshop.de/projects/robot_ascento/html/index.html
[^ascento-interface]: ROS2 interface doc page with explicit topics/services, setup, and environment steps. https://docs.mybotshop.de/projects/robot_ascento/html/ros.html
[^ascento-research]: MyBotShop-facing Ascento Research site (training/messaging claims including Isaac Gym + ROS mentions). https://www.ascento-research.com/
[^ascento-home]: Additional Ascento public framing and product language from the site. https://www.ascento-research.com
