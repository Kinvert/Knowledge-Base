---
title: reBot B601
aliases:
  - reBot Arm B601
  - reBot-DevArm
  - Seeed reBot
  - reBot B601 DM
  - reBot B601 RS
tags:
  - robotics
  - embodied-ai
  - open-source-hardware
  - lerobot
  - ros2
  - pufferlib
  - sim2real
---

# reBot Arm B601

**reBot Arm B601** is Seeed Studio's open-source manipulator stack (DM/RS variants) aimed at embodied AI and low-barrier imitation/RL workflows.  
For RL engineering, the important point is: it is intentionally open on the stack, but not yet a native high-throughput RL environment API.

---

## Research status

| Area | Current status | Evidence |
|---|---|---|
| Product positioning | Low-cost, fully open-source manipulator stack for embodied intelligence | [^seeed-announce] |
| Family | DM + RS variants + leader arm for teleop | [^hf-rebot] |
| Core ecosystems | LeRobot + ROS1/ROS2 + Isaac Sim + Pinocchio + MoveIt | [^seeed-announce] |
| Hardware openness | Full 3D/CNC hardware + BOM, with a known CC BY-SA NC to CERN-OHL-W 2.0 transition | [^devarm-readme] |
| Public low-level RL API | No published `step/reset`; must bridge via ROS2 or LeRobot toolchain | [^wiki-dm], [^wiki-ros2] |
| Practical PufferLib fit | Medium only with adapter, low SPS unless mirrored by a custom surrogate | _Inferred from API surface and transport model_ |

---

## Product stack: what it is

### B601-DM (follower)

The follower arm is a **6-DOF + gripper** unit driven primarily by **Damiao DM43x** motors in this variant.
The ROS/Lerobot docs list this as the primary pair with the `StarArm102`/`reBot Arm 102` leader arm for teleoperation data capture and demonstration workflows.[^hf-rebot]  

### B601-RS (variant)

The RS variant shares the same family but targets **RobStride-class motors** in the open hardware BOM structure.  
Current public docs for the RS get-started are present, but not as complete as the DM stack (content is explicitly sparse/staged).[^wiki-rs]

### Leader follower architecture

The project is designed for teleoperation workflows: a human-driven leader arm (e.g., StarArm102/"Arm 102") captures motion, then the follower executes trajectories on hardware.[^hf-rebot]  
This matters because the software flow is oriented around imitation/data workflows first, then policy transfer.

---

## Open hardware and build stack

The GitHub hardware specification is unusually explicit:

- Open-source CAD with versioned STEP/BOM dumps.
- Mechanical structure includes both 3D-printed and CNC metal parts.
- The authors explicitly state the open-source BOM is a development baseline and may differ from final production kits in finish/material substitutions and tolerances while keeping mechanical layout unchanged.[^devarm-hw]

Important license shift:

- ReBot moved from **CC BY-SA NC** to **CERN-OHL-W 2.0** effective May 11, 2026 (software uses Apache-2.0).  
- This matters for commercialization and closed-source product integration decisions.[^devarm-readme]

---

## Software and APIs (relevant to RL workflow)

### LeRobot integration

Seeed publishes specific packages for this exact robot:

- `lerobot-robot-seeed-b601` (follower integration)
- `lerobot-teleoperator-rebot-arm-102` (teleoperation side)

The package docs explicitly identify hardware compatibility and comm path:

- 6-DOF series with gripper
- Damiao and RobStride motor classes
- CAN bus over USB-CAN adapters, including SocketCAN and Damiao USB2CAN style tools.
- explicit setup commands and ROS package installation paths for demos and data collection.[^pypi-follower]

This is good for data loops and transfer workflows, but it does **not** provide a dedicated RL `env.step()` contract.

### ROS2 integration surface

The ROS2 integration docs for B601-DM expose concrete telemetry + control APIs:

Status topics:

- `/rebotarm/joint_states` -> `sensor_msgs/msg/JointState`
- `/rebotarm/arm_status` -> custom arm status message
- `/rebotarm/joints/<joint>/state`
- `/rebotarm/gripper/state`

Service APIs:

- `/rebotarm/enable`, `/rebotarm/disable`, `/rebotarm/safe_home`
- `/rebotarm/set_mode` (includes `mit`, `pos_vel`, `vel`)
- `/rebotarm/move_to_pose_ik`, `/rebotarm/gripper/set`
- `/rebotarm/gravity_compensation/start|stop`

Actions:

- `/rebotarm/move_to_pose` (pose control)
- `/rebotarm/follow_joint_trajectory` (trajectory)
- `/rebotarm/gripper/command` (standard gripper action)
- package executables include `reBotArmController`, `GravityCompensation`, `GripperControl`, `MoveTo`, and `MoveToPose`.[^wiki-ros2]

That API surface is usable for RL, but you need a deterministic wrapper.

### Simulation/compute stack

The launch/post claims emphasize a full development pipeline using:

- Isaac Sim
- ROS1/ROS2/Humble
- LeRobot
- Pinocchio

with the goal of modeling in simulation -> train -> deploy on hardware.[^seeed-announce]

---

## Why it is attractive for your goals

You asked about **realistic RL + sim-to-real**. reBot is attractive because:

1. it is explicitly built for embodied AI (not just hobby toy code),
2. it has open-source hardware/driver/docs you can inspect end-to-end,
3. it supports both data-collection workflows and inference/control handoff paths,
4. it exposes enough structured interfaces (ROS2 topics/services/actions) to build a stable host-side bridge.

---

## PufferLib harness pattern for reBot

Because PufferLib is strongest with deterministic vectorized `step/reset`, treat reBot as a **target platform** and run one of two harness paths:

### Path A: hardware-throughput bridge (for reality checks)

Use ROS2 bridge with fixed-rate control cadence.

- `reset(seed)` is emulated at Puffer layer by controlled startup + state reinitialization routine.
- `step(action)` converts action vector into a bounded command message/action goal.
- `obs` pulls joint states, arm status, and optionally gripper/status channels at fixed intervals.
- reward/done logic remains outside hardware in the wrapper (stability, joint error, collision flags, timeout).

**Tradeoff:** this is usually low-to-medium SPS because each step crosses IPC/ROS graph and hardware latency.

### Path B: simulation-first surrogate + hardware validation

Build a C-native surrogate env in PufferLib first (minimal fixed dt physics + reduced action semantics), then deploy only checkpoints to hardware through the same contract.  

This is the usual fast-learning route for robotics stacks where hardware transport is inherently non-vectorized.

### Suggested action/observation schema

Observation:

- joint positions (6)
- joint velocities (6)
- gripper state/position
- arm status bits (enabled/error/mode)
- optional battery/temperature fields if your unit publishes them

Action:

- mode switch (position/velocity/torque-like abstractions)
- target pose delta or joint delta (depending on policy design)
- optional gripper command

Reward:
- pose tracking, joint smoothness, error penalties, safety flags, and stability timeout.

---

## Headless and SPS outlook

ReBot can be run in scripted, non-GUI mode on a host PC, but the official arm stack is not a native headless RL simulator.  

Practical expectation:

- **Hardware-in-loop**: deterministic wrapper possible, but throughput is bounded by robot transport/control loops.
- **Native high-SPS candidate**: only if you move most learning into a custom surrogate and leave reBot as deployment target.

So for your objective, reBot is stronger as **sim2real validation** hardware than as a direct high-throughput environment source.

---

## Comparison with nearby platforms

| Platform | Class | Open stack depth | Hardware control API maturity | Headless/sps path | Why this matters |
|---|---|---|---|---|---|
| **reBot B601-DM** | 6-DOF teleop-enabled arm | Strong (OS files + ROS2 + LeRobot packages) | Medium (no native step/reset) | Medium-low (wrapper-limited) | Best candidate in this family with practical open tooling |
| **reBot B601-RS** | 6-DOF sibling variant | Medium (publicly staged docs) | Medium (likely ROS2-compatible path) | Medium-low (similar transport limits) | Useful for motor diversity but more uncertain maturity |
| **reBot StarArm102 / leader arm** | 6-DOF leader controller | Strong for teleop demos | Medium (human-in-loop control primary) | Low for PPO loops | Useful for data collection and dual-arm workflows |
| **SO-ARM100** | 6-DOF low-cost LeRobot arm | Strong community around LeRobot | Medium | Low-medium | Better ecosystem for minimal-cost RL demos |
| **xArm 6 (UFACTORY)** | 6-DOF cobot-class | Strong SDK + ROS2 + MoveIt | High | Medium | Better for robust industrial-style deployment |
| **Panda / FR3 class arms** | 6-DOF force-aware manipulation | High ecosystem quality | High | Medium-low | Strong research standard for manipulation but usually higher cost |

---

## Gaps to close before committing to large-scale RL

1. Measure a hard latency budget for `/rebotarm/*` service+action round-trips on your host.
2. Pin down if your specific unit ships with Damiao or RobStride branch hardware.
3. Decide fixed action interface early (pose-vs-joint) to avoid interface drift.
4. Define `reset` semantics (home pose + enabled/disabled + estimator state) and enforce in wrapper.
5. If RL throughput matters first, keep a custom C99 C environment contract in front and only validate on hardware in the loop.

---

## Related notes

- [[PufferLib]]
- [[PufferLib C99 Environment Authoring]]
- [[PufferLib Robotics Fit and Limits]]
- [[PufferLib Flight Throughput Benchmark]]
- [[LeRobot]]
- [[ROS2]]
- [[Ascento]]
- [[RAI Ultra Mobility Vehicle]]

---

## Source notes

[^hf-rebot]: Hugging Face LeRobot docs for reBot B601-DM and follower/leader definitions, plus workflow links for teleoperation, calibration, and package references. https://huggingface.co/docs/lerobot/main/rebot_b601
[^wiki-dm]: Seeed Studio reBot B601-DM LeRobot docs include open-source positioning and integration context. https://wiki.seeedstudio.com/rebot_arm_b601_dm_lerobot/
[^wiki-rs]: reBot B601-RS getting started docs: family page and staged content with multi-motor framing. https://wiki.seeedstudio.com/rebot_b601_rs_getting_started/
[^wiki-ros2]: reBot Arm B601-DM ROS2 integration docs: status topics, services, actions, and examples. https://wiki.seeedstudio.com/rebot_arm_b601_dm_ros2_integration/
[^pypi-follower]: `lerobot-robot-seeed-b601` package metadata for hardware variants, motor compatibility, CAN transport, and usage examples. https://pypi.org/project/lerobot-robot-seeed-b601/
[^devarm-hw]: reBot-DevArm open hardware repository (versioned CAD/BOM and motor-variant split between DM and RS). https://raw.githubusercontent.com/Seeed-Projects/reBot-DevArm/main/hardware/reBot_B601_DM/readme.md
[^devarm-readme]: reBot-DevArm project README with license migration and full-stack open-source claim. https://raw.githubusercontent.com/Seeed-Projects/reBot-DevArm/main/README.md
[^seeed-announce]: Seeed official launch announcement: full lifecycle claims from simulation to real-world execution and supported stacks. https://www.seeedstudio.com/blog/2026/04/20/seeed-studio-launches-rebot-arm-b601-a-fully-open-source-robotic-arm-built-for-physical-ai/
