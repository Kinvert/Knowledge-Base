---
title: reBot Arm B601
aliases:
  - reBot B601
  - reBot-DevArm
  - Seeed reBot
  - reBot Arm B601 DM
  - reBot Arm B601 RS
tags:
  - robotics
  - robot-arm
  - open-hardware
  - lerobot
  - ros2
  - pufferlib
---

# reBot Arm B601

**reBot Arm B601** is a two-branch open-stack manipulator from Seeed that is built for embodied AI workflows and policy data collection.  
In practice it is one of the few low-cost arms that combines open CAD/BOM, commercial-quality stack depth, and a direct path toward LeRobot workflows.

---

## Overview

This is usually split into two hardware branches:

- `DM` branch (Damiao motor integration).
- `RS` branch (RobStride motor integration).

The project is positioned as a developer-first platform that includes mechanical design files, BOM, software examples, and a documented control path from assembly through perception and imitation learning.

---

## Why this is a top candidate for your objective

- It is open-hardware-first compared with most hobby kits.
- Core docs explicitly include CAD/BOM + simulation-ready integration for LeRobot and ROS2.
- It is affordable as a full starter stack compared with enterprise systems, with Seeed presenting it as targeting under-$1,000 workflows in public positioning.
- The motor stack supports multiple actuator families via the same SDK philosophy, which helps if you want to experiment with cost/performance tradeoffs.

---

## Hardware, openness, and licenses

- The repository is explicit about open design artifacts:
  - hardware blueprints for sheet metal and printed parts,
  - full BOM details and references,
  - open-source licensing for hardware and software components.
- License status indicates **CERN-OHL-W 2.0** for hardware and **Apache-2.0** for software, with the shift from non-commercial terms already communicated in the project updates.
- The reBot roadmap shows concrete published milestones for:
  - STEP/BOM opens,
  - ROS2 integration,
  - Pinocchio support,
  - and Isaac Sim import status updates.
- Reported hardware baseline values in project docs include:
  - 6-DOF + 1 gripper,
  - 650 mm reach,
  - less than 1.5 kg recommended payload,
  - repeatability under 0.2 mm,
  - 4.5 kg platform mass,
  - 24 V supply.

---

## Software and control APIs

- `Rebotarm` Python SDK + ROS2 integration is called out as the official pathway for action/state bridging.
- The ROS2 integration guide describes wrapping the low-level Python control layer into ROS2 topics, services, and actions for higher-level planner integration.
- The same stack is also tied to LeRobot-style toolchains and Pinocchio for kinematics workflows.
- Practical control paths for RL are therefore:
  - action -> wrapper -> joint / pose command -> ROS2 action/service,
  - telemetry observation pull from status + joint feedback.

---

## Community and language footprint

- The repository appears actively maintained and uses common contributor workflows.
- Language support is practical for modern teams:
  - Python examples are core,
  - ROS2 launch and API usage patterns are explicit,
  - the ecosystem exposes integration points for CV and data pipelines.
- The project also targets beginner-to-advanced onboarding with guide structure and quick-start assets.

---

## PufferLib integration strategy

Use this as a hardware target with a host adapter layer:

1. Normalize action vector as joint deltas or Cartesian pose deltas.
2. Use a fixed control cadence in the adapter.
3. Build deterministic `reset(seed)` logic:
   - power/homing handshake,
   - mode state check,
   - optional controller sync.
4. Build observations from:
   - joint positions/velocities,
   - gripper state,
   - mode/safety bits.
5. Keep reward and done logic fully in wrapper code.

Throughput expectation:
- Good for research validation and project demos.
- Not likely to beat true simulator FPS without a surrogate environment due transport and hardware latencies.

---

## Related projects with reusable concepts

- [[LeRobot]]
- [[ROS2]]
- [[PufferLib]]
- [[PufferLib C99 Environment Authoring]]

---

## Pros and Cons

Pros:
- Open hardware + software stack with BOM/CAD clarity.
- Stronger documentation path for sim-to-real projects than most cheap hobby kits.
- Good candidate for building a clean RL deployment branch with a deterministic wrapper.

Cons:
- Not a high-FPS native RL environment.
- Runtime stack can be complex if mixing DM and RS branch docs.
- Final project speed depends more on transport stability than pure policy code performance.

---

## Comparison with 5+ peers

| Platform | Open stack depth | ROS2/LeRobot support | Typical total cost | Ease of setup | HIL throughput |
|---|---|---|---:|---|---|
| reBot Arm B601 | High | Medium-High | Medium | Medium | Medium-Low |
| SO-ARM100 / SO-ARM101 | High | High | Low-Medium | Medium | Low-Medium |
| Interbotix WidowX-250 6DOF | Medium | High (ROS2) | Medium | Medium-High | Medium |
| OpenMANIPULATOR-X | Medium | High (ROS2) | Low-Medium | Medium | Medium |
| Poppy Ergo Jr | Medium | Medium (ROS1-oriented path) | Very Low | Medium | Low |
| Franka Emika Panda | Low | High | Very High | Medium | Medium |

---

## External resources

- reBot announcement and positioning: https://www.seeedstudio.com/blog/2026/04/20/seeed-studio-launches-rebot-arm-b601-a-fully-open-source-robotic-arm-built-for-physical-ai/
- reBot DevArm repository: https://github.com/Seeed-Projects/reBot-DevArm
- reBot B601 DM docs: https://wiki.seeedstudio.com/rebot_arm_b601_dm_lerobot/
- reBot B601-RS docs: https://wiki.seeedstudio.com/rebot_arm_b601_rs_getting_started/
- reBot B601-DM ROS2 integration: https://wiki.seeedstudio.com/rebot_arm_b601_dm_ros2_integration/
- ReBot license notice in repo readme: https://github.com/Seeed-Projects/reBot-DevArm#rebot-devarm-project-license
