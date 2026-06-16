---
title: SO-ARM100
aliases:
  - SO-100
  - TheRobotStudio SO-100
  - SO-ARM101
tags:
  - robotics
  - robot-arm
  - open-source
  - lerobot
  - ros2
  - pufferlib
---

# SO-ARM100

**SO-ARM100** is a low-cost open hardware arm family from TheRobotStudio that is heavily used in educational and embodied-AI prototypes.  
In practice it is one of the cheapest paths into LeRobot-style imitation and robotics-control projects.

---

## Overview

The public repository describes `SO-100` and `SO-101`, with SO-101 positioned as the newer branch while SO-100 remains referenced for compatibility.  
For projects that care more about speed and budget than absolute stiffness, this family is still one of the best value entry points.

---

## Why this is a top candidate

- Open repository with published CAD/BOM and step-by-step assembly guidance.
- Designed around mainstream LeRobot patterns for data collection, calibration, and policy iteration.
- Very active community context from Seeed docs and Hugging Face tooling.
- Cheapest route among these options if you need physical robotics experience fast.

---

## Hardware and openness

- Fully open-source orientation:
  - STL and STEP assets in the repo,
  - BOM and part references,
  - assembly and calibration documentation.
- The documented motor option is standard STS3215 with explicit voltage/torque variants, with 7.4 V as common and stronger 12 V option for heavier loads.
- The repo has explicit sourcing options and vendors for kits/parts across regions.
- It is best treated as a **research-first** rather than heavy-duty manufacturing arm.

---

## Software stack and integration pathways

- The repository and docs are organized around LeRobot compatibility.
- There are separate setup flows for:
  - cloning/assembling,
  - calibration,
  - teleoperation,
  - and policy training.
- The project supports the common learning pattern:
  - collect demonstration data,
  - train with ACT-style workflows,
  - replay and validate policies.

---

## Community and language support

- The docs route points to active examples and multiple support channels (Discord, GitHub discussions in related ecosystems).
- The practical language profile is strong for Python-first stacks, with command patterns that are easy for RL research teams to wrap.
- Because it is positioned as an educational/entry platform, third-party examples are relatively easy to find.

---

## PufferLib integration strategy

1. Keep the environment contract host-side:
   - deterministic action scale,
   - bounded position/velocity commands,
   - explicit homing.
2. Pull state channels from joint data and status bits at fixed tick intervals.
3. Handle resets and episode state in wrapper code, not inside robot firmware behavior.
4. Use a surrogate environment first for dense training; validate final checkpoints on hardware.

This keeps throughput practical while still giving genuine hardware grounding for sim2real.

---

## Cost and setup considerations

- The repo documentation includes concrete unit examples for core servos (for example STS3215 price points), indicating a budget-focused profile.
- Mechanical and calibration complexity is not zero:
  - cable/power routing,
  - calibration passes,
  - and host setup are part of the baseline work.
- Still one of the cheapest viable options for 6-DOF + gripper RL testing.

---

## Related concepts

- [[LeRobot]]
- [[ROS2]]
- [[PufferLib]]
- [[PufferLib C99 Environment Authoring]]

---

## Pros and Cons

Pros:
- Lowest total-entry cost for 6-DOF data-collection workflows.
- Open files reduce custom-fab friction.
- Large base of example-based onboarding.

Cons:
- Lower stiffness/precision than mid-tier industrial stacks.
- SO-100/SO-101 overlap can confuse buyers; prefer recent docs.
- Native throughput is usually bound by transport and command latency.

---

## Comparison with 5+ peers

| Platform | Open files quality | LeRobot alignment | Cost | Setup burden | PufferLib compatibility |
|---|---|---|---:|---|---|
| SO-ARM100 / SO-ARM101 | High | High | Very Low | Medium | Medium |
| reBot Arm B601 | High | Medium-High | Medium | Medium | Medium |
| Interbotix WidowX-250 6DOF | Medium | Medium | Medium | Medium-High | Medium |
| OpenMANIPULATOR-X | Medium | Medium-High | Low-Medium | Medium | Medium |
| Poppy Ergo Jr | Medium | Medium | Very Low | Medium | Low |
| xArm 6 (budget line) | Medium | Medium | Medium-High | Low-Medium | Medium |

---

## External resources

- SO-ARM100 repository: https://github.com/TheRobotStudio/SO-ARM100
- Seeed SO-10x setup tutorial: https://wiki.seeedstudio.com/lerobot_so100m_new/
- SO-ARM docs (legacy): https://wiki.seeedstudio.com/lerobot_so100m/
- SO-100 / SO-101 hardware and parts details: https://github.com/TheRobotStudio/SO-ARM100
