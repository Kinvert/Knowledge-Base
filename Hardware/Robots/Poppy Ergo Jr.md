---
title: Poppy Ergo Jr
aliases:
  - Poppy Ergo
  - Poppy Ergo Jr
tags:
  - robotics
  - robot-arm
  - open-hardware
  - ros
  - python
  - pufferlib
---

# Poppy Ergo Jr

**Poppy Ergo Jr** is a compact, low-cost 6-DOF educational arm designed for quick iteration and high customizability.  
It is one of the most approachable open projects if your goal is fast onboarding and visible demos.

---

## Overview

The project is strongly associated with:
- modular 3D-printable mechanics,
- Dynamixel XL-320 based actuation,
- and Python-based motor control (`pypot`) workflows.

It is still a useful option when the target is rapid prototyping and education-oriented proof-of-concept RL smoke tests.

---

## Why this is a top candidate

- Very low entry cost and approachable BOM.
- Strong physical modifiability:
  - tool swapping with simple fixture design,
  - end-effector swaps in a few minutes,
  - easy mechanical rework cycles.
- Community materials include explicit assembly guides and installation docs.

---

## Hardware and openness

- Repository and docs describe a 6-degree-of-freedom architecture with interchangeable tool heads.
- Mechanical design is intentionally accessible:
  - 3D printed structure,
  - quick mechanical reconfiguration.
- It supports rapid customization for portfolio projects and classroom-style experiments.
- The platform is better for early-stage manipulation research than precision-heavy robotics.

---

## Software, integration, and language support

- Core control path centers on Python:
  - pypot library for Dynamixel communication,
  - robot configuration / command helpers,
  - movement primitives and motion recording.
- REST API support exists for broader tooling integration beyond Python.
- ROS support exists for specific versions and is mostly tied to ROS1 Noetic in current documentation.
- This matters for your job-focused projects: if you need ROS2-first stack, you may spend more setup time.

---

## PufferLib integration strategy

1. Use an adapter that publishes simplified state:
   - motor positions,
   - target setpoint status,
   - simple safety flags.
2. Wrap all policies as constrained joint deltas or pose deltas.
3. Keep reset logic explicit:
   - zero pose,
   - servo compliance/tension check,
   - deterministic "ready" gate.
4. Emphasize smoke tests:
   - short horizon,
   - robust domain randomization,
   - conservative speed/torque limits.

Use a simulator or lightweight surrogate early, then validate on hardware.

---

## Community and maintenance profile

- Useful docs remain available for:
  - installation,
  - motor configuration,
  - assembly,
  - ROS compatibility checks,
  - forum support.
- Because this ecosystem is older than some newer ROS2-first stacks, expect mixed freshness across examples.

---

## Related concepts

- [[ROS]]
- [[ROS2]]
- [[PufferLib]]
- [[PufferLib C99 Environment Authoring]]

---

## Pros and Cons

Pros:
- Best entry-cost choice for fast proof-of-concept cycles.
- Excellent for physical iteration and educational demos.
- Very straightforward mechanical customization path.

Cons:
- ROS2 support is limited in current docs.
- Precision and repeatability limits are more significant than pricier arms.
- Throughput can be inconsistent for strict RL benchmarking loops.

---

## Comparison with 5+ peers

| Platform | Cost | Mechanical customization | Software maturity | ROS path | RL onboarding speed |
|---|---:|---|---|---|---|
| Poppy Ergo Jr | Very Low | Very High | Medium | ROS1-focused | Very Fast |
| SO-ARM100 / SO-ARM101 | Low-Medium | High | High | ROS2 + LeRobot path | Fast |
| reBot Arm B601 | Medium | High | High | ROS2 + LeRobot | Medium |
| Interbotix WidowX-250 | Medium | Medium | Very High | ROS2 | Medium |
| OpenMANIPULATOR-X | Low-Medium | Medium | High | ROS2-first | Medium |
| Franka Emika Panda | Very High | Medium | Very High | ROS2/ROS1 | Slower |

---

## External resources

- Poppy Ergo Jr repository: https://github.com/poppy-project/poppy-ergo-jr
- Assembly guide: https://docs.poppy-project.org/en/assembly-guides/ergo-jr/
- Pypot documentation: https://docs.poppy-project.org/en/software-libraries/pypot
- ROS guide (Poppy Ergo Jr): https://docs.poppy-project.org/en/programming/ros
- Community forum: https://forum.poppy-project.org
