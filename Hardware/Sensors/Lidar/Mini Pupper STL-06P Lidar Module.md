---
title: Mini Pupper STL-06P Lidar Module
aliases:
  - Mini Pupper STL-06P Lidar Module
  - LDROBOT STL-06P
  - STL-06P
tags:
  - lidar
  - robotics
  - sensors
  - mini-pupper
  - rcl
---

# Mini Pupper STL-06P Lidar Module

Use this note as the project-facing name for the STL-06P module used in many
mini-robot stacks, including Mini Pupper ecosystem content.

---

## Why you care

For balancing and short-horizon navigation prototypes, STL-06P is often chosen for:
- tiny physical envelope,
- small mass,
- ROS-friendly community references,
- and a clean UART interface with PWM spin control path in common hardware revisions.

---

## Core characteristics

| Characteristic | Notes |
|---|---|
| Scan style | 2D ring |
| Scan frequency | 6–13 Hz scan-rate class |
| Range sample class | very high ranging frequency compared with package size |
| Range class | roughly up to 12 m |
| Interface | UART + external control pin behavior in v1.3-class modules |
| Mass / size | small; compact for legged/compact builds |

---

## Where it is stronger than a bigger 2D module

- Mechanical fit for narrow mast/e-stop spacing.
- Lower wiring complexity in small baseboards.
- Faster mechanical packaging in custom enclosures.

For raw range noise and spec depth, it is still in hobby-to-low-mid grade class.

---

## Relation to balancing-robot RL

For your 2DOF pogo/balance idea, STL-06P is suitable if you:
- prioritize compact mechanics over raw outdoor range class,
- keep sensor fusion simple (IMU + fixed front sector),
- and want predictable startup behavior under embedded constraints.

---

## See also

- [[LDROBOT STL-06P]] (existing note with detailed package values)
- [[Line Lidar for Balancing Robots]]
- [[Line-Scan Lidar as RL Observation]]
- [[Lidar]]

## Sources

- LDROBOT v1.3 datasheet: https://www.ldrobot.com/images/2023/03/02/LDROBOT_STL-06P_Datasheet_EN_v1.3_txOyicBl.pdf
- Mini Pupper feature docs: https://minipupperdocs.readthedocs.io/en/latest/guide/Features.html
- SensorLidar commerce page summary (listing context): https://www.sensorlidar.com

