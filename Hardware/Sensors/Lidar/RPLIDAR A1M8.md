---
title: RPLIDAR A1M8
aliases:
  - RPLIDAR A1M8
  - A1M8
  - Slamtec A1
tags:
  - lidar
  - robotics
  - sensors
  - slam
---

# RPLIDAR A1M8

## Overview

The **RPLIDAR A1M8** is a low-cost 2D rotating range scanner class in the SLAMTEC portfolio.

For RL robotics, it has three reasons to stay relevant:

- simple 360° geometry,
- modest power and small mechanical size,
- broad legacy support path (serial + possible USB variants).

---

## Documented technical profile

From the official SLAMTEC A1M8 datasheet:

- 360° scan capability and 12 m nominal class max range.
- Scan frequency configurable up to around **10 Hz**.
- Sampling frequency capability above **8000 points per second**.
- Angular output resolution not fixed at a single value due distance-dependent triangulation behavior.
- Communication via **3.3V TTL UART** in base variant, with other interfaces (including USB variants) available by customization.
- Typical operating power class in the small 5V+ motor split architecture.

---

## Protocol and integration behavior

### Data model
- Data frames include per-point distance, angle, quality/health-like fields and scan-start flags.
- Useful for partial front-profile extraction because you can select any sector after deserialization.

### Power and interface
- The datasheet explicitly describes a dual power split between range and motor subsystems.
- The UART path is convenient for embedded controllers, while USB variants are common in dev setups.

### Reliability notes
- Low-cost class for robotics education/prototyping workloads.
- You should expect stronger filtering/cutoff strategy than premium scanners for production use.

---

## When to choose this over C1 / STL-06P

- Pick A1M8 if you need a known-tradition "classic" SLAMTEC package with simple serial semantics and broad legacy integrations.
- Pick C1 if you want higher data throughput claims and SL-DTOF ecosystem positioning.
- Pick STL-06P if your priority is very compact form factor and lower mass.

---

## Useful for balancing robot workflows

- Good low-cost option for early RL experiments.
- Stable enough for **front-sector profile RL** tasks.
- Works with a fixed-rate transform:
  - input sample queue -> select `±60°` window,
  - compress to 60 rays,
  - pass to policy.

---

## PufferLib integration note

Use fixed-shape scan vectors at each step to avoid variable-length obs explosions.

Good action interface pattern:

```text
obs = [
  imu_gyro_xyz,
  imu_accel_xyz,
  imu_pitch, imu_roll,
  scan_front_60x1,
  scan_invalid_mask_count,
  speed_feedback,
  wheel_angle
]
```

This is still light enough for C99-step kernels if preprocessing is outside the high-frequency kernel.

---

## Sources

- A1M8 datasheet (mirror): https://wiki.slamtec.com/download/attachments/83066883/LD108_SLAMTEC_rplidar_datasheet_A1M8_v3.0_en.pdf?api=v2&modificationDate=1677814844081&version=1
- A1M8 datasheet mirror variant: https://wiki.slamtec.com/download/attachments/83066883/LD108_SLAMTEC_rplidar_datasheet_A1M8_v3.0_en.pdf?api=v2&modificationDate=1677786044000&version=1
- SLAMTEC support docs hub (SDK + ROS/ROS2 links): https://www.slamtec.com/en/support

