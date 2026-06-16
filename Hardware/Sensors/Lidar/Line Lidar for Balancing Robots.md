---
title: Line Lidar for Balancing Robots
aliases:
  - Line Lidar Balancing
  - Line-scan Lidar RL
  - Lidar Profile RL
tags:
  - lidar
  - robotics
  - reinforcement-learning
  - control
  - sim2real
---

# Line Lidar for Balancing Robots

You asked for a robot that uses **IMU + line-scan lidar** with two wheels and a pogo-like springing element. This note is a practical contract for that design.

## Core idea

Instead of feeding full 360° point clouds, treat the lidar as a **front profile sensor**:

- mount the ring **vertical** (or tilt up/down) and extract a fixed window (for example `−45°..+45°`),
- project to a compact obstacle profile,
- feed that profile as fixed-length RL observation.

This is lower bandwidth than full image observation, while giving terrain cues not visible in pure IMU.

---

## Observation encoding pattern for Puffer-style policies

### Recommended fixed vector (example for 64 rays)

```text
obs = [
  # motion state (8–12 float)
  imu_gyro_xyz,
  imu_accel_xyz,
  pitch_angle,
  wheel_speed,
  motor_current_proxy,

  # lidar profile (64 float)
  scan_front_64_beams_distance,
  scan_front_64_beams_validity
]
```

Use two vectors if you need deterministic memory layout:
- `scan_range` as float32
- `scan_valid` as uint8 bit/byte channels (packed if needed)

### Derived features (optional, improves learning speed)

- `min(range_window)`,
- `p10`, `p50`, `p90`,
- `slope_5deg_diff`, `roughness_std`,
- `invalid_ratio`.

These can replace raw scan in later curricula.

---

## Why vertical orientation is interesting

For your pogo/2-DOF bot:

- Front-to-back slope and step rise are more detectable with a forward tilt.
- You can derive "cliff risk" from sudden range jumps in first bins.
- Wheel-slip policy can include terrain-awareness without adding a camera.

You lose some lateral obstacle context unless you keep a side sweep or dual scans.

---

## How to map to reward

Potential additive reward terms:

- `+` forward progression,
- `-` absolute roll/pitch deviation,
- `-` invalid lidar ratio,
- `-` minimum range below crash threshold.

Also add termination on:
- fall (IMU + joint geometry),
- unsafe tilt,
- repeatable collision pattern.

---

## Control architecture with estimator stack

A stable stack for this project is:

1. IMU/LQR baseline for local balance.
2. Lidar profile policy for terrain adaptation.
3. safety fallback to recover posture if invalid-rate exceeds threshold.

This matches how Ascento and UMV were interpreted for RL integration: use deterministic estimator/control baseline and let policy add higher-level terrain/intent decisions.

---

## References

- [[Ascento]]
- [[RAI Ultra Mobility Vehicle]]
- [[PufferLib]]
- [[PufferLib C99 Environment Authoring]]
- [[Extended Kalman Filter]]
- [[Linear Quadratic Regulator]]
- [[LDROBOT STL-06P]]
- [[RPLIDAR A1M8]]
- [[RPLIDAR C1]]

