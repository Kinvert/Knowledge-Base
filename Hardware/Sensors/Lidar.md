---
title: Lidar
aliases:
  - LiDAR
  - Lidar Sensor
tags:
  - sensors
  - robotics
  - lidar
  - perception
---

# Lidar

## Overview

**Lidar (LiDAR)** measures distance by sending laser pulses and detecting return timing/phase. For mobile robotics and RL, it is useful when:

- you need structured geometry from sparse observations,
- you want low-latency obstacle feedback (front-wall and stair/step detection),
- you need observation spaces beyond basic pose/IMU states.

This note is written around your use case: a balancing wheeled robot with a forward-facing 2D scan and RL control.

---

## Why this matters for your stack

For PufferLib sim2real workflows, lidar can be used as a first-class observation source:

- `obs`: compressed scan profile (selected rays), min-depth features, rough slope metrics.
- `reward`: progress minus collision risk and roughness penalties.
- `done`: crash/contact + excessive tilt + estimator confidence drops.

You get richer terrain inference than IMU-only or GPS-like inputs, while still keeping `obs` fixed-length for vectorized envs.

---

## Common lidar families for your project

## Comparison (compact)

| Candidate | Scan geometry | Typical scan/data rate | Range class | Mounting profile | ROS / integration fit |
|---|---|---:|---|---|---|
| [[SLAMTEC RPLIDAR C1 360 DTOF Laser Scanner]] | 2D, 360° | 5000 samples/s (high-speed mode) | ~12 m radius | 55.6 × 55.6 × 41.3 mm, 110 g | Slamtec SDK + documented APIs + ROS bridges |
| [[RPLidar A1M8]] | 2D, 360° | up to 8000 samples/s and 10 Hz scan | 12 m class | legacy-style small ring, serial/USB | Official SDK + ROS wrappers |
| [[RPLIDAR A2M6]] | 2D, 360° | up to 8000 samples/s and 15 Hz scan | up to 16–18 m | small industrial 2D form | broader scan modes (legacy/express/boost) |
| [[Mini Pupper STL-06P Lidar Module]] | 2D, 360° | 5000 Hz ranging, 6–13 Hz scan | 12 m class (0.03 m minimum) | compact 38.6 mm square class, 45 g | UART only in v1.3, used in Mini Pupper ecosystem |
| [[Line Lidar for Balancing Robots]] | 2D ring repurposed as quasi-line front profile | depends on motor speed and sample mode | short-to-mid range | mount vertically/horizontally | custom preprocess step needed for policy obs |
| [[Hobbyist Lidar Units]] | 2D lidar family | varies by module | budget class | compact to hobby-grade | use this family overview for first-time architecture choices |

For this exact 2DOF-balance idea, [[RPLidar A1M8]], [[SLAMTEC RPLIDAR C1 360 DTOF Laser Scanner]], and [[Mini Pupper STL-06P Lidar Module]] are the practical starters because they are compact and well-supported in early RL stacks.

---

## Key design tradeoffs for balancing robots

- **Throughput vs payload:** You usually run the same sensor at lower scan rates (e.g., 5–10 Hz) and subsample down to 60–360 rays per observation.
- **Field-of-view strategy:** Don’t keep raw 360° unless your policy needs it. For forward-balance and stair-edge work, a forward wedge (`±40°` etc) is often better.
- **Data hygiene:** lidar returns include invalid/NaN sectors near reflective glass or glass floors. Track invalid-count and confidence channels for robust reward shaping.
- **Coordinate frame discipline:** define frame orientation clearly (`scan 0°` forward/downward vs wall-clocking). It matters for PPO policy reuse and deterministic policy distillation.

---

## Integration patterns with PufferLib

### Pattern 1: Raw fixed-length lidar vector

- Convert scan packets to fixed bins (e.g., 64 or 128 ranges).
- Include per-bin validity bits or `1/range` for invalids.
- Keep action and state update rate deterministic (e.g., 50 Hz policy, 100–200 Hz local control).

### Pattern 2: Feature-compressed lidar features

- Keep `min_k` (minimum distance), `median`, `p10`, `roughness`, `cliff_index`, `slope_grad`.
- Good for stability, less bandwidth, often faster training.

### Pattern 3: Curriculum gating

- Start in narrow obstacle fields with synthetic scans.
- Progress to mixed slopes and glass-like targets.
- Track invalid-rate and add safe-policy overrides during high-noise events.

---

## Related to your current projects

- [[Ascento]]
- [[RAI Ultra Mobility Vehicle]]
- [[Line Lidar for Balancing Robots]]
- [[Extended Kalman Filter]]
- [[Linear Quadratic Regulator]]
- [[PufferLib]]
- [[PufferLib C99 Environment Authoring]]
- [[PufferLib Robotics Fit and Limits]]

---

## Further links

- [[RPLIDAR C1]]
- [[RPLidar A1M8]]
- [[RPLIDAR A2M6]]
- [[Mini Pupper STL-06P Lidar Module]]
- [[Hobbyist Lidar Units]]
