---
title: Hobbyist Lidar Units
aliases:
  - Hobbyist Lidar
  - Budget Lidar Modules
tags:
  - lidar
  - robotics
  - sensors
  - simulation
  - rl
---

# Hobbyist Lidar Units

This is a practical shortlist for 2D lidar choices in low-cost RL and balancing-robot work.

---

## Selection axis for RL stacks

Choose by:
- whether you need full 360° versus front wedge only,
- tolerance for dropouts,
- serial overhead,
- package size,
- and scan-rate-to-latency budget for your policy loop.

---

## Comparison chart (hobbyist-focused)

| Unit | Typical form factor | Strengths | Risks | Good first fit |
|---|---|---|---|---|
| `RPLIDAR C1` / `RPLIDAR A1M8` | compact 2D ring | low cost + ecosystem support + easy serial setup | more noise on specular/transparent surfaces | proto RL balancing + obstacle sensing |
| `RPLIDAR A2M6` | compact 2D ring, wider scan options | generally stronger scan-performance options | interface tuning can be less beginner-friendly | intermediate prototype stage |
| `STL-06P` family | tiny 2D ring | very small form factor, low mass | 2D only, short-range quirks for some environments | tight payload mounts |
| Mini Pupper class modules | community-vetted integrations | good precedent for compact robotics | ecosystem-specific assumptions | small-legged/balancing crossover projects |
| USB/LiDAR dongle style modules | plug-and-play | simple setup, low dev friction | power/USB contention and latency control | desktop simulation + early data collection |

---

## Recommendation ladder

1. Start with `RPLIDAR A1M8` for fastest debugging and low risk.
2. Move to `RPLIDAR C1` when you want stronger scan throughput and cleaner package messaging.
3. Move to `STL-06P` if payload constraints dominate before range.
4. Keep only one high-end module later when you need consistent high-precision terrain features.

---

## PufferLib-specific advice

For all hobbyist units:
- normalize scan output to fixed length,
- track stale/invalid metrics,
- keep one deterministic preprocessing branch per SKU,
- never let the raw scan length vary with mode.

If you do this, even budget lidars can produce stable PPO baselines.

---

## Sensor stack pairing

- [[Lidar]]
- [[Line Lidar for Balancing Robots]]
- [[Line-Scan Lidar as RL Observation]]
- [[RPLIDAR C1]]
- [[RPLIDAR A1M8]]
- [[LDROBOT STL-06P]]
- [[Mini Pupper STL-06P Lidar Module]]
- [[SLAMTEC RPLIDAR C1 360 DTOF Laser Scanner]]
