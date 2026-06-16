---
title: RPLIDAR C1
aliases:
  - RPLIDAR C1
  - Slamtec C1
tags:
  - lidar
  - robotics
  - sensors
  - slam
---

# RPLIDAR C1

## Overview

The **RPLIDAR C1** is a small 2D lidar from SLAMTEC targeted at consumer/indoor robots and embedded mapping. It is listed as a fusion-style time-of-flight design (SL-DTOF wording in vendor marketing) and is intended for navigation and obstacle avoidance.

The main value for your projects is the combination of:

- full-circle scan geometry,
- reasonably small package,
- official driver chain via SDK and ROS support ecosystem (from Slamtec docs).

---

## What is claimed in source docs

From the official C1 datasheet and product pages:

- 360° scan with about **12 m max range class** (product-level spec wording).
- high throughput orientation and **5,000 times/s** scan sample characteristics in docs.
- lightweight and compact enclosure, with **~110 g** weight and ~55.6 mm square form from spec page.
- **UART-based comms in default form factor** with a standard XH2.54-5P connector.

Source excerpts and docs:

- Datasheet core claims and interface definitions: SLAMTEC product data and wiki mirrors.
- Product spec page values and model dimensions from SLAMTEC.

---

## Integration details

### Electrical
- Typical power input around 5V class for core operation.
- XH2.54-5P physical interface (UART + power pins) with motor behavior depending on host power quality.

### Communications
- Command/data path is serial-style from core docs.
- Scan control and data frames are documented in the official protocol/material pack.

### Host stack
- Slamtec SDK and RoboStudio mention platform compatibility in official docs.
- This is the key practical point for sim2real: you can treat it as a standard serial sensor front-end and avoid heavy packet parsing in your RL loop by preprocessing into fixed bins.

---

## Practicality for balancing robot + 2DOF experiments

### Good uses
- Forward obstacle detection for fall prevention (single-step/short horizon policy).
- Terrain break detection (`min range` drop + variance) used as an auxiliary terrain-risk term.
- Works with one forward-looking band in your action policy so PPO can learn to balance while avoiding edges.

### Risks
- Glass/wide specular surfaces can create invalid sectors.
- Must tune motor power budget; under-voltage causes scan jitter.
- ROS integration can add latency unless you isolate serial read thread from policy thread.

---

## Ascento / UMV / sim2real relevance

- Useful as a realistic, cheap sensor layer for the “wheeled balancing with jumps/steps” class of robots.
- Good for your **ROS + PufferLib** path because data semantics are straightforward: fixed-length range vector + validity.
- Better for local obstacle policy shaping than raw image stacks when you want compact RL input.

---

## Comparison to your other options

| Option | 360° scan | Range | Weight / size | Notes |
|---|---|---:|---|---|
| RPLIDAR C1 | Yes | ~12 m class | 110 g, small | Good baseline for general nav and obstacle RL |
| RPLIDAR A1M8 | Yes | up to 12 m | slightly larger legacy class | Larger ecosystem maturity in older ROS setups |
| RPLIDAR A2M6 | Yes | up to 16–18 m (model variant) | small/compact | higher max scan rate mode options |
| STL-06P | Yes | up to 12 m | 45 g, tiny | very compact + Mini Pupper-ready |

---

## Puffer integration checklist

1. Keep a single canonical `scan_to_obs(scan)` transform with bounded NaN handling.
2. Maintain fixed output length with deterministic ring downsampling (`N=64` or `N=128`).
3. Track `scan_rate`, `dropped_scan`, `invalid_ratio` and feed those as optional debug channels.
4. If using `step()` at 50 Hz, gate sensor read at the same cadence and hold previous scan if no new packet.

---

## Sources

- SLAMTEC C1 datasheet mirror (official PDF): https://www.slamtec.com/en/c1/spec
- SLAMTEC C1 product page: https://www.slamtec.com/en/c1
- C1 datasheet mirror with interface and safety/spec details: https://wiki.slamtec.com/download/attachments/83066883/SLAMTEC_rplidar_datasheet_C1_v1.0_en.pdf?api=v2&modificationDate=1700531479533&version=1

