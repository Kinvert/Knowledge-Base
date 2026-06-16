---
title: SLAMTEC RPLIDAR C1 360 DTOF Laser Scanner
aliases:
  - SLAMTEC RPLIDAR C1 360° DTOF Laser Scanner
  - SLAMTEC RPLIDAR C1 360 Laser Scanner
  - RPLIDAR C1
tags:
  - lidar
  - robotics
  - sensors
  - slam
  - rl
---

# SLAMTEC RPLIDAR C1 360 DTOF Laser Scanner

Use this file as the short-form hub for the Slamtec C1 line (also indexed as
`RPLIDAR C1`) in this KB.

---

## Overview

`RPLIDAR C1` is a compact 2D LiDAR with full 360-degree sweep and a compact consumer-to-prosumer form factor. The class is sold as an entry-level mapper/obstacle sensor with:

- 2D 360° scan geometry,
- modest package size and weight,
- serial-facing integration path through Slamtec SDKs and ROS wrappers,
- and pricing in the hobby-to-small-robot segment.

For balancing-robot RL, the practical value is: fixed-geometry range profiles at low latency.

---

## Practical spec envelope

| Parameter | Typical class |
|---|---|
| Scan geometry | 360° 2D spinning ring |
| Scan sample envelope | high-frequency range sampling (common docs cite ~5 k sample/s class) |
| Scan speed class | single-digit to low-double-digit hertz (model-variant dependent) |
| Range class | ~12 m class |
| Interface | UART/TTL serial in common variants |
| Form factor | small ring module |

This is enough for front-sector profile extraction even when your full 360° scan is overkill.

---

## Where it fits in a balancing RL stack

### For your two-wheel pogo/balance project
- mount horizontally for obstacle ring + front wedge extraction,
- mount vertically for "step/curb height profile" probing,
- subsample to fixed ray count (e.g. 64/128 bins) for policy stability.

### Recommended observation path
1. read packet timestamp,
2. extract fixed sector (e.g., `-45°..+45°`),
3. fill invalid sectors with deterministic placeholder,
4. output fixed-length `float32` array plus stale/validity counters.

This keeps your Puffer contract deterministic and vectorizable.

---

## Puffer-friendly contract

For PPO-like loops, include at least:

- `scan_front` (fixed-length normalized distances),
- `scan_invalid_ratio`,
- `scan_stale_steps`,
- `scan_fov_used_deg`,
- `scan_update_age_ms`.

Do not vary vector size by mode; change mode only via separate scalar channels.

---

## Integration links

- [[RPLIDAR C1]] (legacy note with previous details)
- [[Line Lidar for Balancing Robots]]
- [[Line-Scan Lidar as RL Observation]]
- [[Lidar]]

---

## Source links

- Slamtec C1 product page: https://www.slamtec.com/en/c1
- C1 product docs: https://www.slamtec.com/en/Products/rplidar-c1
- Slamtec data documentation (mirror): https://wiki.slamtec.com
