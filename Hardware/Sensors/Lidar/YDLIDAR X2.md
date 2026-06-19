---
title: YDLIDAR X2
aliases:
  - YDLIDAR X2
  - YDLIDAR X2L
tags:
  - lidar
  - robotics
  - sensors
  - rl
---

# YDLIDAR X2

Small legacy-class 2D lidar with classic triangulation design and 360 degree scanning. It is usually the default budget first-step sensor for small platforms that still want full-around obstacle sensing.

---

## Core profile

- Scan geometry: 360 degrees
- Ranging frequency: 3000 points per second
- Motor frequency: about 7 Hz default (wider control range depends on model wiring/firmware)
- Ranging distance: 0.10 m to 8 m (indoor typical)
- Angular resolution: ~0.82 to 0.86 degrees around 7 Hz
- Interface: UART/TX serial at 115200 baud
- Power: 4.8 to 5.2 V, working current around 350 mA
- Form factor: compact ring, typical hobby build footprint

---

## Practical notes for robotics

- Works with fixed scanning loops at low budget; expect stronger tuning requirements than ToF units on reflective surfaces.
- Typical startup behavior is sensitive to motor startup current and noisy USB-UART adapters.
- Many older tutorials assume direct UART packet framing; keep parser strict and drop stale packets instead of extrapolating if timeout occurs.
- Serial bandwidth is lower than some newer ToF units, so run lightweight preprocessing before adding heavy transforms.

---

## Pros

- Very common in hobby and education stacks.
- Cheapest route into 360 degree ring scans among older models.
- Good for obstacle and cliff warning channels when angular resolution can be lower.
- Good historical compatibility in many third-party examples and scripts.

## Cons

- Indoor-only emphasis in datasheet; outdoor sunlit behavior is weaker than newer ToF classes.
- Older generation error behavior and calibration drift on some units.
- Lower range ceiling than LD06/LD19/T-mini Plus.
- Less suitable for very narrow tunnels due to 8 m range cap.

---

## PufferLib / RL wiring pattern

### Suggested scan preprocessing

1. Read raw packet stream in a dedicated UART thread.
2. Convert range to float meters; set NaN for zero/invalid samples.
3. Downsample or interpolate to a fixed observation size (for example `N=72` or `N=128`).
4. Add two scalar channels:
   - `lidar_invalid_ratio`
   - `lidar_min_range`
5. Keep reward shaping simple and geometry heavy (noisy features first).

### Useful RL assumptions

- At 115200 baud, keep your serial poll cadence deterministic (for example 20-50 Hz).
- At lower scan speeds, use ring-time alignment and avoid policy lag by carrying previous valid scan forward.

---

## Comparison chart

| Unit | Range | Angular resolution | Interface | Approx. scan speed | Why choose |
|---|---:|---|---|---|---|
| X2 | ~8 m | ~0.82 deg | UART 115200 | 7 Hz | lowest-cost entry unit |
| X4 | >10 m | ~0.50 deg | UART 128000 | 6-12 Hz | better range/resolution |
| LD06 | 12 m | ~1 deg | UART 230400 | up to 13 Hz | better ambient rejection |
| LD19 | 12 m | ~0.8 deg | UART 230400 | up to 13 Hz | longer range/stronger payload |
| T-mini Plus | 12 m | 0.54 deg | UART | 6-12 Hz | best value among modern mini units |

---

## Sources

- YDLIDAR X2 datasheet PDF mirror: https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/datasheet/application/lidarbot/YDLIDAR%20X2%20Datasheet.pdf
- YDLIDAR X2 product page listing: https://www.robotshop.com/products/ydlidar-x2-360-laser-scanner
- YDLIDAR X2 page snapshot and variant notes: https://www.ydlidar.cn/product/ydlidar-x2

