---
title: YDLIDAR X4
aliases:
  - YDLIDAR X4
  - YDLIDAR X4 Ring
tags:
  - lidar
  - robotics
  - sensors
  - rl
---

# YDLIDAR X4

Mid-range budget 2D spinning lidar with better distance and resolution than the X2 class while still staying in the DIY-friendly category.

---

## Core profile

- Scan geometry: 360 degrees
- Ranging frequency: 5000 Hz
- Scan frequency: 6-12 Hz
- Maximum scanning distance: over 10 m (indoor class in common specs)
- Angular resolution: ~0.48 to 0.52 degrees (around 7 Hz)
- Power: 4.8 to 5.2 V, working current 330-380 mA
- Interface: PH2.0 8-pin connector, UART at 128000 baud
- Drive control: M_EN + DEV_EN + M_SCTP pins for enable and speed control
- Typical weight: compact small-housing unit

---

## Practical notes for use

- Better scan density than older 8 m units at similar price tier.
- One-way and control lines support external motor and ranging control when needed.
- The 8-pin interface and separate enable lines can reduce accidental boot-loop issues if you need deterministic startup.
- UART is 3.3 V logic; use compatible level handling if your MCU uses 5 V.

---

## Pros

- Stronger long-range baseline for indoor maps and obstacle policy.
- Higher point density and wider angular coverage than X2.
- Good stepping point for custom firmware when you want more deterministic motor control.

## Cons

- More current and hardware complexity than 4-pin modules.
- Some boards may expose dead-angle sectors depending on power quality or motor control.
- Driver compatibility can be more uneven than the older simplest UART modules if using legacy scripts.

---

## PufferLib / RL wiring pattern

- Keep `scan_hz` in your observation timestamp model so you can normalize velocities by scan age.
- Build fixed-size beam arrays from the raw ring before forming observations (`n_beams` often 64/128).
- Use `scan_invalid_ratio` as an explicit state channel; models that overfit invalid points can collapse quickly.
- If your motor control line is exposed, avoid changing speed during rollout unless it is a curriculum variable.

---

## Comparison chart

| Unit | Range | Scan speed | Angular resolution | Baud | Best use |
|---|---:|---:|---:|---:|---|
| X4 | >10 m | 6-12 Hz | 0.48-0.52 deg | 128000 | balanced field mapping |
| X2 | 8 m | ~7 Hz | 0.82-0.86 deg | 115200 | lowest-cost baseline |
| T-mini Plus | 12 m | 6-12 Hz | 0.54 deg | UART | modern compact upgrade |
| LD06 | 12 m | up to 13 Hz | 1 deg (typ) | 230400 | compact low-power |
| LD19 | 12 m | up to 13 Hz | ~0.8 deg | 230400 | stronger ambient profile |

---

## Sources

- YDLIDAR X4 datasheet mirror (robotshop cache): https://cdn.robotshop.com/media/y/ydl/rb-ydl-01/pdf/ydlidar_x4_datasheet.pdf
- YDLIDAR X4 listing and docs references: https://www.ydlidar.cn/product/ydlidar-x4
- YDLIDAR X4 manual reference bundle (official site IDs often 404 but referenced in product ecosystem): https://www.ydlidar.com/products

