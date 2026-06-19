---
title: YDLIDAR T-mini Plus
aliases:
  - T-mini Plus
  - YDLIDAR T-mini Plus
tags:
  - lidar
  - robotics
  - sensors
  - rl
---

# YDLIDAR T-mini Plus

Modern compact ToF model aimed at hobby and service robots that want 12 m range with small package size and lower blur under changing lighting.

---

## Core profile

- Scan geometry: 360 degrees
- Ranging distance: 0.05 to 12 m (80% target class)
- Scan frequency: 6-12 Hz
- Ranging frequency: 4000 Hz
- Angle resolution: around 0.54 degrees
- Accuracy (typical): ±20 mm over 0.05 to 12 m
- Supply: 4.8 to 5.2 V
- Current: around 340 mA typical
- Interface: UART
- Dimensions: around 39 x 39 x 34 mm
- Ambient tolerance: around 60,000 lux stated in public listings
- Temperature envelope: around -10 to +45 C

---

## Why this is a practical budget choice

- Better short-range floor distance capture than 8 m legacy units.
- Compact enough for narrow mounts while offering stronger max range.
- More modern scan density than older triangulation models at similar footprint.
- Still simple enough for spinning-lidar RL integration without exotic drivers.

---

## Pros

- Good "small package + 12 m range" balance for mobile bots.
- Higher angular resolution than many legacy consumer units in the same price band.
- Usually supported by YDLIDAR software toolchains and docs.

## Cons

- Documentation is fragmented across reseller and manufacturer pages.
- Not all low-cost resellers keep updated firmware bundles.
- If you buy older revision clones, compatibility can vary by firmware.

---

## PufferLib / RL notes

- Use one fixed scan preprocessing branch:
  - convert units to meters
  - clip outlier returns to min/max range envelope
  - compute `sector_min`, `sector_std`, `sector_trend`
- Because `4000 Hz` sample rates can produce denser packets at same RPM, use deterministic downsampling rather than every-packet ingestion.
- Add a "stale scan age" gate if your loop runs faster than the lidar refresh rate.

---

## Comparison chart

| Unit | Range | Angle resolution | Scan speed | Interface | Best fit |
|---|---:|---:|---|---|---|
| T-mini Plus | 12 m | 0.54 deg | 6-12 Hz | UART | balanced upgrade |
| LD06 | 12 m | ~1 deg | up to 13 Hz | UART @230400 | low-power compact variant |
| LD19 | 12 m | ~0.8 deg | up to 13 Hz | UART @230400 | larger payload, similar style |
| X4 | >10 m | 0.48-0.52 deg | 6-12 Hz | UART @128000 | more control pins |
| STL-06P | 12 m | ~1 deg | around 10 Hz | UART @230400 | tiny proven mini robotics |

---

## Sources

- YDLIDAR T-mini Plus Chinese product page: https://www.ydlidar.cn/product/ydlidar-t-mini-plus
- Public distributor spec page with voltage/current and feature list: https://www.soselectronic.com/en-us/products/ydlidar/t-mini-plus-364789
- YDLIDAR catalog style entry (snapshot): https://www.ydlidar.com/products

