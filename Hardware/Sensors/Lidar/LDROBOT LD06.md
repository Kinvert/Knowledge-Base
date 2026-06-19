---
title: LDROBOT LD06
aliases:
  - LD06
  - LDROBOT LD06
tags:
  - lidar
  - robotics
  - sensors
  - rl
---

# LDROBOT LD06

Compact DTOF 2D lidar family member commonly used in mini-legged and compact mobile robots. It keeps weight low while matching the 12 m class range target.

---

## Core profile

- Scan geometry: 360 degrees
- Super mini ToF architecture with 5000 Hz sampling
- Measurement range: 0.02 to 12 m (70% reflectivity target)
- Scan speed: typically around 10 Hz at 40% PWM duty, 5-13 Hz with control range
- Angular resolution: around 1 degree (typical in community usage)
- Accuracy: around 10 mm standard deviation on 12 m envelope; better in controlled lighting
- Power: 4.5 to 5.5 V, start current ~300 mA, running ~180 mA
- Interface: 4-pin ZH1.5T style
  - UART@230400
  - PWM motor pin
  - GND and P5V
- Form factor: ~38.6 x 38.6 x 33.5 mm, ~42 g
- Ambient: around 30 K lux spec in datasheet

---

## Why it is used in hobby stacks

- Tiny package is easy to mount on narrow chassis and legged rigs.
- Better short-to-mid range than X2-class triangulation units.
- Stable enough for cliff + obstacle RL signals when scan is preprocessed into fixed-size rays.
- Often used as a direct drop-in around Mini Pupper style platforms.

---

## Pros

- Small form factor and low mass.
- UART@230400 avoids very low-speed serial bottlenecks.
- Good community footprint for small ROS/mobile builds.
- Better angular sample count stability than very old legacy units.

## Cons

- Lower advertised ambient tolerance than some competitor variants.
- Some implementations expose protocol quirks across clone batches.
- You often need power cleanliness to avoid startup jitter.

---

## PufferLib / RL integration

### Recommended observation recipe

1. Capture fixed-rate raw ranges from each scan.
2. Fill invalid bins and carry old ranges only when timestamp drift is bounded.
3. Extract both geometry channels:
   - `scan_minmax`
   - `scan_mean` over sectors
4. Add `scan_invalid_ratio` + `motor_speed_state` if you control PWM.

If you see noisy transitions from frame to frame, increase scan buffer and compute moving-window features (`p10/p90`) before feeding the policy.

---

## Comparison chart

| Unit | Range | Speed control | Interface | Weight class | Best for |
|---|---:|---|---|---|---|
| LD06 | 12 m | PWM, 5-13 Hz | UART 230400 | lightest-to-light | tiny balancing or narrow bots |
| LD19 | 12 m | PWM, 5-13 Hz | UART 230400 | slightly larger | same form factor, more margin |
| STL-06P | 12 m | PWM, ~10 Hz | UART 230400 | tiny | compact with known ecosystem hooks |
| RPLIDAR C1 | 12 m | PWM/motor ctrl | UART | larger | established ecosystem |
| T-mini Plus | 12 m | 6-12 Hz | UART | tiny | improved angular/accuracy |

---

## Sources

- LD06 datasheet mirror: https://www.yahboom.net/xiazai/LiDar-LD06/LDROBOT_LD06_Datasheet.pdf
- Mini Pupper relation and practical usage notes: https://www.minipupperdocs.readthedocs.io/en/latest/guide/Features.html
- Commercial availability listing context: https://www.robotshop.com/products/mangdang-mini-pupper-stl-06p-lidar-module

