---
title: LDROBOT LD19
aliases:
  - LD19
  - LDROBOT LD19
  - DTOF LD19
tags:
  - lidar
  - robotics
  - sensors
  - rl
---

# LDROBOT LD19

Higher-end DTOF sibling in the LD06 class with stronger signal and similar ring integration behavior, often sold as LD19/D300 family variants.

---

## Core profile

- Scan geometry: 360 degrees
- Ranging range: 0.02 to 12 m (70% target reflectivity)
- Scan speed: 5 to 13 Hz, with PWM-exposed control path
- Ranging sampling frequency: 4500 Hz
- Angular resolution: around 0.8 degrees
- Accuracy band: approx. ±45 mm class in many practical documents
- Supply: 4.5 to 5.5 V
- Interface: 4-pin
  - UART@230400
  - PWM motor control
  - GND and P5V
- Form factor: around 54 x 46.29 x 34.8 mm
- Estimated weight: around 47 g
- Ambient resistance: around 30 K lux listed in LD19 docs

---

## Practical notes

- LD19 and LD06 share a lot of parser assumptions, but LD19 can show a cleaner envelope at longer ranges.
- Keep an explicit startup handshake if possible; some modules pause when external PWM control is floating.
- Use deterministic packet checksums and frame counters to avoid feeding ghost sectors into PPO/CV style policies.

---

## Pros

- Better range stability near 12 m compared with smaller hobby variants.
- Similar firmware simplicity to LD06-style 4-pin modules.
- Good stepping point before moving to higher-end multi-hz and larger FoV hardware.

## Cons

- Fewer mini-form-factor integrations than STL-06P style modules.
- More than three wire variants in the wild (board revisions), so UART parser checks matter.
- Still 2D only; no vertical profile unless physically tilted.

---

## PufferLib / RL wiring pattern

### Useful preprocessing baseline

1. Keep scan acquisition in `scan_callback`, not in the policy thread.
2. Normalize to fixed ring length (`N` rays) before stacking temporal features.
3. Add validity metadata:
   - `invalid_ratio`
   - `spin_hz_est`
4. For terrain tasks, compute sector statistics every frame:
   - minimum/median/front_window_min
   - slope changes across bins

This tends to reduce crashy policies driven by occasional invalid blips.

---

## Comparison chart

| Unit | Range | Scan speed | Baud | Form factor | Typical fit |
|---|---:|---:|---:|---|---|
| LD19 | 12 m | up to 13 Hz | 230400 | larger than LD06 | outdoor-aware ground robot |
| LD06 | 12 m | up to 13 Hz | 230400 | smaller | tight payload builds |
| STL-06P | 12 m | around 10 Hz | 230400 | tiny | mini quadruped/class |
| T-mini Plus | 12 m | 6-12 Hz | UART | tiny | modern compact option |
| YDLIDAR X4 | >10 m | 6-12 Hz | 128000 | medium | higher scan density |

---

## Sources

- LDROBOT LD19 datasheet: https://www.ldrobot.com/images/2023/05/23/LDROBOT_LD19_Datasheet_EN_v2.6_Q1JXIRVq.pdf
- LD19 product context and features: https://www.waveshare.com/product/dtof-lidar-ld19.htm

