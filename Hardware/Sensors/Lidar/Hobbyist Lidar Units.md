---
title: Hobbyist Lidar Units
aliases:
  - Hobby Lidar
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

For balancing bots, small service robots, and hobby platforms, this is the practical shortlist of spinning 2D lidar modules that are typically usable around hobby budgets and simple MCU/Jetson-class integration.

---

## 🧠 Why hobby spinning lidars over depth cameras?

- You need continuous obstacle geometry around the robot (especially cliffs/edges), not just forward depth.
- You can tolerate the `360°` sweep cadence (typically `5–13 Hz`) and fixed-rate downstream policy steps.
- You want a deterministic, compact observation shape (`64/128/256 rays`) for RL.

Compared with ToF depth modules, spinning lidars usually give better floor-contact safety signals at close range and easier deterministic scan preprocessing.

They are weaker for:

- aggressive high-speed driving where scan latency becomes dominant.
- height-disambiguation and 3D structure (unless you stack sensors).
- bright outdoor sunlight, glass, and highly reflective clutter without extra filtering.

---

## 🛒 5+ practical picks in the hobby price band

| Option | Price class | Scan geometry | Approx. range | Scan speed | Interface | Why hobby builders pick it |
|---|---|---:|---|---|---|
| Option | Price class | Scan geometry | Approx. range | Scan speed | Interface | Why hobby builders pick it |
|---|---|---|---:|---|---|---|
| [[YDLIDAR X2]] | budget | 360° | ~8 m | ~7 Hz | UART | cheapest entry to 2D spinning lidar |
| [[YDLIDAR X4]] | mid | 360° | >10 m | 6–12 Hz | UART @128000 | denser scan behavior than X2 |
| [[YDLIDAR T-mini Plus]] | mid | 360° | 12 m | 6–12 Hz | UART | compact ToF profile with modern behavior |
| [[LDROBOT LD06]] | budget-mid | 360° | 12 m | 5–13 Hz | UART @230400 | tiny form factor and low mass |
| [[LDROBOT LD19]] | budget-mid | 360° | 12 m | 5–13 Hz | UART @230400 | similar stack with different mechanical size |
| [[SLAMTEC RPLIDAR C1 360 DTOF Laser Scanner]] | higher | 360° | ~12 m | ~5–10 Hz | UART | known ecosystem and documented tooling |
| [[RPLIDAR C1]] | mid | 360° | ~12 m | ~5–10 Hz | UART | stable family behavior and compatibility examples |
| [[LDROBOT STL-06P]] | budget-mid | 360° | 12 m | ~10 Hz | UART @230400 | compact package for micro-bots |
| [[Mini Pupper STL-06P Lidar Module]] | mid | 360° | 12 m | 6–10 Hz | UART | very small integration footprint |
| [[RPLIDAR A1M8]] | budget-mid | 360° | ~12 m | 10 Hz | UART / USB bridge | mature, popular ecosystem for older hardware |

---

## 💰 What to buy for a rough “~$100” target

Noisy market prices vary by reseller, shipping, and revision. Good practical logic:

- safest bet: buy from a seller with a clear datasheet and known firmware revision
- expect clone variance, especially on older batches
- if a listing omits exact model revision, budget additional time for parser validation
- prefer UART baud ≥230400 only if your MCU can keep stable timing at that speed

A stable starter set near this band is usually:

- [[YDLIDAR X2]] when lowest cost and fastest integration matters most
- [[LDROBOT LD06]] for smallest form factor with better range than many legacy units
- [[YDLIDAR T-mini Plus]] if you want compact ToF behavior
- [[RPLIDAR A1M8]] when you want legacy ecosystem examples first

---

## 🧪 Build matrix for RL and sim2real

| Profile | Best candidates | Why |
|---|---|---|
| First RL experiment / lowest friction | [[YDLIDAR X2]], [[RPLIDAR A1M8]] | minimal docs and tooling friction |
| Tiny balancing bots | [[Mini Pupper STL-06P Lidar Module]], [[LDROBOT LD06]] | smallest mounts |
| Mixed indoor/outdoor obstacle work | [[LDROBOT LD06]], [[YDLIDAR T-mini Plus]] | better short-range behavior |
| Denser local geometry features | [[YDLIDAR X4]], [[YDLIDAR T-mini Plus]] | more angular detail for obstacle edges |
| Long-term maintainability | [[RPLIDAR C1]], [[SLAMTEC RPLIDAR C1 360 DTOF Laser Scanner]] | larger public examples and ROS references |
| “I want line-scanner behavior for balancing front profile” | [[Line Lidar for Balancing Robots]] | quasi-front profile with custom processing |

---

## ⚙️ PufferLib integration pattern

### Mandatory

- force a fixed ring length per step (`N=64/128/256`)
- track `invalid_ratio` and `scan_stale_age` in your observation metadata
- keep one preprocessing function per sensor SKU behind a versioned config key
- gate policy transitions until one full scan packet is ready
- prefer deterministic resampling so packet jitter does not change observation shape

### Recommended derived features

- `sector_min`, `sector_p10`, `sector_median`, `sector_p90`
- `front_delta` between front window and its previous frame
- obstacle-risk term like `front_window_min + invalid_ratio`
- scan dropout handling via fallback of last valid frame for one step

For noisy links, feeding a short previous-frame fallback before forced reinitialization is often better than hard reset in PPO-style loops.

---

## ✅ Pros and ❌ Cons

### Pros
- predictable observation shape if preprocessing is standardized
- fast integration on STM32/ESP32-class hosts via UART
- strong forward obstacle signal for reward shaping and safety wrappers
- simple enough for sim2real pipelines in [[PufferLib]]

### Cons
- 2D-only geometry, no elevation awareness
- clone/factory variance across sellers, especially older revisions
- reflective glass and sunlight can produce invalid point clusters
- UART timing variance can still hurt if parser code is not rate-limited

---

## 🧭 Comparison chart by design objective

| Objective | Best fit | Why |
|---|---|---|
| lowest startup cost | [[YDLIDAR X2]] | cheapest entry, easiest to source |
| lowest mass | [[LDROBOT STL-06P]], [[Mini Pupper STL-06P Lidar Module]] | physical footprint wins |
| strongest ecosystem confidence | [[SLAMTEC RPLIDAR C1 360 DTOF Laser Scanner]], [[RPLIDAR C1]] | established reference stacks |
| modern small-to-medium robot | [[YDLIDAR T-mini Plus]], [[LDROBOT LD06]] | compact + modern ranging behavior |
| denser obstacle profile | [[YDLIDAR X4]], [[YDLIDAR T-mini Plus]] | improved angular precision |
| maximum compatibility with earlier code | [[RPLIDAR A1M8]] | many older ROS examples |

---

## Related notes in this vault

- [[Line-Scan Lidar as RL Observation]]
- [[Line Lidar for Balancing Robots]]
- [[RPLIDAR A1M8]]
- [[RPLIDAR A2M6]]
- [[RPLIDAR C1]]
- [[RPLIDAR A1M8]]
- [[LDROBOT LD06]]
- [[LDROBOT LD19]]
- [[LDROBOT STL-06P]]
- [[YDLIDAR X2]]
- [[YDLIDAR X4]]
- [[YDLIDAR T-mini Plus]]
- [[Mini Pupper STL-06P Lidar Module]]
- [[SLAMTEC RPLIDAR C1 360 DTOF Laser Scanner]]
- [[RPLIDAR A2M6]]

---

## External resources

- [[Lidar]] for core lidar terminology
- manufacturer and SDK links inside each module note
- [[PufferLib]]
