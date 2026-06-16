---
title: Line-Scan Lidar as RL Observation
aliases:
  - Line Lidar as RL Observation
  - 2D Lidar Profile Observations for RL
tags:
  - robotics
  - reinforcement-learning
  - lidar
  - perception
  - observation-space
---

# Line-Scan Lidar as RL Observation

Line-scan lidar for balancing robots means taking a ring/scanner and using only a forward band (or a fixed angular sector), then feeding that profile to the policy.
This is a practical compromise for:
- low-latency obstacle sensing,
- reduced bandwidth,
- better sim-to-real transfer than raw images.

For your 2DOF pogo bot use-case, it is especially good because it preserves **terrain edge signals** (`floor rise`, `step`, `drop`, `curb`) without requiring stereo rigs.

---

## Recommended observation design

### Option A: Raw sector vector
If your scan rate is stable, use fixed-length floats:

```text
obs_scan = [d0, d1, ... dN-1, invalid_count, invalid_ratio]
```

- N from 32 to 256 depending compute.
- Convert with `min(max(d, 0), max_range)` and keep validity channels.
- Keep units consistent (`m`) and orientation fixed (`forward wedge`).

### Option B: Derived features (more sample efficient)
Keep a small fixed feature vector:
- `min`, `p10`, `p50`, `p90`,
- forward slope proxy,
- roughness metric (`std` over sector),
- rise-edge count,
- invalid ratio + confidence.

Derived features usually stabilize early learning with little loss of terrain relevance.

---

## Mechanical mounting for balancing robots

Two common mount strategies:

- **Horizontal ring with forward wedge extraction**
  - simplest hardware path,
  - good for wall/maze/obstacle tasks.

- **Vertical tilt for stair/step probing**
- scan plane intersects ground profile directly,
- best for 1D terrain features like ramps/curbs/cliffs.

For your forward-only wheel-locked motion, keep a fixed orientation in software and calibrate in software after each startup.

---

## Preprocessing contract for sim and hardware

For stable RL, treat lidar preprocessing as part of the env and keep it deterministic.

### Deterministic pipeline
1. Raw packet acquisition (timestamped),
2. sector extraction (`±45°` or fixed indices),
3. invalid filtering (`NaN`, out-of-range),
4. fixed-point vectorization and clipping,
5. optional feature extraction,
6. concat to existing `obs`.

If no fresh scan arrives before policy step, hold last observation and increment an `stale_scan_age` channel.

---

## Reward coupling pattern

Useful reward terms for terrain-aware balance policies:
- forward progress,
- upright penalty (`|pitch|`, `|pitch_rate|`),
- slope/terrain penalty from scan variance,
- penalty for repeated invalid sectors,
- crash/loss penalties if any sector crosses near-range threshold too often.

Done conditions:
- fall detection (IMU),
- persistent invalid ratio above threshold,
- large terrain shock (jerk spike + low contact confidence).

---

## Why line-scan beats raw depth images for this task

- Lower bandwidth and deterministic shape.
- Much cheaper preprocessing than stereo/monocular geometry.
- Better for policies on embedded MCUs or SBCs with limited throughput.
- Keeps policy from overfitting to textural visual artifacts.

Cons:
- no object identity,
- no lateral obstacle depth context outside selected sector unless you add second scan,
- can mis-handle glass/specular returns without confidence gating.

---

## PufferLib integration checklist

- Keep scan size fixed in schema (do not vary length by scan mode).
- Add scan channels in a deterministic order before reward/state heads.
- Use `float32` for scan bins, `uint8` for validity if separate.
- Track a dedicated `scan_stale_steps` metric in `info`.
- If running policy at slower rate than sensor, downsample at fixed schedule.

This mirrors the broader approach used in the balancing notes: RL gets a compact observation contract, while stabilization remains deterministic at lower layers.

---

## Related implementation pattern (Ascento / UMV)

For Ascento/UMV style projects:
- keep lidar as a high-level terrain feature layer,
- keep the stabilizer in inner loop,
- route RL commands through mode-safe interface (`vx`,`yaw`,`jump_mode`) instead of direct wheel torque unless policy stack is mature.

This matches the same principle used in:
- [[Ascento]] (reduced actuation abstraction),
- [[RAI Ultra Mobility Vehicle]] (policy-level modes over raw torques),
- [[Embedded Timing and Multi-Rate Control for RL]] (multi-rate sensor-policy path).

---

## Sources

- `RPLIDAR C1`, `RPLIDAR A1M8`, and `LDROBOT STL-06P` product pages and datasheet mirrors in current notes.
- Internal balancing sensor integration notes in this KB:
  - [[Line Lidar for Balancing Robots]]
  - [[RPLIDAR C1]]
  - [[RPLIDAR A1M8]]
  - [[LDROBOT STL-06P]]

