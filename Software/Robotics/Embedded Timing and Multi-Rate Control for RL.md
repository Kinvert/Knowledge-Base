---
title: Embedded Timing and Multi-Rate Control for RL
aliases:
  - Multi-Rate Control for RL
  - Embedded Timing for Robot RL
  - RL Control Timing
tags:
  - robotics
  - control
  - reinforcement-learning
  - timing
  - embedded-systems
---

# Embedded Timing and Multi-Rate Control for RL

Balancing RL systems fail when timing is implicit.
For this class of robot, treat timing as part of the **environment contract**.

## Why multi-rate matters

Real platforms commonly use layered loops:
- 100 Hz to 1 kHz state estimation/control,
- 50 Hz to 200 Hz policy or trajectory commands,
- 5 Hz to 20 Hz planner/mission updates.

If policy and stabilization run at the same frequency, you often lose stability margin.
If policy is slower but bounded with deterministic gating, you can still get good behavior.

Ascento/UMV notes already indicate:
- high-rate local control,
- and policy cadence lower than motor-level loops.

---

## Canonical three-layer schedule

| Layer | Typical Rate | Function | Failure handling |
|---|---:|---|---|
| Layer 0 | 500 Hz – 8 kHz | Current/torque loop, current limiting, motor current protection | Hardware cutoff first |
| Layer 1 | 100 Hz – 500 Hz | State estimation (`EKF`/state filter), mode transitions, safety checks | Drop to safe mode, estimator confidence gating |
| Layer 2 | 20 Hz – 100 Hz | RL policy step, objective shaping, mode selection | Action hold/failsafe blend |

For balancing robots, Layer 1 and Layer 2 rates are usually where tuning errors happen.

---

## Pattern for deterministic PufferLib loops

Your Puffer loop should model only outer layers as action producers:

1. `step()` at fixed policy rate (e.g., 50 Hz),
2. env applies `hold + smoothing` to map policy action to lower-rate control command,
3. inner simulator/controller simulates actuation lag and actuator limits,
4. obs returned includes `stale` and saturation metrics.

This lets you train fast while preserving hardware-like timing.

### Typical action post-processing

- `a_policy` -> low-pass filter (`α`),
- slew limit per axis,
- saturation to physical torque,
- command interpolation over inner ticks.

---

## Jitter budget and deadlines

Define a hard budget per stage:
- estimation deadline (`t_est`),
- action apply deadline (`t_cmd`),
- inference deadline (`t_inf`).

If any deadline is exceeded:
- increase policy confidence penalty,
- trigger mode fallback,
- log `timing_violation_ms` in `info`.

Do not hide violations with longer averaging; track them explicitly.

---

## Time alignment checklist

- Single clock source for:
  - IMU packets,
  - lidar scans,
  - encoder sample,
  - policy step.
- Sequence every `step()` result with:
  - `sim_time`,
  - `sensor_time`,
  - `obs_age_ms`.
- Keep reset deterministic:
 - clear timestamps,
 - clear action history,
 - clear estimator state history.

---

## Timing table for your target stack

| Platform | policy loop target | inner-loop realism | RL fit |
|---|---:|---|---|
| Single-board host + simulated inner loop | 20–100 Hz | Configurable by model | Excellent for dev speed |
| Mid SBC with C99 env | 50–250 Hz policy stepping | High if deterministic timing in C | Very good SPS |
| UMV-like stack (policy + custom inner) | 50 Hz policy / 200 Hz local / 8 kHz motor loops | High if implemented | Good transfer profile |
| Ascento-like ROS2-based adaptation | 50–200 Hz command + low-level loops | Medium-high | Better with adapter and smoothing |

This table is not a hardware benchmark, it is a **deployment expectation model**.

---

## Common hazards and mitigations

- **Policy overdrives during spikes**  
  Add action clipping + slew limit + torque budget.
- **Sensor stale windows**  
  Add explicit stale counters and zero-order holds.
- **Clock drift**  
  Add periodic re-sync and monotonic time stamps.
- **Dropouts from IPC/ROS scheduling**  
  Keep RL path lean, and isolate low-level loops from high-latency graph nodes.

---

## Practical implementation pattern (robot + Puffer)

1. Build `PufferLib` policy loop at fixed rate (e.g., 50 Hz) in a separate process/thread.
2. Run local controller process at fixed high rate with deterministic ring buffer.
3. Publish only fixed-size sampled state to RL.
4. On policy timeout:
   - hold last valid action,
   - or blend to damping mode depending safety layer.
5. Log:
   - action_age,
   - inference_ms,
   - jitter_ms,
   - saturation.

---

## Sources and connected notes

- [[Ascento]]
- [[RAI Ultra Mobility Vehicle]]
- [[Extended Kalman Filter]]
- [[Line-Scan Lidar as RL Observation]]
- [[Drive-Train Modeling for Balancing Robots]]
- [[Robot Policy Deployment]]
- [[PufferLib]]
