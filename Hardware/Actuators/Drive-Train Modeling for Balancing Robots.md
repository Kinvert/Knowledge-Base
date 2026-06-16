---
title: Drive-Train Modeling for Balancing Robots
aliases:
  - Drive-Train Modeling for Balancers
  - Balancing Robot Drivetrain
tags:
  - robotics
  - actuators
  - drivetrain
  - control
  - dynamics
  - simulation
---

# Drive-Train Modeling for Balancing Robots

For balancing robots, the drive train is where policy intent becomes physical action.
Most RL failures here are not from policy design, but from mismatched torque/speed models or hidden actuator dead-bands.

This note gives a pragmatic modeling stack you can reuse directly in a simulator or a PufferLib bridge.

---

## 1) System architecture

Typical two-wheel balancer drivetrain:

```text
policy (vx, yaw, mode) → torque controller → motor → gearbox → wheel → ground contact
```

Each wheel gets its own chain, plus a cross-coupling path when wheel speeds are converted into `vx` and yaw rates.

### Variables
- `ω_L, ω_R` = left/right wheel angular velocity,
- `τ_mL, τ_mR` = motor shaft torques,
- `τ_wL, τ_wR` = wheel torques after gearing,
- `r` = wheel radius,
- `w` = wheel separation,
- `T_g` = gear ratio,
- `η` = drivetrain efficiency.

---

## 2) Core kinematic mapping

For differential drive:

```text
v = (r/2)(ω_R + ω_L)
ψ̇ = (r/w)(ω_R - ω_L)
```

Inverse:

```text
ω_R = v/r + (w/2r) ψ̇
ω_L = v/r - (w/2r) ψ̇
```

For RL contracts, keep both:
- command space (high-level `v`, `ψ̇`)
- wheel-space actuator space (`ω_R`, `ω_L`) where actuator constraints live.

---

## 3) Torque and speed conversion

Useful reduced torque pipeline:

```text
τ_w = η T_g τ_m
T_m = J_m τ_ṁ = K_t i_q - b_m ω_m - τ_load
v_bus = K_e ω_m + i_q R + L di/dt
```

where motor constant and electrical equations become important once you add current limits, brownouts, and fast transients.

Include saturation before physics step:
```text
|τ_command| <= τ_max(i_bus, T_limit, temperature)
```

Without saturation in the model, policies learn commands that are impossible on real hardware.

---

## 4) Ground force relation

Wheel force:

```text
F_x = (τ_w / r) * C_slip
```

with `C_slip ∈ [0, 1]`.

For deterministic sim:
- `C_slip = 1` near nominal,
- then decay toward 0 as slip ratio rises or normal-force drops.

The easiest robust approximation:
```text
C_slip = clamp(1 - k_s |v_rel| - k_v |a_long|, 0, 1)
```

Add mode switch:
- `C_slip = C_nominal` on flat static,
- reduced during impact/jump landings.

---

## 5) Balance-critical coupling terms

In balancing robots, drivetrain errors couple directly into body pitch:
- over-thrust wheel torque gives fast correction but can amplify oscillation,
- under-thrust causes drift and delayed recovery,
- asymmetric torque command from control delays can destabilize yaw.

For robust sim:
- add delay `τ_d` in torque command application (2–10 ms typical),
- add first-order actuation lag:

```text
τ_applied_dot = (τ_cmd - τ_applied) / T_act
```

This is critical for matching UMV- and Ascento-like behavior.

---

## 6) Suggested model levels for RL

### Level 0 (fast prototyping)
- direct wheel force from action,
- bounded by `F_max`,
- no electrical model.

### Level 1 (default balancing robot)
- motor current/torque saturation,
- gear efficiency and stiction,
- slip proxy and thrust lag.

### Level 2 (sim2real-focused)
- two-stage motor driver + power limit model,
- thermal/current derating,
- impact force spikes,
- delay + jitter injection.

Level 2 is recommended before real robot deployment; Level 0 is fine for policy exploration.

---

## 7) PufferLib mapping

Use a fixed env contract:

- `obs`: `ω_L`, `ω_R`, estimated torque usage, battery/load proxy,
- `act`: `τ_L_cmd`, `τ_R_cmd` (or derived `v_cmd`, `yaw_cmd`),
- `reward`: speed tracking + energy + stability,
- `info`: `saturation_count`, `clipped_ratio`, `gear_slip_proxy`.

Keep motor model in env step if possible; avoid runtime network calls.

---

## 8) Common failure modes

- **Exploding power draws**: action head missing torque caps.
- **Unstable oscillation**: omitted lag and low-pass on torque.
- **Sim2real under-performance**: ideal-ground contact with no slip mode.
- **Policy overconfidence**: no explicit `info` channels for actuator health.

---

## Related topics

- [[Motor Controllers]]
- [[ESC]]
- [[FOC]]
- [[PMSM]]
- [[PID Controller]]
- [[State Estimation]]
- [[Two-Wheeled Inverted-Pendulum Dynamics]]
- [[Embedded Timing and Multi-Rate Control for RL]]

---

## Sources

- Hardware and drivetrain inference from balancing robot sources already in this KB:
  - [[Ascento]]
  - [[RAI Ultra Mobility Vehicle]]
- ROS2/robot control standards for command/state boundaries and estimator-controller separation:
  - [[ros2_control]]
- RL-safe control structure and inner-loop/outer-loop coupling:
  - [[Robot Policy Deployment]]

