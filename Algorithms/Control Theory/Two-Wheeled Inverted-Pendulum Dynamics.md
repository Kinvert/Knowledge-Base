---
title: Two-Wheeled Inverted-Pendulum Dynamics
aliases:
  - Two-Wheeled Inverted Pendulum Dynamics
  - Two-Wheeled Self-Balancing Dynamics
tags:
  - control-theory
  - robotics
  - dynamics
  - reinforcement-learning
  - balancing
  - state-space
---

# Two-Wheeled Inverted-Pendulum Dynamics

Two-wheeled balancing robots (Segway-style hubs, self-balancing scooters, Ascento-like platforms) are usually modeled as a **narrow inverted-pendulum system with two drive wheels**.
The critical point for RL is matching model order to what you need for learning:
- a fast, interpretable reduced model for stable sim-to-real transfer, or
- a richer model when you care about aggressive jumps, wheel slip, and terrain effects.

---

## State choice and reduced equations

Most practical controllers for balance use a reduced 4- to 6-state model:

```text
x = [p, ṗ, θ, θ̇, ψ, ψ̇]ᵀ
```

- `p`: wheel-frame position (or body CoM travel proxy),
- `ṗ`: forward velocity,
- `θ`: tilt angle (pitch), positive forward lean,
- `θ̇`: tilt rate,
- `ψ`: average wheel angle (or body yaw),
- `ψ̇`: yaw rate.

The reduced continuous-time linear form is:

```text
ẋ = A x + B u
y = C x + D u
```

where `u` is typically `[u_l, u_r]` or `[u_v, u_ψ]` (linear-speed and yaw-rate commands), depending on your abstraction.

In one common reduced form:

```text
[ṗ]   = [0 1 0 0 0 0] x + [0 0] u
[θ̈]   = f_θ(x, u, params)
[ψ̇]   = f_ψ(Δu, geometry, params)
```

The exact nonlinear terms come from:
- gravity-restoring torque on tilt,
- wheel-ground contact geometry,
- wheel inertial coupling,
- motor torque limits and actuator dead-zones.

For RL harnesses, the most robust approach is:
1. keep nonlinear propagation in your "physics" env,
2. expose linearized gains via LQR/PID around upright (`θ ≈ 0`) for safety fallback.

---

## Core nonlinear dynamics structure (high-signal summary)

For a rigid-body body mass `m_b` and wheel mass `m_w`, wheel radius `r`, wheelbase separation `w`, COM height `h`:

- Pitch equation (schematic):

```text
(J_θ + m_b h²) θ̈ = m_b g h sin(θ) + τ_drive_term - τ_friction_term - τ_feedback
```

- Translation equation (schematic):

```text
m_tot p̈ = F_wheel - F_slope - F_drag - F_friction
```

- Yaw equation (schematic):

```text
J_ψ ψ̈ = (τ_r - τ_l) * r / w - τ_yaw_damping
```

These are placeholders for implementation-level derivations (wheel inertia and exact sign conventions vary by parameter convention).
The important thing is identifying what you model explicitly:
- do you include wheel slip as explicit state?
- do you include motor dynamics or treat motor torque as perfect input?
- do you include actuator lag and saturation as stateful dynamics?

---

## From physics to RL state contract

For `Ascento`/`UMV` style stack design, use:

- Raw physical state (for sim): position, tilt, tilt rate, wheel velocities, wheel torque, battery/current proxy.
- RL state (recommended minimal): normalized `[θ, θ̇, ṗ, ψ̇, contact_flag, tilt_jerk]` plus estimator confidence.

This contract avoids overloading policy with unobservable internals while still retaining balancing-critical observables.

Use a second-stage state if you add jumps:
- jump mechanism height,
- jump phase,
- contact mode (ground/airborne),
- posture-lock mode.

---

## Contact and slip terms that matter

For balancing systems, many policies fail in sim because ground model is too ideal:

### Coulomb + viscous grip model

```text
F_x = μ F_z sign(v_rel) + b v_rel
```

When this is omitted, policies that overfit to smooth simulation often become too aggressive on real terrain.

### Regimes you should represent
- **Sticky ground**: normal force dominates and longitudinal acceleration maps tightly to torque.
- **Micro-slip**: small slip transitions; policy needs smooth damping.
- **Slip-loss**: slip ratio saturates; policy must recover quickly.

For RL, include either:
- mode flag + slip proxy in observation, or
- conservative action clipping at high-slip confidence.

---

## Why high-rate inner loops still matter

Balancing is a fast instability; if your policy loop is too slow, you need extra estimator/controller layers:

- `1 kHz` inner stabilization + `50–100 Hz` policy (like UMV papers),
- `400 Hz` control authority in Ascento WBC stack timing reports,
- inner-loop safety filter should saturate outputs before policy sees unsafe margins.

If you expose this explicitly in env design, `PufferLib` step can stay deterministic at moderate rates while hardware safety remains fast.

---

## Controller compatibility and failure behavior

Recommended structure:

- `LQR`/PID baseline for upright stabilization,
- optional estimator (`EKF` or equivalent) for state fusion,
- RL actor for mode-selection and command shaping.

When in doubt for a robot with strong coupling:
- keep upright policy objective as reward term with hard penalty,
- add action-rate limits and soft torque penalties,
- kill episodes on tilt/jerk divergence plus recovery state flag.

---

## What to include in custom env contracts

At minimum include:
- `obs`: `[θ, θ̇, ṗ, ψ̇, pitch_accel_confidence, cmd_inflight_ms]`
- `act`: `[torque_left, torque_right]` or `[vx_cmd, yaw_cmd, posture_gain]`
- `done`: fall, hard tilt, contact loss, timeout
- `info`: `in_saturation`, `estimator_cov_trace`, `mode`

Then align with a fixed `dt` and deterministic reset.

---

## Practical links

- [[Ascento]]
- [[RAI Ultra Mobility Vehicle]]
- [[PufferLib]]
- [[PufferLib C99 Environment Authoring]]
- [[Extended Kalman Filter]]
- [[Linear Quadratic Regulator]]
- [[Line-Scan Lidar as RL Observation]]
- [[Embedded Timing and Multi-Rate Control for RL]]

---

## Sources and implementation references

- Ascento WBC and balancing architecture paper references, including high-rate control loop context and actuator abstraction.
- RAI UMV platform docs and papers referencing policy-to-low-level timing separation.
- Robotics balancing literature (inverted pendulum family, LQR stabilization, and state-space control) as the baseline modeling anchor.
