---
title: Linear Quadratic Regulator
aliases:
  - LQR
  - Linear Quadratic Regulator
  - Linear-Quadratic Regulator
tags:
  - control-theory
  - optimal-control
  - robotics
  - stabilization
  - state-feedback
  - control
---

# 🧭 Linear Quadratic Regulator (LQR)

## 📘 Core Idea

**LQR** is a model-based optimal feedback controller for systems with linear dynamics and quadratic cost.  
It computes a gain matrix `K` so the policy is:

```text
u = -Kx
```

That one line is why LQR is still useful for RL practitioners: it gives a stable, cheap baseline and a principled local controller for nonlinear robots after linearization.

---

## ⚙️ Mathematical Form (continuous time)

For continuous dynamics:

```text
ẋ = Ax + Bu
```

with infinite-horizon cost:

```text
J = ∫ (xᵀQx + uᵀRu) dt
```

LQR chooses `u` to minimize `J` over time.

The optimal control law is `u = -Kx` where `K = R⁻¹ Bᵀ S` (sign convention may vary).

`S` is the positive-semidefinite solution to the Algebraic Riccati Equation (ARE):

```text
0 = SA + AᵀS - SB R⁻¹BᵀS + Q
```

This is what `lqr` solves in tooling like MATLAB/Control Toolbox and Drake.

---

## 🧪 Finite-Horizon / Time-Varying Extension

For finite-horizon control, `S(t)` solves a differential Riccati equation backward in time and converges to the ARE solution as horizon → ∞.

For time-varying dynamics, `A(t), B(t)` and `Q, R` can change with time and LQR still applies through TVLQR (time-varying LQR), linearization at each sample, and numerical Riccati solves.

That is why LQR can be used as a robust trajectory stabilizer around a reference path: linearize around the nominal trajectory, compute local gains, apply at runtime.

---

## 🧱 LQR vs LQG and Other Baselines

| Method | Assumptions | Handles Estimation Uncertainty | Constraint Handling | Relative Compute | Notes |
|---|---|---|---|---:|---|
| **LQR** | Linear model + quadratic cost | No explicit estimator | No inequality constraints in base form | Low | Great baseline/stabilizer |
| **LQG** | LQR + Kalman Filter | Explicit via KF/Kalman estimator | No hard constraints | Medium | Standard separation principle |
| **PID** | Local tracking intuition | No model of uncertainty | No hard constraints | Very low | Easy, often weaker for coupled dynamics |
| **MPC** | Generic model + constraints | External | Explicit constraints | High/Very high | Better for constraints, heavier compute |
| **iLQR / DDP** | Nonlinear extension with local linear-quadratic models | Optional estimator | Approximate constraints | Medium-High | Used when nonlinearity is strong |
| **LQR/PD blend** | Hierarchical with low-level PD | Depends | Limited | Low | Common in balancing and embedded stacks |

LQR is best when you need a **fast, explainable, stable feedback shell** and can tolerate operating-point locality.

---

## 🧭 How It Behaves on Real Hardware

- Good when dynamics are close to linear around equilibrium.
- Bad if actuation saturates frequently; then constrained MPC or gain scheduling is usually needed.
- Sensitive to `Q/R` tuning:
  - larger `Q` drives faster correction / more energy,
  - larger `R` reduces control aggression / power draw.
- Works best as one layer in a stack:
  - EKF/estimator for state,
  - LQR for stabilization,
  - RL for policy adaptation / task planning.

In unstable platforms (like wheeled bipeds), LQR is often retained as a "guardian" that prevents full-state divergence.

---

## 🧠 LQR in Ascento and UMV Workstreams

- **Ascento:** Ascento control papers explicitly report LQR used for balancing in both prototype and whole-body-control variants. The WBC paper explicitly calls LQR a motion task and reports high-rate controller timing in the 1–2 ms class in embedded-like setups.[^ascento-wbc]
- **UMV:** public UMV materials emphasize constrained RL behaviors and a simulation-to-real workflow; they do not foreground LQR as the primary controller in the same explicit way as Ascento notes.[^umv-paper]

For your pipeline, that suggests:
- Use LQR as a **hard safety controller** in Ascento-style stacks.
- Use RL actor layers above LQR for behavior selection (jump/wheelie/hop modes).

---

## 🧪 PufferLib Integration Pattern

For PufferLib, treat LQR as a fixed function module:

- Keep env-step deterministic with explicit state transition and reward.
- Add optional **policy override** path:
  - `policy_output` can gate or blend with `u_lqr` when estimator confidence drops.
- Track `lqr_error = |x - x_des|`, `torque_norm`, and `command_saturation_ratio`.

This lets you keep rollout stability while training high-level controllers in the same framework.

---

## 📐 Suggested Tuning Checklist

1. Start with state scaling and meaningful units (`rad`, `m`, `m/s`, etc.).
2. Set `Q` from state target priorities (e.g., tilt > position > speed depending on task).
3. Set `R` from actuator limits and energy budget.
4. Validate closed-loop poles `eig(A-BK)` are stable and sufficiently damped.
5. Stress test for actuator saturation and startup transients.

---

## 🔗 Related Topics

- [[Control Theory]]
- [[Kalman Filter]]
- [[Extended Kalman Filter]]
- [[Whole-Body Control]]
- [[Trajectory Optimization and MPC]]
- [[Ascento]]
- [[RAI Ultra Mobility Vehicle]]
- [[PufferLib]]
- [[PufferLib Robotics Fit and Limits]]

---

## 📚 Further Reading

- Underactuated Robotics: LQR chapter. https://underactuated.csail.mit.edu/lqr.html
- MathWorks `lqr` API docs. https://www.mathworks.com/help/control/ref/lti.lqr.html
- CMU 16-299 LQR/DDP notes. https://www.cs.cmu.edu/~cga/controls-intro/16-299-2026-LQR-DDP.pdf
- MathWorks LQG overview for estimator-controller composition. https://www.mathworks.com/help/control/ref/ss.lqg.html

---

## Source Notes

[^ascento-wbc]: Ascento follow-up whole-body-control paper describes LQR-assisted balancing in a kinematically constrained model and reports low-control-loop computational cost in the embedded-like architecture. https://arxiv.org/abs/2005.11431
[^umv-paper]: UMV paper/summary is centered on constrained RL and does not frame LQR as the core documented controller, but provides rich RL transfer context for comparison. https://arxiv.org/abs/2602.22118
[^mathworks-lqr]: MathWorks LQR API computes gain `K`, ARE solution `S`, and closed-loop poles via `lqr`. https://www.mathworks.com/help/control/ref/lti.lqr.html
[^underactuated-lqr]: Tedrake, R. Underactuated Robotics LQR chapter includes the infinite-horizon derivation and finite-horizon Riccati evolution. https://underactuated.csail.mit.edu/lqr.html
[^cmu-lqr]: CMU LQR/DDP notes summarize linearization-based derivation and relationship to trajectory methods. https://www.cs.cmu.edu/~cga/controls-intro/16-299-2026-LQR-DDP.pdf
