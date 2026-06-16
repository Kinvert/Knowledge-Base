---
title: Extended Kalman Filter
aliases:
  - Extended Kalman Filter
  - EKF
  - Nonlinear Kalman Filter
tags:
  - control-theory
  - estimation
  - sensor-fusion
  - robotics
  - localization
  - navigation
  - kalman-filter
---

# 📉 Extended Kalman Filter

## 🧭 Overview

The **Extended Kalman Filter (EKF)** is the nonlinear extension of the Kalman Filter.  
It uses the same recursive **predict–correct** structure as KF but linearizes nonlinear dynamics and measurements at every time step.

In practice, EKF is the de-facto starting point for robots that need:

- high-rate inertial integration,
- correction from vision/encoders/gps/marker sensors,
- low-latency state estimates for balance and control.

For robotics, this is usually the first practical estimator layer below RL policies and above raw sensors.

---

## 📚 Why EKF Still Matters

- Pure kinematics or raw IMU integration drifts quickly.
- Full nonlinear filters (particle filters, factor graphs, batch optimizers) are often too heavy for hard realtime loops.
- EKF gives a lightweight estimator with uncertainty tracking that is easy to tune and deterministic when implemented correctly.

This is exactly the pattern used in many physical systems where RL is used for high-level control: estimator + controller loop stays model-driven, policy stays task-driven.

---

## ⚙️ Mathematical Core

For a nonlinear discrete-time plant:

```text
x_k = f(x_{k-1}, u_{k-1}, w_{k-1})
z_k = h(x_k, v_k)
```

with process noise `w_k`, measurement noise `v_k`, and measurements `z_k`.

1. **Predict**
   - Predict state: `x̂_{k|k-1} = f(x̂_{k-1|k-1}, u_{k-1})`
   - Linearize state transition:
     `F_k = ∂f/∂x | x̂_{k-1|k-1}`
   - Predict covariance:
     `P_{k|k-1} = F_k P_{k-1|k-1} F_kᵀ + Q_k`

2. **Correct**
   - Linearize measurement:
     `H_k = ∂h/∂x | x̂_{k|k-1}`
   - Innovation:
     `y_k = z_k - h(x̂_{k|k-1})`
   - Innovation covariance:
     `S_k = H_k P_{k|k-1} H_kᵀ + R_k`
   - Kalman gain:
     `K_k = P_{k|k-1} H_kᵀ S_k^{-1}`
   - Update state:
     `x̂_{k|k} = x̂_{k|k-1} + K_k y_k`
   - Update covariance:
     `P_{k|k} = (I - K_k H_k) P_{k|k-1}`

The key difference from KF is that `F_k` and `H_k` are recomputed every step via Jacobians.

---

## 🧭 EKF vs Kalman Family

| Filter | Model Class | Core Approximation | Complexity | Best Use Case | Typical Failure Risk |
|---|---|---|---:|---|---|
| **Kalman Filter** | Linear | None (exact for linear Gaussian) | Low | Wheeled odom + low-order sensor fusion | Breaks on nonlinear geometry |
| **Extended KF** | Nonlinear | 1st-order Jacobian linearization | Medium | IMU+encoder attitude/state pipelines | Divergence if model linearization is poor |
| **Iterated EKF** | Nonlinear | Re-linearize around corrected state | Medium-High | Strongly nonlinear measurement regime | Higher compute/cadence pressure |
| **Unscented KF** | Nonlinear | Sigma-point transform (derivative free) | High | Severe nonlinearities | More compute, careful tuning |
| **Particle Filter** | Nonlinear/non-Gaussian | Sampling | High | Multi-modal distributions | Slow at high state dimension |
| **Factor-graph optimizers** | Nonlinear factor graphs | Batch/sliding-window optimization | High | Loop closure-heavy VIO/SLAM | Not always real-time deterministic |

---

## 🔬 EKF Design Realities (important for real robots)

- **Model consistency matters more than code speed.** Bad noise covariance (`Q`, `R`) quickly destabilizes estimates.
- **Observability assumptions matter.** If a state is weakly observed, filter innovation can collapse it incorrectly.
- **Linearization validity is local.** At large tilt/motion, Jacobian errors dominate; instability can appear as covariance collapse or aggressive gain oscillation.
- **Numeric robustness matters.** Numerically stable implementations usually do Joseph-form covariance update or square-root variants for long runs.
- **Initialization matters.** Poor initial covariance makes the first few seconds bad even with good models.

EKF is not “magic.” It is a good default estimator when:
- nonlinear system dynamics are moderate,
- you need deterministic real-time behavior,
- and your state space is moderate (pose + rates + biases + biases and not entire scene graphs).

---

## 🧠 Practical EKF for RL Workflows

For **PufferLib + real robots**, the most practical placement is:

- **Inner loop:** EKF runs on the robot host or in a low-latency control thread,
- **Outer loop:** RL policy gets filtered state as observation,
- **Safety layer:** separate hard limits based on innovation magnitude and estimator confidence.

That split keeps estimator timing deterministic while still letting RL get cleaner state than raw IMU/encoder data.

If you are vectorizing, keep EKF outside vectorized environment and expose only a deterministic emulation branch for rollout loops.

---

## 🛠️ EKF in Ascento and UMV Context

- **Ascento**: published platform references explicitly mention Kalman-state estimation and tight low-level balancing control. For nonlinear state dynamics, the implementation pattern is equivalent to EKF-style state fusion, but check the exact branch if you need strict proof of `extendedKalmanFilter` vs simple linearized KF in firmware.[^ascento-paper]
- **RAI UMV**: the public design mentions estimator structure around multi-source state reconstruction and RL-driven behavior. The current summaries indicate heavy state reconstruction (IMUs, encoders, trajectory priors, and map-free policy deployment), but not a declared EKF-only pipeline in public documents. Treat EKF usage as probable but **not confirmed** unless you verify the exact open implementation details or supplemental code release.[^umv-paper]

For sim-to-real work, this means:

- In Ascento, EKF-style state fusion is a strong candidate for your `obs` front-end.
- In UMV, if you cannot verify EKF, keep estimator interface abstract and do policy-level adaptation to multiple estimator styles.

---

## 🧪 Suggested Observation Contract for PufferLib

For a wheeled/legged RL environment, an estimator-derived observation often works better than raw sensor channels:

```text
obs = [
  quat_xyzw, angular_rate, linear_accel,
  joint_positions, joint_velocities,
  estimated_body_vel, tilt_estimation_confidence,
  bias_terms_if_exposed
]
```

Keep `innovation_norm` and `estimator_confidence` as explicit channels; it gives your policy and reward model cheap diagnostics for observability collapse and dropouts.

---

## 🧭 Source Strength and Limits

Primary sources for EKF mechanics:
- MathWorks explicitly defines EKF linearization and command flow (`predict` / `correct`), with model equations and Jacobian requirements.[^mathworks-ekf]
- The classic undergrad derivation and implementation lineage are mirrored by `trackingEKF` references and Jacobian-based nonlinear state-space formulation.[^welch]

Use those as the implementation reference; use platform papers as integration evidence.

---

## 🔗 Related Topics

- [[Kalman Filter]]
- [[State Estimation]]
- [[Sensor Fusion]]
- [[SLAM]]
- [[IMU]]
- [[Ascento]]
- [[RAI Ultra Mobility Vehicle]]
- [[Trajectory Optimization and MPC]]
- [[Linear Quadratic Regulator]]
- [[PufferLib]]

---

## 📚 Further Reading

- MathWorks: Extended Kalman Filter (trackingEKF) and related filter flow. https://www.mathworks.com/help/driving/ug/extended-kalman-filters.html
- MathWorks: Extended and Unscented Kalman filter comparison and algorithm details. https://www.mathworks.com/help/control/ug/extended-and-unscented-kalman-filter-algorithms-for-online-state-estimation.html
- Optimal State Estimation: Kalman, H-Infinity, and Nonlinear Approaches.
- The Kalman Filter tutorial by Welch & Bishop (pdf). https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf

---

## Source Notes

[^ascento-paper]: Ascento ICRA work and follow-on controls papers explicitly reference Kalman-style state estimation in the control stack and a reduced-state control model for balancing and jumping dynamics. https://ar5iv.labs.arxiv.org/html/2005.11435
[^umv-paper]: RAI UMV design and RL paper/public-facing notes indicate multi-sensor state reconstruction and high-rate control stack, but do not provide a public, explicit EKF implementation contract in summary-level materials. https://arxiv.org/abs/2602.22118
[^mathworks-ekf]: MathWorks Extended Kalman Filter examples and API flow for `predict` / `correct` on nonlinear models. https://www.mathworks.com/help/driving/ug/extended-kalman-filters.html
[^mathworks-ekf-algos]: MathWorks Extended and Unscented Kalman filter comparison and algorithm details. https://www.mathworks.com/help/control/ug/extended-and-unscented-kalman-filter-algorithms-for-online-state-estimation.html
[^simon-book]: Simon, D. (2006). *Optimal State Estimation: Kalman, H-Infinity, and Nonlinear Approaches*. Wiley.
[^welch]: Welch, G., & Bishop, G. (2006). *An Introduction to the Kalman Filter*. https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf
[^kalman-tutorial-pdf]: (alias) The Kalman filter tutorial by Welch & Bishop. https://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf
