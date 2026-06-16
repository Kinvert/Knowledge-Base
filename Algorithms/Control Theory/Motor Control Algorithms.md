---
title: Motor Control Algorithms
aliases:
  - Motor Control
  - Motor Control Methods
  - Motor Control Loop
tags:
  - control-theory
  - motors
  - actuators
  - robotics
---

# Motor Control Algorithms

**Motor control algorithms** are methods for commanding motor drives so that speed, torque, position, or force follow a desired trajectory under real-world constraints.

They appear across robotics, drones, CNC machines, and industrial motion systems, where motors must remain stable despite disturbances, sensor noise, load changes, and nonlinear effects.

---

## Core control families

### Feedback loops
- **PID control**: Widely used default for speed or position loops.
- **Cascade control**: Nested loops (often current, speed, then position) for better bandwidth and disturbance rejection.
- **Gain scheduling**: Adjusts gains across operating regimes.

### Model-based methods
- **State-space control**: Uses explicit state models for multi-axis coupling and constraints.
- **LQR / MPC**: Optimizes control effort while tracking trajectories and respecting limits.
- **Observer-aided control**: Pairing observers (e.g., [[Kalman Filter]]) with controllers when states are not directly measurable.

### Modern motor-oriented methods
- **Field-oriented control (FOC)**: Transforms motor currents to a rotating frame for high-efficiency torque control.
- **Vector control / DTC**: Direct torque approaches tuned for torque response.
- **Adaptive control**: Adjusts parameters online for load or temperature changes.
- **Learning-augmented control**: Adds learned compensation for unmodeled effects.

---

## Why it matters

Motor dynamics are often nonlinear and sensitive to:
- Back-EMF and saturation
- Coulomb friction and stiction
- Cogging and mechanical resonances
- Variable loads and battery droop

A controller choice determines not only stability but also:
- Responsiveness (rise time)
- Overshoot and precision
- Power efficiency
- Mechanical stress and vibration

---

## Comparison table

| Method | Primary Objective | Typical Use | Responsiveness | Modeling Need | Robustness to Load Change |
|---|---|---|---|---|---|
| PID | Error minimization | Cheap single-axis systems | Low-Medium | Low | Medium |
| Cascade PID | Multi-stage stability | Industrial drives | Medium | Low-Medium | Medium-High |
| FOC | Torque and efficiency | AC/BLDC precision drives | High | Medium-High | High |
| MPC | Constraint-aware optimization | High-end robotics, CNC | Medium | High | High |
| LQR | Regulated state tracking | Aerial/ground platforms | Medium | Medium | Medium |
| Adaptive control | Varying dynamics | Unknown payloads, changing conditions | Medium | Medium | High |

---

## Common hardware + software pairings

- [[PID Controller]] + incremental controller
- [[FOC]] for BLDC and PMSM control
- [[Motor Controllers]] for hardware abstraction
- `PWM` signal generation for low-level actuation
- Encoder + IMU feedback for closed-loop estimation

---

## Strengths and limitations

### ✅ Pros
- Greatly improves repeatability and trajectory accuracy.
- Can isolate disturbances quickly when loop design is solid.
- Enables energy-efficient operation by minimizing unnecessary corrections.

### ⚠️ Cons
- Aggressive tuning can destabilize mechanically coupled systems.
- Accurate models are often required for advanced methods.
- Higher algorithmic complexity increases integration and tuning time.

---

## Related notes

- [[Control Theory]]
- [[PID Controller]]
- [[Drive-Train Modeling for Balancing Robots]]
- [[Kalman Filter]]
- [[State Estimation]]

