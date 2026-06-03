# 🎚️ RL and Classical Control

**RL and Classical Control** describes how learned policies are combined with controllers such as [[PID Controller]], inverse kinematics, model predictive control, and state estimators. In robotics, RL usually works best as one layer in a control stack, not as a replacement for all control logic.

---

## 📚 Overview

Classical control provides stability, constraints, interpretability, and known failure behavior. [[Reinforcement Learning]] provides adaptation, nonlinear behavior, and task-level optimization. Real robots often combine them: the learned policy outputs targets or residuals, while classical controllers enforce timing, limits, and tracking.

---

## 🧠 Core Concepts

- **Policy as Setpoint Generator**: RL outputs desired joint positions, velocities, or body commands.
- **Residual RL**: RL learns a correction on top of a classical controller.
- **Hierarchical Control**: High-level policy selects goals, low-level controller executes them.
- **Safety Filter**: Classical layer clips or rejects unsafe learned actions.
- **State Estimation**: Filters produce the state used by both the policy and controller.
- **Controller Frequency Split**: Policy may run slower than the low-level motor loop.

---

## 📊 Comparison Chart

| Control Pattern | RL Role | Classical Role | Strength | Weakness |
|---|---|---|---|---|
| Direct torque policy | Full control | Safety only | Maximum expressiveness | Hardest Sim2Real |
| Joint target policy | Setpoints | PD tracking | Practical deployment | Depends on gains |
| Residual RL | Correction | Baseline controller | Safer learning | Limited by baseline |
| MPC + RL | Cost/model/prior | Constrained optimizer | Handles constraints | More compute |
| Behavior tree + RL | Skill policy | Task orchestration | Modular robot behavior | Integration complexity |
| Imitation policy + controller | Motion generation | Tracking/safety | Good for manipulation | Dataset dependent |

---

## ✅ Pros

- Makes deployment safer and more interpretable.
- Reduces action-space difficulty for RL.
- Allows different loop rates for policy and motor control.
- Works with [[ros2_control]] and embedded controllers.
- Improves Sim2Real transfer by matching real control interfaces.

---

## ❌ Cons

- Bad low-level controllers can hide or distort policy behavior.
- RL may exploit controller quirks.
- More layers make debugging harder.
- Residual policies can exceed safe behavior if not bounded.
- Requires careful observation and action contract design.

---

## 🧰 Practical Robotics Pattern

1. Use RL to output joint targets or velocity commands.
2. Track those commands with PD, impedance, or whole-body control.
3. Clip actions and action rates.
4. Run state estimation separately from the policy.
5. Add fallback behavior for policy failure.
6. Log both policy actions and controller outputs.

---

## 🔗 Related Notes

- [[Reinforcement Learning]]
- [[PID Controller]]
- [[Inverse Kinematics]]
- [[Operational Space Control]]
- [[Trajectory Optimization and MPC]]
- [[Robot Policy Deployment]]
- [[Safe RL for Robotics]]

---

## 📝 Summary

RL becomes more useful in robotics when paired with classical control. The learned policy should solve the hard behavioral part, while the control stack handles tracking, limits, timing, and safety.
