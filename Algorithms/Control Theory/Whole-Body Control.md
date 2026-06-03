# 🧍 Whole-Body Control

**Whole-Body Control** coordinates many joints and contacts to satisfy multiple robot tasks at once. It is common in humanoids, quadrupeds, mobile manipulators, and legged systems where balance, contact forces, posture, and task goals must be solved together.

---

## 📚 Overview

Whole-body control often uses optimization to compute joint torques, accelerations, or contact forces subject to constraints. In RL robotics, it can serve as the low-level controller under a learned high-level policy, or as a safety layer that keeps learned actions physically feasible.

---

## 🧠 Core Concepts

- **Task Hierarchy**: Prioritized objectives such as balance, foot contact, hand pose, and posture.
- **Contact Constraints**: Conditions for stance feet, hands, or wheels.
- **Centroidal Dynamics**: Dynamics of the robot center of mass and angular momentum.
- **QP Control**: Quadratic program used to solve constrained control commands.
- **Friction Cone**: Constraint on allowable contact forces.
- **Null-Space Control**: Secondary objectives that do not disturb primary tasks.

---

## 📊 Comparison Chart

| Controller | Best For | Strength | Weakness | RL Relationship |
|---|---|---|---|---|
| Joint PD | Simple robots | Easy and robust | Limited coordination | Tracks policy targets |
| Operational Space Control | Arms | Task-space behavior | Fewer whole-body constraints | Policy outputs end-effector goals |
| Whole-Body Control | Legged/full-body robots | Multi-task constraints | Complex model/control setup | Low-level policy interface |
| MPC | Dynamic planning | Predictive constraints | Compute heavy | Policy can guide goals |
| Direct RL torque | Research policies | Maximum flexibility | Hard Sim2Real | Can replace or augment WBC |
| Behavior Trees | Task logic | Modular sequencing | Not dynamics control | Calls learned skills |

---

## ✅ Pros

- Handles multiple simultaneous tasks and constraints.
- Good fit for legged robots and humanoids.
- Can make learned high-level commands safer.
- Uses explicit dynamics and contact models.
- Integrates with model libraries such as [[Pinocchio]].

---

## ❌ Cons

- Requires accurate robot model and state estimation.
- QP tuning and task weights can be complex.
- Contact switching is hard.
- Real-time implementation requires careful engineering.
- May limit learned behavior if constraints are too conservative.

---

## 🧰 RL Connection

- RL policy can output desired velocity, footstep, posture, or end-effector commands.
- Whole-body controller converts commands into feasible torques or accelerations.
- Useful for [[Legged Locomotion RL]], humanoids, and mobile manipulation.
- Helps deploy policies without giving them raw motor authority.

---

## 🔗 Related Notes

- [[Operational Space Control]]
- [[Trajectory Optimization and MPC]]
- [[Robot Dynamics and Spatial Algebra]]
- [[Legged Locomotion RL]]
- [[RL and Classical Control]]
- [[Pinocchio]]
- [[Sim2Real]]

---

## 📝 Summary

Whole-body control is a practical way to combine learned behavior with physics constraints. For advanced robotics RL, it is one of the main bridges between policy output and stable full-body motion.
