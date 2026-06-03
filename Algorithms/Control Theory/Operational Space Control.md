# 🦾 Operational Space Control

**Operational Space Control** controls a robot in task space, such as end-effector position, orientation, force, or impedance, instead of directly controlling individual joints. It is a key bridge between [[Inverse Kinematics]], dynamics, manipulation, and [[Reinforcement Learning]] policies that output task-space commands.

---

## 📚 Overview

For robot arms, the task usually happens at the end effector: reach a pose, follow a trajectory, apply a force, or maintain compliant contact. Operational space control maps task-space objectives through the robot Jacobian and dynamics into joint commands.

---

## 🧠 Core Concepts

- **Task Space**: Cartesian space where the end effector or body task is defined.
- **Joint Space**: Robot configuration space.
- **Jacobian**: Maps joint velocities to task-space velocities.
- **Operational Space Inertia**: Effective inertia seen at the end effector.
- **Null Space**: Joint motion that does not disturb the primary task.
- **Impedance Control**: Controls stiffness and damping rather than exact position.

---

## 📊 Comparison Chart

| Method | Command Space | Strength | Weakness | Common Robotics Use |
|---|---|---|---|---|
| Joint PD | Joint space | Simple and stable | Not task-aware | Low-level tracking |
| [[Inverse Kinematics]] | Task pose to joints | Easy for reaching | Ignores dynamics | Arm positioning |
| Operational Space Control | Task space with dynamics | Natural manipulation control | Needs model/Jacobian | Contact-rich arms |
| Whole-Body Control | Multiple task constraints | Handles full robots | Complex QP setup | Humanoids, quadrupeds |
| MPC | Future trajectory | Constraint-aware | Compute heavy | Dynamic motion |
| Direct RL torque | Learned joint efforts | Expressive | Hard to deploy safely | Research control |

---

## ✅ Pros

- Matches how many manipulation tasks are specified.
- Supports force and compliance behavior.
- Can use null-space motion for posture or joint-limit avoidance.
- Useful action interface for learned policies.
- Works well with [[MoveIt]], [[Pinocchio]], and dynamics libraries.

---

## ❌ Cons

- Needs accurate kinematics and often dynamics.
- Singularities can cause unstable commands.
- Force control requires careful sensing and safety.
- Real-time implementation is more complex than joint PD.
- Model mismatch can hurt contact behavior.

---

## 🧰 RL Connection

- RL can output target end-effector deltas.
- OSC handles conversion to joint torques or accelerations.
- Policy actions become easier to learn and safer to bound.
- Useful for [[Manipulation RL]] and imitation learning policies.

---

## 🔗 Related Notes

- [[Inverse Kinematics]]
- [[Whole-Body Control]]
- [[Robot Dynamics and Spatial Algebra]]
- [[MoveIt]]
- [[Pinocchio]]
- [[Manipulation RL]]
- [[RL and Classical Control]]

---

## 📝 Summary

Operational space control is useful when the policy should reason about the task, not every motor. It gives robotics RL a practical action interface for manipulation and contact.
