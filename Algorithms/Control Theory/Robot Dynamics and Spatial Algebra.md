# 🧮 Robot Dynamics and Spatial Algebra

**Robot Dynamics and Spatial Algebra** describe how forces, torques, velocities, inertias, and joint motion interact in articulated robots. These concepts underpin [[MuJoCo]], [[Pinocchio]], whole-body control, trajectory optimization, and many [[Sim2Real]] workflows.

---

## 📚 Overview

Robotics RL can hide dynamics behind a simulator, but the learned policy is still shaped by those dynamics. Understanding mass matrices, Jacobians, inverse dynamics, contact forces, and spatial transforms helps explain why policies fail, why action spaces matter, and why model parameters must be identified.

---

## 🧠 Core Concepts

- **Forward Kinematics**: Computes body poses from joint positions.
- **Jacobian**: Maps joint velocities to task-space velocities.
- **Mass Matrix**: Configuration-dependent inertia of the robot.
- **Coriolis and Centrifugal Terms**: Velocity-dependent dynamics terms.
- **Gravity Compensation**: Torque needed to counter gravity.
- **Forward Dynamics**: Computes acceleration from state and applied forces.
- **Inverse Dynamics**: Computes required torque for desired motion.
- **Spatial Vector Algebra**: Compact 6D representation of motion and force.

---

## 📊 Comparison Chart

| Library / Tool | Main Role | Strength | Weakness | Robotics RL Use |
|---|---|---|---|---|
| [[Pinocchio]] | Rigid-body algorithms | Fast dynamics and derivatives | C++/Python complexity | High |
| [[MuJoCo]] | Simulation engine | Fast contacts and dynamics | Simulator-specific modeling | Very high |
| [[MJX]] | JAX MuJoCo | Accelerator-friendly | Newer ecosystem | High |
| RBDL | Dynamics library | Lightweight | Smaller ecosystem | Medium |
| Drake | Robotics toolbox | Broad planning/control | Heavy framework | High |
| PyBullet | Simulation | Easy to start | Lower fidelity for RL scaling | Medium |

---

## ✅ Pros

- Makes control and Sim2Real failures easier to understand.
- Helps choose action spaces and controllers.
- Supports system identification and model-based control.
- Useful for debugging URDF/MJCF asset problems.
- Connects RL policies to real robot physics.

---

## ❌ Cons

- Math-heavy compared with black-box simulation.
- Requires careful frame and unit conventions.
- Contact dynamics remain difficult.
- Full rigid-body models can still mismatch hardware.
- Not all RL workflows expose dynamics directly.

---

## 🧰 Robotics RL Relevance

- Joint target policies depend on controller and dynamics assumptions.
- Torque policies need realistic mass, damping, and actuator limits.
- Whole-body control and MPC require dynamics terms.
- Sim2Real transfer depends on identified masses, inertias, friction, and delays.
- Bad inertias or link frames can make policies exploit simulation artifacts.

---

## 🔗 Related Notes

- [[Pinocchio]]
- [[MuJoCo]]
- [[MuJoCo MJCF]]
- [[Whole-Body Control]]
- [[Operational Space Control]]
- [[System Identification for Sim2Real]]
- [[URDF to MJCF and USD Pipeline]]

---

## 📝 Summary

Robot dynamics are the physical substrate under robot learning. Even when using model-free RL, knowing the dynamics vocabulary helps debug simulators, controllers, policies, and Sim2Real failures.
