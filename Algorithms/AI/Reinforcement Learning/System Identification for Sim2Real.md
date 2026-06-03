# 📏 System Identification for Sim2Real

**System Identification for Sim2Real** is the process of measuring real robot parameters and tuning simulation so that simulated behavior matches hardware behavior. It complements [[Domain Randomization]] by anchoring randomization ranges around reality.

---

## 📚 Overview

Sim2Real is easier when the simulator starts near the real system. System identification estimates parameters such as mass, inertia, friction, damping, motor delay, sensor noise, and contact behavior. The goal is not a perfect simulator. The goal is a simulator accurate enough that a learned policy does not exploit impossible dynamics.

---

## 🧠 Core Concepts

- **Parameter Identification**: Estimating physical values from measurements.
- **Trajectory Matching**: Comparing simulated and real state trajectories under the same commands.
- **Step Response**: Applying simple commands to reveal delay, gain, damping, and overshoot.
- **Residual Error**: Difference between simulated and real behavior.
- **Identifiability**: Whether the available data can actually determine a parameter.
- **Validation Set**: Real logs not used during fitting.

---

## 📊 Comparison Chart

| Method | Best For | Strength | Weakness | Robotics Use |
|---|---|---|---|---|
| Datasheet parameters | First model | Fast starting point | Often optimistic | Motors, sensors |
| Manual tuning | Small systems | Simple and intuitive | Not reproducible | Friction, damping |
| Step-response fitting | Actuators | Captures delay/gain | Limited operating range | Joint control |
| Trajectory optimization | Dynamics | Uses real motion logs | Needs good data | Arms, legs |
| Bayesian optimization | Black-box tuning | Handles messy sims | Sample expensive | Contact parameters |
| Differentiable simulation | Gradient fitting | Efficient when available | Model assumptions matter | [[MuJoCo]], [[MJX]] |

---

## ✅ Pros

- Makes simulation failures easier to diagnose.
- Reduces the amount of randomization needed.
- Gives concrete parameters for actuator and sensor models.
- Helps distinguish policy bugs from model bugs.
- Produces logs useful for deployment and safety reviews.

---

## ❌ Cons

- Requires careful real-world experiments.
- Some parameters are hard to identify independently.
- Contact and terrain parameters can vary by environment.
- Good fits on simple tests may not generalize.
- Hardware wear, temperature, and battery state can shift behavior.

---

## 🧰 What To Measure

- Joint position, velocity, torque or current.
- Command timestamps and sensor timestamps.
- Control loop frequency and jitter.
- Motor step responses under small and large commands.
- Static and dynamic friction.
- Camera, IMU, and encoder noise.
- Contact behavior on expected surfaces.

---

## 🔗 Related Notes

- [[Sim2Real]]
- [[Actuator Modeling for Sim2Real]]
- [[Domain Randomization]]
- [[MuJoCo MJCF]]
- [[Isaac Lab Task Authoring]]
- [[Kalman Filter]]
- [[Sensor Fusion]]

---

## 🌐 External Resources

- Sim-to-Real Transfer Survey: https://arxiv.org/abs/2009.13303
- MuJoCo Modeling Docs: https://mujoco.readthedocs.io/en/stable/modeling.html

---

## 📝 Summary

System identification makes Sim2Real less mystical. It turns real robot behavior into numbers that can be modeled, randomized, and validated before trusting a learned policy on hardware.
