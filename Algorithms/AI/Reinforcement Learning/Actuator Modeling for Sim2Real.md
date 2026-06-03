# ⚙️ Actuator Modeling for Sim2Real

**Actuator Modeling for Sim2Real** is the practice of making simulated motors, servos, gearboxes, and low-level controllers behave enough like real hardware that a learned policy can transfer. It is one of the most important bridges between [[Reinforcement Learning]] and real robotics.

---

## 📚 Overview

Many Sim2Real failures are blamed on the RL algorithm when the real issue is actuator mismatch. A policy trained with instant torque, no latency, no saturation, and perfect tracking will often fail on a robot with gearbox backlash, thermal limits, motor lag, voltage sag, and embedded controller filtering.

Good actuator modeling makes [[Domain Randomization]] more useful because randomization starts around realistic parameters.

---

## 🧠 Core Concepts

- **Torque Limit**: Maximum joint effort available from the motor and gearbox.
- **Velocity Limit**: Maximum achievable joint speed.
- **PD Control**: Low-level proportional-derivative servo behavior used under many learned policies.
- **Latency**: Delay between policy output and physical motion.
- **Saturation**: Clipping caused by current, voltage, torque, velocity, or command limits.
- **Backlash**: Dead zone caused by mechanical looseness.
- **Friction**: Coulomb, viscous, and static friction in joints and transmissions.
- **Action Rate Limit**: Bound on how quickly commands may change.

---

## 📊 Comparison Chart

| Actuator Model | Fidelity | Best For | Strength | Weakness |
|---|---|---|---|---|
| Ideal torque source | Low | Algorithm debugging | Simple and fast | Unrealistic transfer |
| PD joint target | Medium | Legged locomotion, arms | Common deployment pattern | Needs gain tuning |
| DC motor model | Medium-high | Embedded robotics | Captures voltage/current effects | More parameters |
| Servo model | Medium | Hobby arms, grippers | Matches command interface | Hides internal controller |
| Learned actuator net | High | Legged Sim2Real | Captures unmodeled dynamics | Needs real data |
| Full drivetrain model | High | Precision robotics | Physical interpretability | Expensive to identify |

---

## ✅ Pros

- Improves zero-shot and few-shot [[Sim2Real]] transfer.
- Reduces policy reliance on impossible accelerations.
- Makes action limits and safety constraints explicit.
- Helps explain failures seen only on hardware.
- Works with [[MuJoCo]], [[Isaac Lab]], and custom [[PufferLib]] environments.

---

## ❌ Cons

- Requires hardware measurements or datasheets.
- Adds parameters that can be hard to identify.
- Overly complex models can slow iteration.
- Bad actuator models can be worse than simple conservative limits.
- Learned actuator models need clean logs and careful validation.

---

## 🧰 Practical Checklist

1. Start with real joint limits, torque limits, and velocity limits.
2. Match the real control frequency and command interface.
3. Add command latency and observation latency.
4. Add action clipping and action-rate limits.
5. Tune PD gains against simple step-response logs.
6. Randomize actuator strength and latency during training.
7. Validate with hardware logs before deploying aggressive policies.

---

## 🔗 Related Notes

- [[Sim2Real]]
- [[System Identification for Sim2Real]]
- [[Domain Randomization]]
- [[Robot Policy Deployment]]
- [[MuJoCo MJCF]]
- [[Isaac Lab Task Authoring]]
- [[FOC]]

---

## 🌐 External Resources

- ETH RSL locomotion research: https://rsl.ethz.ch/research/researchtopics/rl-robotics.html
- Isaac Lab Docs: https://isaac-sim.github.io/IsaacLab/main/

---

## 📝 Summary

Actuator modeling turns a simulated control output into something a real robot could plausibly execute. For robotics RL, it is as important as reward design because the policy learns the actuator model it is given.
