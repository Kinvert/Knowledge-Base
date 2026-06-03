# 📈 Trajectory Optimization and MPC

**Trajectory Optimization and Model Predictive Control (MPC)** compute control sequences that optimize future motion under dynamics and constraints. They are central tools for robotics and a useful comparison point for [[Reinforcement Learning]].

---

## 📚 Overview

RL learns a policy from interaction. Trajectory optimization solves an optimization problem for a specific start state and goal. MPC repeatedly solves a short-horizon optimization problem online, executes the first action, and replans. In robotics, RL and MPC are often combined: RL can provide priors, costs, residuals, or warm starts.

---

## 🧠 Core Concepts

- **Trajectory Optimization**: Finds a sequence of states and controls that minimize a cost.
- **MPC**: Receding-horizon control using repeated online optimization.
- **Dynamics Model**: Predicts how controls change future states.
- **Cost Function**: Encodes goals, energy, smoothness, and constraints.
- **Warm Start**: Initial guess from a previous solution or learned policy.
- **Constraint Handling**: Joint limits, collision constraints, torque bounds, and contact constraints.

---

## 📊 Comparison Chart

| Method | Model Needed | Online Compute | Strength | Weakness |
|---|---|---|---|---|
| [[PPO]] policy | No explicit model | Low | Fast inference | Needs training data |
| iLQR/DDP | Yes | Medium | Strong local control | Local minima |
| CEM/MPPI | Yes or simulator | Medium-high | Handles non-smooth costs | Many rollouts |
| MPC | Yes | High | Constraint-aware | Real-time burden |
| CHOMP/STOMP | Kinematic/dynamic costs | Medium | Good trajectory shaping | Not reactive alone |
| Behavior cloning | No explicit model | Low | Easy deployment | Dataset limited |

---

## ✅ Pros

- Explicitly handles costs and constraints.
- Good for known dynamics and safety-critical behavior.
- Useful for generating demonstrations.
- Can supervise or initialize learned policies.
- Provides a strong baseline against RL policies.

---

## ❌ Cons

- Requires a good model or simulator.
- Online optimization can be too slow for fast robots.
- Cost design can be as hard as reward design.
- Contact-rich tasks are difficult.
- Local methods can fail with poor initialization.

---

## 🧰 RL Connections

- Use MPC to generate expert data for [[Imitation Learning]].
- Distill MPC behavior into a neural policy for fast inference.
- Use RL to learn cost functions or residual terms.
- Use learned dynamics models inside MPC.
- Compare RL policies against MPC baselines before deployment.

---

## 🔗 Related Notes

- [[RL and Classical Control]]
- [[Whole-Body Control]]
- [[Operational Space Control]]
- [[CHOMP]]
- [[RRT]]
- [[Model-Based RL]]
- [[Robot Dynamics and Spatial Algebra]]

---

## 📝 Summary

Trajectory optimization and MPC are not competitors to robotics RL so much as complementary tools. They provide constraint-aware control, expert demonstrations, warm starts, and strong baselines for learned policies.
