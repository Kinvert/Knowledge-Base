# 🧍 HumanoidBench

**HumanoidBench** is a benchmark focused on humanoid robot control tasks, usually involving whole-body motion, locomotion, manipulation, and long-horizon control challenges.

---

## 📚 Overview

Humanoid control is harder than quadruped or arm-only control because the policy must coordinate balance, contacts, posture, locomotion, and manipulation. HumanoidBench-style environments are useful for testing whether algorithms scale to high-dimensional whole-body control.

---

## 🧠 Core Concepts

- **High-DoF Control**: Many joints and contacts must be coordinated.
- **Whole-Body Tasks**: Locomotion, reaching, carrying, standing, and recovery.
- **Balance and Contacts**: Falls and foot contacts dominate learning.
- **Long-Horizon Control**: Tasks often require extended coordinated behavior.
- **Sim2Real Difficulty**: Actuators, contacts, and safety are major barriers.

---

## 📊 Comparison Chart

| Benchmark / Stack | Focus | Strength | Weakness | Related Notes |
|---|---|---|---|---|
| **HumanoidBench** | Humanoid control | High-dimensional challenge | Difficult and expensive | [[Whole-Body Control]] |
| [[Legged Gym]] | Quadruped locomotion | Proven Sim2Real | Less humanoid focus | [[RSL-RL]] |
| [[Isaac Lab]] | General robot learning | Scalable humanoid tasks | Heavy setup | [[Isaac Lab Task Authoring]] |
| [[MuJoCo]] humanoid | Classic benchmark | Simple baseline | Less realistic | [[MuJoCo MJCF]] |
| [[Brax]] humanoid | Fast JAX control | High throughput | Asset/deployment gap | [[JAX]] |
| [[RoboCasa]] | Household manipulation | Rich scenes | Not humanoid-specific | [[Manipulation RL]] |

---

## ✅ Pros

- Tests algorithms on difficult full-body control.
- Connects locomotion, manipulation, and balance.
- Useful for humanoid robotics research.
- Encourages whole-body control thinking.
- Strong stress test for observation/action design.

---

## ❌ Cons

- Harder than most first robotics RL projects.
- Real humanoid deployment is high-risk and expensive.
- Requires strong actuator modeling and safety.
- Debugging failures can be difficult.
- Benchmark details may dominate results.

---

## 🔗 Related Notes

- [[Whole-Body Control]]
- [[Legged Locomotion RL]]
- [[Isaac Lab]]
- [[MuJoCo]]
- [[Safe RL for Robotics]]
- [[Actuator Modeling for Sim2Real]]

---

## 🌐 External Resources

- HumanoidBench Project: https://humanoid-bench.github.io/

---

## 📝 Summary

HumanoidBench is a high-difficulty benchmark area for robotics RL. It is best approached after understanding legged locomotion, whole-body control, and Sim2Real constraints.
