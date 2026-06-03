# 🧩 RLBench

**RLBench** is a robot learning benchmark built around a large collection of manipulation tasks with demonstrations. It is often used for vision-based control, imitation learning, language-conditioned robotics, and task generalization.

---

## 📚 Overview

RLBench emphasizes task diversity and demonstrations. It is useful for learning how vision, language, and manipulation datasets interact, especially before moving to real robot datasets such as [[DROID]] or [[Open X-Embodiment]].

---

## 🧠 Core Concepts

- **Task Demonstrations**: Expert trajectories for imitation learning.
- **Vision-Based Control**: Policies often consume camera observations.
- **Language Condition**: Some workflows pair task instructions with robot behavior.
- **Task Variation**: Same task family with different object poses or settings.
- **Benchmark Protocol**: Standard splits for evaluating generalization.

---

## 📊 Comparison Chart

| Benchmark | Focus | Strength | Weakness | Robotics Fit |
|---|---|---|---|---|
| **RLBench** | Vision manipulation | Many demos and tasks | CoppeliaSim stack | High for IL research |
| [[LIBERO]] | Lifelong learning | Strong generalization suites | Benchmark-specific | High |
| [[ManiSkill]] | Manipulation RL/IL | Many assets | More physics/task complexity | High |
| [[RoboSuite]] | MuJoCo arms | Clean control | Smaller task scope | Medium-high |
| [[Meta-World]] | Multi-task RL | Compact tasks | Simplified realism | Medium |
| [[LeRobot]] | Real robot data | Hardware workflow | Younger ecosystem | Very high |

---

## ✅ Pros

- Large collection of manipulation tasks.
- Useful for vision-based imitation learning.
- Demonstrations reduce exploration difficulty.
- Good for testing task generalization.
- Connects to language-conditioned robot learning ideas.

---

## ❌ Cons

- Simulator stack differs from MuJoCo and Isaac.
- Benchmark success may not imply real robot success.
- Data and task setup can be heavier than simple Gym tasks.
- Less aligned with PufferLib-style fast custom envs.

---

## 🔗 Related Notes

- [[Imitation Learning]]
- [[Manipulation RL]]
- [[Robot Dataset Formats]]
- [[LeRobot]]
- [[Open X-Embodiment]]

---

## 🌐 External Resources

- RLBench GitHub: https://github.com/stepjam/RLBench
- RLBench Paper: https://arxiv.org/abs/1909.12271

---

## 📝 Summary

RLBench is valuable for studying demonstration-rich manipulation and vision-language robot learning. It is more useful as a benchmark and dataset environment than as a direct deployment stack.
