# 🧠 LIBERO

**LIBERO** is a benchmark suite for lifelong robot learning and manipulation generalization. It builds on robosuite-style environments and focuses on learning policies that transfer across tasks, objects, goals, and scenes.

---

## 📚 Overview

LIBERO is useful for studying how robot policies generalize beyond one fixed task. It is especially relevant to [[Imitation Learning]], [[Manipulation RL]], and offline policy learning because it provides structured task suites and demonstrations.

---

## 🧠 Core Concepts

- **Lifelong Learning**: Learning new tasks without forgetting old ones.
- **Task Suites**: Organized groups of manipulation tasks.
- **Demonstration Data**: Expert examples for imitation learning.
- **Generalization Split**: Evaluation on held-out tasks, objects, or scenes.
- **Policy Transfer**: Reusing behavior across related manipulation settings.

---

## 📊 Comparison Chart

| Benchmark | Main Focus | Strength | Weakness | Related Stack |
|---|---|---|---|---|
| **LIBERO** | Lifelong manipulation | Generalization task suites | Benchmark-specific | robosuite |
| [[RoboSuite]] | Manipulation simulation | Clean task/control API | Less lifelong focus | MuJoCo |
| [[robomimic]] | Imitation training | Reproducible baselines | Not a simulator | HDF5 datasets |
| [[RLBench]] | Vision demonstrations | Many tasks | Different simulator | CoppeliaSim |
| [[ManiSkill]] | Manipulation RL/IL | Many tasks/assets | More complex | SAPIEN |
| [[LeRobot]] | Real robot imitation | Hardware/data workflow | Young ecosystem | Real robots |

---

## ✅ Pros

- Focuses on generalization, not just single-task success.
- Good fit for imitation and offline learning.
- Useful for evaluating forgetting and transfer.
- Connects well to robosuite and robomimic concepts.
- Helps study long-horizon manipulation.

---

## ❌ Cons

- Benchmark-specific skills may not transfer directly.
- Not a real robot deployment framework.
- Requires understanding dataset splits and evaluation protocol.
- Adds complexity beyond simple manipulation tasks.

---

## 🔗 Related Notes

- [[Manipulation RL]]
- [[Imitation Learning]]
- [[RoboSuite]]
- [[robomimic]]
- [[Offline RL for Robotics]]

---

## 🌐 External Resources

- LIBERO GitHub: https://github.com/Lifelong-Robot-Learning/LIBERO
- LIBERO Project: https://libero-project.github.io/

---

## 📝 Summary

LIBERO is a useful benchmark for studying how manipulation policies generalize and retain skills across tasks. It is most relevant after basic imitation learning is understood.
