# 🦾 ManiSkill

**ManiSkill** is a robot manipulation benchmark and simulation framework for reinforcement learning, imitation learning, and visuomotor policy research. It focuses on diverse manipulation tasks, objects, and scalable simulation.

---

## 📚 Overview

ManiSkill is useful for learning manipulation because it provides many tasks and assets beyond classic MuJoCo locomotion benchmarks. It is a good bridge between [[Manipulation RL]], [[Imitation Learning]], [[Offline RL for Robotics]], and modern simulation-based robot learning.

---

## 🧠 Core Concepts

- **Manipulation Tasks**: Grasping, pushing, assembly, opening, and tool-like interactions.
- **Object Diversity**: Varied object shapes and task layouts.
- **State and Vision Observations**: Supports low-dimensional and image-based policies.
- **RL and IL Support**: Designed for both reinforcement and imitation workflows.
- **Benchmark Protocols**: Standardized tasks for comparing methods.

---

## 📊 Comparison Chart

| Benchmark | Simulator | Best For | Strength | Weakness |
|---|---|---|---|---|
| **ManiSkill** | SAPIEN/PhysX | Manipulation RL/IL | Many tasks and assets | Benchmark complexity |
| [[RoboSuite]] | [[MuJoCo]] | Robot arm manipulation | Clean research API | Less large-scale |
| [[Meta-World]] | MuJoCo | Multi-task manipulation | Many small tasks | Simplified tasks |
| [[RLBench]] | CoppeliaSim | Vision-language tasks | Rich task suite | Different simulator stack |
| [[LIBERO]] | robosuite | Lifelong robot learning | Task suites for generalization | Benchmark-specific |
| [[RoboCasa]] | MuJoCo | Household manipulation | Realistic kitchen tasks | Domain-specific |

---

## ✅ Pros

- Strong manipulation benchmark coverage.
- Supports visual and state-based learning.
- Useful for both RL and imitation learning.
- More robotics-specific than classic control benchmarks.
- Good place to test generalization across objects and tasks.

---

## ❌ Cons

- More complex than simple Gymnasium tasks.
- Benchmark success does not guarantee real robot transfer.
- Contact and asset issues still require simulator knowledge.
- Setup can be heavier than classic MuJoCo benchmarks.

---

## 🔗 Related Notes

- [[Manipulation RL]]
- [[Imitation Learning]]
- [[Isaac Lab]]
- [[MuJoCo]]
- [[Robot Dataset Formats]]

---

## 🌐 External Resources

- ManiSkill Website: https://www.maniskill.ai/
- ManiSkill GitHub: https://github.com/haosulab/ManiSkill

---

## 📝 Summary

ManiSkill is a high-value benchmark for robot manipulation because it exposes policies to richer tasks and objects than classic locomotion environments.
