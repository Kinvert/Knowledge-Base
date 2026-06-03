# 🌐 Meta-World

**Meta-World** is a benchmark suite of MuJoCo-based robotic manipulation tasks designed for meta-learning, multi-task reinforcement learning, and generalization research.

---

## 📚 Overview

Meta-World provides many small tabletop manipulation tasks with a shared robot arm. It is useful for studying whether a policy or algorithm can generalize across task variations, but it is less focused on high-fidelity deployment than [[Isaac Lab]] or real robot imitation pipelines.

---

## 🧠 Core Concepts

- **Multi-Task RL**: Training one policy across many tasks.
- **Meta-Learning**: Learning to adapt quickly to new tasks.
- **Task Families**: Related manipulation tasks with shared structure.
- **Goal Conditioning**: Conditioning policy behavior on task or goal.
- **Benchmark Splits**: Standard train/test task splits for evaluation.

---

## 📊 Comparison Chart

| Benchmark | Focus | Simulator | Strength | Weakness |
|---|---|---|---|---|
| **Meta-World** | Multi-task manipulation | [[MuJoCo]] | Many compact tasks | Simplified realism |
| [[RoboSuite]] | Arm manipulation | MuJoCo | Clean robot tasks | Fewer task families |
| [[ManiSkill]] | Manipulation RL/IL | SAPIEN/PhysX | Rich assets | More complex |
| [[RLBench]] | Vision-language tasks | CoppeliaSim | Demonstrations | Different stack |
| [[LIBERO]] | Lifelong learning | robosuite | Generalization suites | Benchmark-specific |
| [[RoboCasa]] | Household tasks | MuJoCo | Realistic scenes | Narrow domain |

---

## ✅ Pros

- Good for multi-task and meta-RL experiments.
- Compact tasks make algorithm comparison easier.
- Uses familiar MuJoCo-style continuous control.
- Useful bridge from simple RL to manipulation.
- Standardized benchmark splits.

---

## ❌ Cons

- Simplified compared with real manipulation.
- Less focused on sensors and deployment.
- Sim2Real relevance is indirect.
- Task success can depend on benchmark-specific details.

---

## 🔗 Related Notes

- [[Manipulation RL]]
- [[MuJoCo]]
- [[PPO]]
- [[SAC]]
- [[Imitation Learning]]

---

## 🌐 External Resources

- Meta-World GitHub: https://github.com/Farama-Foundation/Metaworld
- Meta-World Paper: https://arxiv.org/abs/1910.10897

---

## 📝 Summary

Meta-World is most useful for studying task generalization and multi-task RL in manipulation, not for direct real robot deployment.
