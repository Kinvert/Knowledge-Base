# 🏠 RoboCasa

**RoboCasa** is a simulation benchmark for household robot manipulation, especially kitchen-style tasks. It is built around realistic scenes, objects, and long-horizon manipulation workflows.

---

## 📚 Overview

RoboCasa is useful because many real robot manipulation tasks are not isolated reacher problems. They happen in cluttered, semantically meaningful environments with cabinets, drawers, counters, tools, and object arrangements. RoboCasa pushes manipulation learning toward those richer settings.

---

## 🧠 Core Concepts

- **Household Manipulation**: Tasks in kitchen and home-like scenes.
- **Long-Horizon Tasks**: Multi-step behavior beyond one grasp or push.
- **Scene Randomization**: Varying layouts, objects, and initial states.
- **Embodied Generalization**: Learning policies that handle different household contexts.
- **Demonstration and Policy Learning**: Can be used with imitation and RL workflows.

---

## 📊 Comparison Chart

| Benchmark | Domain | Strength | Weakness | Best Use |
|---|---|---|---|---|
| **RoboCasa** | Household manipulation | Realistic kitchen tasks | Domain-specific | Long-horizon manipulation |
| [[RoboSuite]] | Arm manipulation | Clean base tasks | Less realistic scenes | Control research |
| [[LIBERO]] | Lifelong manipulation | Generalization suites | Benchmark-specific | Transfer and forgetting |
| [[ManiSkill]] | General manipulation | Large task variety | Complex benchmark | RL/IL evaluation |
| [[RLBench]] | Vision tasks | Many demos | Different simulator | Vision-language IL |
| [[ProcTHOR]] | Indoor scenes | Procedural environments | Not robot-control focused | Scene generation |

---

## ✅ Pros

- More realistic than many tabletop manipulation benchmarks.
- Useful for long-horizon household tasks.
- Encourages scene and object generalization.
- Connects manipulation to embodied AI settings.
- Good stepping stone toward real home robotics.

---

## ❌ Cons

- Narrower domain than general manipulation suites.
- More environment complexity means harder debugging.
- Sim2Real still requires hardware-specific validation.
- May be too heavy for first manipulation experiments.

---

## 🔗 Related Notes

- [[Manipulation RL]]
- [[Imitation Learning]]
- [[RoboSuite]]
- [[LIBERO]]
- [[ProcTHOR]]

---

## 🌐 External Resources

- RoboCasa Project: https://robocasa.ai/
- RoboCasa GitHub: https://github.com/robocasa/robocasa

---

## 📝 Summary

RoboCasa is a useful benchmark when the goal moves from isolated manipulation skills to realistic household task settings.
