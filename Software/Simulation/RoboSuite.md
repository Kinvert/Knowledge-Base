# 🧰 RoboSuite

**RoboSuite** is a MuJoCo-based simulation framework for robot manipulation research. It provides robot arms, grippers, objects, controllers, and task environments for reinforcement learning and imitation learning.

---

## 📚 Overview

RoboSuite is useful when the goal is clean, reproducible manipulation experiments using [[MuJoCo]]. It is closely related to [[robomimic]], which uses RoboSuite-style datasets and tasks for imitation and offline learning.

---

## 🧠 Core Concepts

- **Robot Models**: Common arms such as Panda, Sawyer, IIWA, and UR5e.
- **Controllers**: Operational space, joint position, velocity, and torque control.
- **Task Suite**: Lift, stack, door, wipe, nut assembly, and related manipulation tasks.
- **Observation Modalities**: Low-dimensional state and camera observations.
- **Dataset Integration**: Often used with imitation-learning datasets and [[robomimic]].

---

## 📊 Comparison Chart

| Tool | Simulator | Best For | Strength | Weakness |
|---|---|---|---|---|
| **RoboSuite** | [[MuJoCo]] | Arm manipulation | Clean control/task API | Simulation-only |
| [[robomimic]] | Dataset/training | Imitation learning | Reproducible baselines | Not a simulator |
| [[ManiSkill]] | SAPIEN/PhysX | Broad manipulation | Many assets/tasks | Heavier benchmark |
| [[Meta-World]] | MuJoCo | Multi-task RL | Simple task family | Less realistic |
| [[RLBench]] | CoppeliaSim | Vision-language tasks | Rich demonstrations | Different stack |
| [[Isaac Lab]] | Isaac Sim | GPU robot learning | Scalable and sensor-rich | Heavy setup |

---

## ✅ Pros

- Clean MuJoCo manipulation framework.
- Good controller options for arms.
- Pairs well with [[robomimic]].
- Easier to inspect than many larger benchmark suites.
- Useful for learning manipulation task design.

---

## ❌ Cons

- Not focused on real robot deployment.
- Less scalable than GPU-native Isaac workflows.
- Sim2Real still requires separate modeling and safety work.
- Task set is narrower than some newer benchmarks.

---

## 🔗 Related Notes

- [[MuJoCo]]
- [[MuJoCo MJCF]]
- [[robomimic]]
- [[Manipulation RL]]
- [[Operational Space Control]]

---

## 🌐 External Resources

- RoboSuite Website: https://robosuite.ai/
- RoboSuite GitHub: https://github.com/ARISE-Initiative/robosuite

---

## 📝 Summary

RoboSuite is one of the cleanest MuJoCo-based entry points for robot arm manipulation research, especially when paired with [[robomimic]] for imitation learning.
