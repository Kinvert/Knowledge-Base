# 🧪 robomimic

**robomimic** is an open-source framework for robot imitation learning and offline learning experiments, especially for manipulation tasks. It provides datasets, training pipelines, policy architectures, and reproducible baselines.

---

## 📚 Overview

robomimic is useful when the goal is to understand behavior cloning, sequence models, offline datasets, and manipulation benchmarks. It is commonly paired with datasets from [[RoboSuite]] and other robot manipulation sources.

---

## 🧠 Core Concepts

- **Demonstration Dataset**: Expert trajectories with observations and actions.
- **Behavior Cloning**: Supervised imitation of demonstration actions.
- **Sequence Policy**: Model that uses observation history.
- **HDF5 Dataset**: Common storage format for trajectories.
- **Observation Modality**: Low-dimensional state, RGB images, depth, or proprioception.
- **Benchmark Configs**: Reproducible training and evaluation settings.

---

## 📊 Comparison Chart

| Tool | Best For | Strength | Weakness | Main Focus |
|---|---|---|---|---|
| **robomimic** | IL/offline benchmarks | Reproducible manipulation baselines | Less hardware-focused | Manipulation datasets |
| [[LeRobot]] | Real robot workflows | Teleop to deployment | Younger ecosystem | Practical robot learning |
| [[RoboSuite]] | Sim manipulation | Clean MuJoCo tasks | Simulation-focused | Benchmark environments |
| [[ManiSkill]] | Manipulation RL/IL | Many tasks and assets | Benchmark complexity | Sim manipulation |
| [[DROID]] | Real robot demos | Large diverse dataset | Data scale | Real manipulation |
| [[Open X-Embodiment]] | Cross-robot learning | Huge multi-robot data | Heterogeneity | Generalist policies |

---

## ✅ Pros

- Strong learning resource for imitation and offline robot learning.
- Good benchmark structure and configs.
- Supports multiple observation modalities.
- Pairs well with [[RoboSuite]].
- Useful before moving to real robot pipelines.

---

## ❌ Cons

- Less focused on live robot data collection.
- HDF5 dataset handling requires care.
- Sim benchmark success may not transfer directly.
- Not a full robotics middleware or deployment stack.
- Real-world safety must be handled elsewhere.

---

## 🧰 Use Cases

- Train behavior cloning policies on manipulation datasets.
- Compare sequence policies and observation modalities.
- Learn dataset structure for robot trajectories.
- Prototype offline learning before using real robot data.
- Reproduce published manipulation baselines.

---

## 🔗 Related Notes

- [[Imitation Learning]]
- [[Offline RL for Robotics]]
- [[Manipulation RL]]
- [[Robot Dataset Formats]]
- [[RoboSuite]]
- [[HDF5]]
- [[DROID]]

---

## 🌐 External Resources

- robomimic Docs: https://robomimic.github.io/docs/
- robomimic GitHub: https://github.com/ARISE-Initiative/robomimic

---

## 📝 Summary

robomimic is a strong benchmark and learning framework for robot imitation and offline manipulation. It is especially useful for understanding dataset-driven robot learning before moving into hardware-heavy workflows.
