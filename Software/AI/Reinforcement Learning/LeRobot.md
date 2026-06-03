# 🤗 LeRobot

**LeRobot** is Hugging Face's open-source toolkit for real-world robot learning. It provides datasets, training code, pretrained policies, robot interfaces, and tools for imitation learning workflows such as [[ACT Action Chunking Transformer]] and [[Diffusion Policy]].

---

## 📚 Overview

LeRobot is important because it focuses on the practical path from robot data to deployable policies. It is less about classic online RL and more about real robot datasets, teleoperation, behavior cloning, policy training, and reproducible robot learning pipelines.

---

## 🧠 Core Concepts

- **LeRobotDataset**: Dataset format for robot observations, actions, videos, and metadata.
- **Teleoperation**: Human control used to collect demonstrations.
- **Policy Training**: Behavior cloning, ACT, diffusion-style policies, and related methods.
- **Robot Interface**: Code for connecting supported robot arms and cameras.
- **Hugging Face Hub**: Dataset and model sharing.
- **Evaluation Rollouts**: Running trained policies on robot tasks.

---

## 📊 Comparison Chart

| Tool | Best For | Strength | Weakness | Robotics Fit |
|---|---|---|---|---|
| **LeRobot** | Real robot imitation | Datasets, policies, hardware workflows | Young ecosystem | Very high |
| [[robomimic]] | Manipulation benchmarks | Strong IL baselines | More benchmark-oriented | High |
| [[DROID]] | Large real robot data | Diverse demos | Dataset scale complexity | High |
| [[Open X-Embodiment]] | Cross-robot data | Broad embodied dataset | Heterogeneous data | High |
| [[Isaac Lab]] | Sim RL/IL | Scalable simulation | Heavy setup | High |
| [[Stable-Baselines3]] | Classic RL | Simple algorithms | Not robot-data focused | Medium |

---

## ✅ Pros

- Directly relevant to real robot learning.
- Good bridge from data collection to policy training.
- Supports modern imitation-learning policy types.
- Works with public datasets and local robot data.
- Fits low-cost robot arm experimentation.

---

## ❌ Cons

- Less focused on classic online RL.
- Hardware support depends on available drivers and robot setup.
- Dataset quality and synchronization dominate results.
- APIs may evolve quickly.
- Still needs deployment safety outside the model.

---

## 🧰 Practical Workflow

1. Set up robot, cameras, and teleoperation.
2. Record demonstrations into a dataset.
3. Validate timestamps, video frames, actions, and metadata.
4. Train a simple behavior cloning baseline.
5. Train [[ACT Action Chunking Transformer]] or [[Diffusion Policy]].
6. Deploy with workspace limits and logging.

---

## 🔗 Related Notes

- [[Imitation Learning]]
- [[ACT Action Chunking Transformer]]
- [[Diffusion Policy]]
- [[Robot Data Collection and Teleoperation]]
- [[Robot Dataset Formats]]
- [[SO-ARM100]]
- [[DROID]]

---

## 🌐 External Resources

- LeRobot GitHub: https://github.com/huggingface/lerobot
- Hugging Face LeRobot Docs: https://huggingface.co/docs/lerobot

---

## 📝 Summary

LeRobot is one of the most relevant tools for learning real-world robot imitation. It is a practical complement to simulation-focused RL stacks such as [[PufferLib]], [[MuJoCo]], and [[Isaac Lab]].
