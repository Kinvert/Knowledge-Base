# 🧩 ACT Action Chunking Transformer

**ACT (Action Chunking Transformer)** is an imitation learning architecture that predicts chunks of future robot actions instead of one action at a time. It is widely associated with low-cost bimanual manipulation and modern robot learning pipelines such as [[LeRobot]].

---

## 📚 Overview

ACT reduces compounding error by producing a short sequence of actions from current observations. The robot executes actions from the chunk while the model repeatedly replans. This works well for teleoperated manipulation where demonstrations are smooth and temporally structured.

---

## 🧠 Core Concepts

- **Action Chunk**: Short sequence of future actions predicted at once.
- **Transformer Policy**: Neural network that processes visual and proprioceptive context.
- **Temporal Ensembling**: Blends overlapping action chunks for smoother execution.
- **Behavior Cloning**: Supervised learning from demonstrations.
- **Bimanual Manipulation**: Common use case with two robot arms.
- **Teleoperation Dataset**: Human-collected demonstrations used as training data.

---

## 📊 Comparison Chart

| Method | Predicts | Strength | Weakness | Best For |
|---|---|---|---|---|
| Single-step BC | One action | Simple and fast | Jitter and drift | Easy tasks |
| RNN BC | Sequential actions | Uses history | Harder training | Temporal tasks |
| **ACT** | Action chunks | Smooth, practical imitation | Needs good demos | Teleop manipulation |
| [[Diffusion Policy]] | Denoised action chunks | Multimodal actions | More compute | Contact-rich demos |
| Offline RL | Reward-improved policy | Can improve data | Value errors | Logged transitions |
| Online RL | Trial-and-error policy | Can discover strategies | Expensive on hardware | Simulation |

---

## ✅ Pros

- Practical for low-cost robot imitation learning.
- Smooths action execution through chunking.
- Works well with vision and proprioception.
- Easier to train than many value-based offline RL methods.
- Fits real robot data collection workflows.

---

## ❌ Cons

- Strongly depends on demonstration quality.
- Limited ability to recover from states outside the dataset.
- Action chunk size must match task timing.
- Vision setup and calibration matter.
- Not a reward-maximizing RL method by itself.

---

## 🧰 Robotics Workflow

1. Collect teleoperation demonstrations.
2. Store synchronized images, joint states, and actions.
3. Train ACT with action chunks.
4. Evaluate with temporal ensembling enabled.
5. Add safety limits and workspace checks for deployment.
6. Compare against simple behavior cloning and [[Diffusion Policy]].

---

## 🔗 Related Notes

- [[Imitation Learning]]
- [[LeRobot]]
- [[Robot Data Collection and Teleoperation]]
- [[Robot Dataset Formats]]
- [[Diffusion Policy]]
- [[Manipulation RL]]
- [[DROID]]

---

## 🌐 External Resources

- ACT / ALOHA Paper: https://arxiv.org/abs/2304.13705
- LeRobot GitHub: https://github.com/huggingface/lerobot

---

## 📝 Summary

ACT is a practical imitation-learning architecture for robot manipulation. It is not a general RL algorithm, but it is highly relevant because modern real robot learning often starts with teleoperation and action-chunk behavior cloning.
