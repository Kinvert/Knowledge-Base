# 🌫️ Diffusion Policy

**Diffusion Policy** is an imitation learning approach that uses diffusion models to generate robot action sequences. It has become important in robot manipulation because it handles multimodal demonstrations and produces smooth visuomotor behavior.

---

## 📚 Overview

Instead of predicting a single next action with mean squared error, Diffusion Policy learns to denoise action trajectories conditioned on observations. This is useful when a task has multiple valid ways to act, such as grasping an object from different angles or choosing different contact strategies.

---

## 🧠 Core Concepts

- **Action Sequence Prediction**: Predicts a short horizon of future actions.
- **Denoising Process**: Learns to convert noise into plausible actions.
- **Receding Horizon Execution**: Executes only the first part of the generated action chunk.
- **Multimodal Behavior**: Can represent multiple valid action choices.
- **Visuomotor Policy**: Conditions on images and robot proprioception.
- **Demonstration Dataset**: Usually trained from teleoperation or scripted expert data.

---

## 📊 Comparison Chart

| Policy Type | Strength | Weakness | Best For | Common Tooling |
|---|---|---|---|---|
| Behavior Cloning MLP | Simple and fast | Averages modes | State-based demos | [[robomimic]] |
| RNN/LSTM policy | Temporal memory | Training complexity | Sequential tasks | PyTorch |
| [[ACT Action Chunking Transformer]] | Smooth chunks | Dataset dependent | Teleop imitation | [[LeRobot]] |
| **Diffusion Policy** | Multimodal actions | More compute | Manipulation demos | PyTorch, real robot datasets |
| Offline RL policy | Can improve rewards | Value errors | Logged RL data | CQL, IQL, AWAC |
| Online RL policy | Can discover behavior | Exploration cost | Sim tasks | [[SAC]], [[PPO]] |

---

## ✅ Pros

- Handles multimodal action distributions better than simple BC.
- Produces smooth action sequences.
- Strong fit for manipulation demonstrations.
- Works with image and proprioceptive observations.
- Good conceptual bridge between generative models and robot control.

---

## ❌ Cons

- More expensive than simple behavior cloning.
- Needs high-quality aligned demonstrations.
- Deployment latency must be managed.
- Does not automatically solve safety or recovery.
- Can overfit to camera setup and dataset conditions.

---

## 🧰 Robotics Workflow

1. Collect consistent teleoperation demonstrations.
2. Align camera frames, proprioception, and actions.
3. Train a behavior cloning baseline.
4. Train Diffusion Policy on action chunks.
5. Evaluate with held-out tasks and real rollouts.
6. Deploy with action smoothing, workspace limits, and watchdogs.

---

## 🔗 Related Notes

- [[Imitation Learning]]
- [[Manipulation RL]]
- [[LeRobot]]
- [[robomimic]]
- [[Robot Data Collection and Teleoperation]]
- [[Robot Dataset Formats]]
- [[Offline RL for Robotics]]

---

## 🌐 External Resources

- Diffusion Policy Paper: https://arxiv.org/abs/2303.04137
- Project Page: https://diffusion-policy.cs.columbia.edu/

---

## 📝 Summary

Diffusion Policy is a modern robot imitation method for generating smooth, multimodal action sequences. It is especially relevant for manipulation tasks where demonstrations contain many valid strategies.
