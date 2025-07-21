# Weights & Biases (wandb)

Weights & Biases, commonly abbreviated as `wandb`, is a developer tool designed to help machine learning practitioners streamline their workflows. It offers experiment tracking, hyperparameter tuning, model versioning, and collaboration tools to make ML projects more reproducible and easier to scale. Though not robotics-specific, `wandb` is highly applicable in robotics pipelines that rely on deep learning and reinforcement learning, especially when tuning models in simulation before deploying on real hardware.

---

## 🧠 Overview

`wandb` helps track and visualize machine learning experiments in real-time. It integrates with popular frameworks like PyTorch, TensorFlow, and Keras, and can sync logs, loss curves, hyperparameters, gradients, and more to a centralized web dashboard. This centralized view is especially helpful when training complex models in robotics, such as vision-based navigation or sensor fusion networks.

---

## 🔧 Core Features

- Real-time tracking of loss, metrics, and other training outputs
- Hyperparameter sweeps with grid, random, and Bayesian optimization
- Model checkpointing and artifact versioning
- Integration with Jupyter notebooks and major ML libraries
- Team collaboration tools with project dashboards

---

## 📊 Comparison Chart

| Tool              | Hyperparameter Tuning | Visualization | Artifact Tracking | Collaboration | Robotics Use |
|------------------|------------------------|---------------|-------------------|---------------|--------------|
| wandb            | ✅ Grid, Random, Bayesian | ✅ Live + Logged | ✅ Built-in         | ✅ Teams/Projects | ✅ Widely used |
| TensorBoard      | ❌ Manual logging only | ✅              | ❌                 | ❌             | ✅ Common     |
| MLflow           | ✅ Basic               | ✅              | ✅                 | ✅             | ✅ Some       |
| Comet.ml         | ✅ Advanced            | ✅              | ✅                 | ✅             | ✅ Moderate   |
| Neptune.ai       | ✅ Good                | ✅              | ✅                 | ✅             | ✅ Moderate   |

---

## 🧰 Use Cases

- Training vision models for object detection or segmentation in robotics
- Tuning reinforcement learning algorithms like [[PPO]] (Proximal Policy Optimization) or [[DQN]] (Deep Q Network)
- Tracking performance of different models in [[Sim2Real]] pipelines
- Collaborating on deep learning experiments across research or engineering teams

---

## ✅ Strengths

- Powerful hyperparameter sweeping with minimal setup
- Clean UI for visualizing complex training behavior
- Easy to integrate into existing Python codebases
- Tracks system-level metrics like GPU usage automatically

---

## ❌ Weaknesses

- Requires internet access unless used with the on-prem solution
- Free tier limits access to some advanced features
- May introduce minor performance overhead in tight training loops

---

## 🧭 Related Concepts

- [[Hyperparameter Optimization]] (Tuning strategy for model performance)
- [[PPO]] (Proximal Policy Optimization)
- [[DQN]] (Deep Q Network)
- [[RLlib]] (Reinforcement Learning Library)
- [[Sim2Real]] (Simulation to Reality transfer)
- [[TensorBoard]] (Training visualization toolkit)
- [[MLflow]] (ML lifecycle management tool)

---

## 🧩 Compatible Tools

- PyTorch
- TensorFlow
- Keras
- [[OpenAI Gym]]
- [[Stable Baselines3]]
- [[Hugging Face Transformers]]

---

## 🌐 External Resources

- https://wandb.ai/site
- https://docs.wandb.ai/
- https://github.com/wandb/client

---

## 📚 Documentation and Support

- Official Docs: https://docs.wandb.ai
- GitHub Discussions: https://github.com/wandb/client/discussions
- Community Slack: https://join.slack.com/t/wandb-community

---

## 📦 Developer Tools

- `wandb.init()` — starts a new run
- `wandb.log({"loss": value})` — logs custom metrics
- `wandb.config` — store and retrieve hyperparameters
- `wandb.agent()` — runs sweep agents for tuning

---
