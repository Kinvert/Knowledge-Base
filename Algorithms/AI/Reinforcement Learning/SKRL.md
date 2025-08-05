# SKRL

SKRL (Scalable Reinforcement Learning) is an open-source library designed to provide modular, scalable, and hardware-accelerated implementations of state-of-the-art Reinforcement Learning (RL) algorithms. Built with flexibility in mind, SKRL supports PyTorch and NVIDIA’s Isaac Gym and Omniverse Isaac Sim, making it particularly well-suited for robotics and simulation environments.

---

## 🧠 Overview

SKRL offers plug-and-play RL algorithms and utilities to accelerate development in research and industrial robotics. It prioritizes ease of use, GPU acceleration, and compatibility with robotics simulators. Developers can easily swap environments, policies, and backends to suit different RL workflows.

---

## 📘 Core Concepts

- **Policy and Value Models:** Modular design supports independent customization and architecture swapping.
- **Environments:** Supports Gym-like APIs and native integration with NVIDIA’s Isaac platforms.
- **Accelerated Training:** Leverages GPU acceleration for high-speed training in simulation.
- **Logging and Evaluation:** Built-in evaluation and logging tools including [[WandB]] and TensorBoard support.

---

## 🛠️ Key Features

- Modular architecture with support for both discrete and continuous action spaces
- Multiple backend compatibility (PyTorch, TorchScript)
- Native integration with Isaac Gym and Isaac Sim
- Ready-to-use implementations of popular algorithms like [[PPO]], [[DDPG]], [[SAC]], and [[TD3]]
- Support for custom environments and models
- On-policy and off-policy learning

---

## 🧪 Use Cases

- Sim-to-real robotics training
- Reinforcement Learning research and benchmarking
- Robotic manipulation and locomotion tasks
- High-throughput training in simulation environments

---

## 📊 Comparison Chart

| Library     | Backend      | Isaac Support | Algorithms        | Robotics Focus | Logging        |
|-------------|--------------|----------------|-------------------|----------------|----------------|
| SKRL        | PyTorch      | ✅              | PPO, SAC, TD3, DDPG | ✅              | WandB, TB       |
| [[Stable-Baselines3]] | PyTorch | ❌              | PPO, A2C, DQN     | ❌              | TensorBoard    |
| [[RLlib]]   | TensorFlow / PyTorch | ❌       | Extensive          | ⚠️ (not robotics-centric) | Custom, WandB |
| [[CleanRL]] | PyTorch      | ❌              | Light set          | ❌              | Lightweight     |
| [[Isaac Gym Envs]] | PyTorch | ✅              | PPO, SAC           | ✅              | N/A             |

---

## ✅ Strengths

- Optimized for fast simulation in Isaac Gym
- Hardware acceleration with minimal setup
- Clear modular structure
- Ideal for robotics-focused research

---

## ⚠️ Weaknesses

- Less mature than larger RL libraries like [[Stable-Baselines3]]
- Documentation still growing
- Smaller user community

---

## 🔗 Related Concepts

- [[PPO]] (Proximal Policy Optimization)
- [[SAC]] (Soft Actor-Critic)
- [[Isaac Gym]] (GPU-accelerated simulator)
- [[Omniverse Isaac Sim]]
- [[Reinforcement Learning]]
- [[WandB]] (Weights & Biases)
- [[Gymnasium]] (OpenAI Gym successor)

---

## 🔧 Compatible Items

- [[PyTorch]]
- [[Isaac Gym]]
- [[Isaac Sim]]
- [[WandB]]
- [[NumPy]]
- [[Gymnasium]]

---

## 🌐 External Resources

- [GitHub: skrl/skrl](https://github.com/Toni-SM/skrl)
- [Documentation](https://skrl.readthedocs.io)
- [Isaac Gym](https://developer.nvidia.com/isaac-gym)
- [Omniverse Isaac Sim](https://developer.nvidia.com/omniverse/isaac-sim)

---

## 📚 Further Reading

- RL papers and algorithm overviews on [Papers with Code](https://paperswithcode.com/)
- NVIDIA Robotics SDKs
- Tutorials from NVIDIA and SKRL GitHub repository

---
