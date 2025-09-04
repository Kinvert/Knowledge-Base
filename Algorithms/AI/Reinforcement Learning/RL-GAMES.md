# RL-Games
RL-Games is a PyTorch-based reinforcement learning (RL) library that is lightweight, flexible, and highly optimized for training in simulation. It is widely used in robotics research (especially with NVIDIA Isaac Gym and Isaac Lab) and supports multiple continuous-control algorithms with strong GPU acceleration.

---

## 🧭 Overview
RL-Games was originally developed by Denis Tarasov at NVIDIA to provide an efficient RL training library for large-scale parallel simulation environments. Unlike more general-purpose RL libraries, RL-Games is tightly focused on performance in robotics, control, and continuous-action environments.

GitHub: https://github.com/Denys88/rl_games  
Docs: https://denys88.github.io/rl_games/

---

## 🧩 Core Concepts
- **Simulation-Oriented**: Designed for large-batch training in GPU-accelerated simulators like Isaac Gym and Isaac Lab.
- **Algorithm Coverage**: Implements PPO, SAC, DDPG, A2C, and others, with optimized variants for robotics.
- **Policy Flexibility**: Supports feedforward MLPs, recurrent policies (LSTMs/GRUs), and custom architectures.
- **Config Driven**: YAML/JSON-based experiment setup; easily reproducible.
- **Domain Randomization**: Natively supports sim2real robustness training.
- **Multi-GPU Support**: Designed to scale across large parallel environments.

---

## 🔎 Comparison Chart
| Framework | Algorithms | Robotics Focus | GPU Acceleration | Isaac Gym/Lab Integration | Docs/Tutorials | License |
|---|---|---|---|---|---|---|
| **RL-Games** | PPO, SAC, DDPG, A2C, etc. | Yes (core design) | Yes (multi-GPU) | Native (NVIDIA collaboration) | Medium (improving) | MIT |
| **RSL-RL** | PPO (optimized) | Yes | Yes | Tight (ETH/NVIDIA) | Medium | BSD-3 |
| **Stable Baselines3** | PPO, SAC, TD3, A2C, DQN | General RL | Limited | Indirect | Strong | MIT |
| **RLlib (Ray)** | Wide (PPO, IMPALA, SAC, etc.) | General | Distributed CPU/GPU | Indirect | Strong | Apache 2.0 |
| **CleanRL** | Minimal PPO, SAC, A2C | General | Partial | Indirect | Strong (educational) | MIT |

---

## ⚙️ Use Cases
- Training locomotion controllers for quadrupeds and bipeds
- Dexterous manipulation tasks with multi-fingered hands
- Aerial vehicle control (drones, multirotors)
- Large-scale policy training in Isaac Gym/Isaac Lab
- Research prototyping for new RL algorithm development

---

## ✅ Strengths
- Efficient, scalable, GPU-native
- Multiple algorithms implemented beyond PPO
- Tight NVIDIA integration for Isaac Gym/Isaac Lab
- Clear configuration system for reproducibility
- Widely used in research and industry robotics projects

---

## ⚠️ Weaknesses
- Documentation less extensive than SB3 or RLlib
- Smaller community outside of NVIDIA robotics ecosystem
- Fewer pretrained policies openly available compared to SB3
- Focused on simulation; less turn-key for discrete RL tasks

---

## 🛠️ Developer Tools
- **PyTorch** backend (high performance)
- **Hydra/YAML configs** for experiments
- **TensorBoard & Weights & Biases (`wandb`)** logging
- **Custom env API** for integration with Isaac and Gym

---

## 📚 Documentation and Tutorials
- Official docs: https://denys88.github.io/rl_games/
- GitHub repo with examples: https://github.com/Denys88/rl_games/tree/master/examples
- Isaac Gym example environments (humanoids, quadrupeds): https://github.com/NVIDIA-Omniverse/IsaacGymEnvs
- Isaac Lab integration (RL-Games backend option): https://isaac-sim.github.io/IsaacLab/source/overview/rl_games.html
- NVIDIA tutorial blog: https://developer.nvidia.com/blog/scaling-robotics-reinforcement-learning-with-isaac-gym/

---

## 🔗 Related Notes
- [[Reinforcement Learning]]
- [[PPO]] (Proximal Policy Optimization)
- [[SAC]] (Soft Actor-Critic)
- [[RSL-RL]]
- [[Isaac Gym]]
- [[Isaac Lab]]
- [[Sim2Real]]
- [[PyTorch]]

---
