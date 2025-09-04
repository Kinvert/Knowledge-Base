# RSL-RL (Robotics and Simulation Learning - Reinforcement Learning)
RSL-RL is a lightweight and high-performance reinforcement learning (RL) framework designed specifically for robotics and simulation environments. It was developed by the Robotic Systems Lab (ETH Zürich) to support continuous control tasks, including locomotion, manipulation, and aerial robotics. RSL-RL is optimized for training with NVIDIA Isaac Gym and Isaac Lab but is flexible enough to be used with other simulators.

---

## 🧭 Overview
RSL-RL provides a modular and GPU-accelerated implementation of common RL algorithms, making it highly efficient for training robotic control policies. It is frequently used in academic and industrial robotics projects, especially where simulation-to-reality transfer (sim2real) is required.

GitHub: https://github.com/leggedrobotics/rsl_rl

Docs/Examples: https://leggedrobotics.github.io/rsl_rl/

---

## 🧩 Core Concepts
- **Continuous Control Focus**: Designed for high-dimensional action spaces typical in robotics.
- **Algorithms**: Implements Proximal Policy Optimization (PPO) and variants.
- **GPU Utilization**: Built for parallelized simulation on GPUs (especially with Isaac Gym/Isaac Lab).
- **Modularity**: Easy to extend with custom environments, policies, and reward functions.
- **Torch-based**: Written in PyTorch for flexibility and speed.
- **Sim2Real Orientation**: Emphasis on policies that can be transferred to physical robots.

---

## 🔎 Comparison Chart
| Framework | Main Algorithms | Robotics Focus | GPU Acceleration | Isaac Integration | Sim2Real Proven? | License |
|---|---|---|---|---|---|---|
| **RSL-RL** | PPO (optimized for robotics) | Yes | Yes (Isaac Gym/Isaac Lab) | Tight (ETH & NVIDIA collab) | Yes (quadrupeds, manipulators) | BSD-3 |
| **Stable Baselines3** | PPO, SAC, TD3, etc. | General RL | Limited | Indirect | Some | MIT |
| **RLlib (Ray)** | PPO, IMPALA, SAC, etc. | General RL | Distributed CPU/GPU | Indirect | Limited | Apache 2.0 |
| **CleanRL** | PPO, SAC, A2C (minimalistic) | General RL | Partial | Indirect | No | MIT |
| **ElegantRL** | PPO, SAC, TD3, DDPG | General RL | Yes (PyTorch/NumPy) | Indirect | Some demos | MIT |
| **Isaac Lab Built-ins** | PPO (via RSL-RL), SAC | Robotics | Yes | Native | Yes | NVIDIA proprietary + open parts |

---

## ⚙️ Use Cases
- Training locomotion policies for quadrupeds (ANYmal, Unitree, Spot)
- Manipulation tasks with arms (Franka Emika, Kinova)
- Aerial robotics (drones, tiltrotors)
- Domain randomization for robust sim2real transfer
- Benchmarking new RL algorithms in robotics settings

---

## ✅ Strengths
- Purpose-built for robotics and simulation
- Highly optimized PPO implementation
- Easy integration with Isaac Gym and Isaac Lab
- Actively used in ETH Zürich and NVIDIA research
- Strong track record of real-world robot deployments

---

## ⚠️ Weaknesses
- Supports fewer algorithms compared to general RL libraries
- Documentation less extensive than mainstream RL libs (though improving)
- Community smaller than SB3 or RLlib
- Primarily tied to PPO (less algorithmic diversity)

---

## 🛠️ Developer Tools
- **PyTorch** backend for policies and training
- **Hydra** for configuration management
- **Isaac Gym/Isaac Lab environments** for scalable training
- Logging with TensorBoard and Weights & Biases (`wandb`)

---

## 📚 Documentation and Tutorials
- Official docs: https://leggedrobotics.github.io/rsl_rl/
- GitHub examples: https://github.com/leggedrobotics/rsl_rl/tree/master/examples
- Isaac Lab integration guide: https://isaac-sim.github.io/IsaacLab/source/overview/rsl_rl.html
- ETH Zürich ANYmal locomotion research (using RSL-RL): https://leggedrobotics.github.io

---

## 🔗 Related Notes
- [[Reinforcement Learning]]
- [[PPO]] (Proximal Policy Optimization)
- [[Isaac Gym]]
- [[Isaac Lab]]
- [[Sim2Real]]
- [[PyTorch]]

---
