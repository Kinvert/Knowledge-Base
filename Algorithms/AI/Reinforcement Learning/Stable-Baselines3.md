# Stable-Baselines3

**Stable-Baselines3 (SB3)** is a popular, well-maintained, and user-friendly open-source library for reinforcement learning (RL) in Python. It provides reliable implementations of state-of-the-art RL algorithms with clean APIs, extensive documentation, and compatibility with popular environments like OpenAI Gym. SB3 is widely used in research and industry for benchmarking, prototyping, and deploying RL agents.

---

## 🔍 Overview

- A successor to Stable-Baselines, rewritten for PyTorch (instead of TensorFlow).  
- Implements many classic and modern RL algorithms, including [[PPO]], [[DQN]], [[A2C]], [[SAC]], [[TD3]], and more.  
- Designed for ease of use, modularity, and extensibility.  
- Supports integration with popular RL environments and experiment tracking tools.  
- Focuses on clean, readable code and thorough testing for reliability.  

---

## 🧠 Core Concepts

- `Modular Architecture`: Clear separation between policies, algorithms, environments, and replay buffers.  
- `PyTorch Backend`: Leverages dynamic computation graphs and GPU acceleration.  
- `On-policy and Off-policy Algorithms`: Covers a wide range of methods for different RL needs.  
- `Vectorized Environments`: Efficient training across multiple parallel instances.  
- `Callback System`: Flexible hooks for logging, evaluation, early stopping, and custom behaviors.  
- `Hyperparameter Optimization`: Integration with libraries like Optuna for tuning.  

---

## 🧰 Use Cases

- Benchmarking RL algorithms on standard and custom tasks.  
- Rapid prototyping of RL models for robotics, games, and control systems.  
- Teaching RL concepts with accessible API and example scripts.  
- Deploying RL agents in research projects and industrial applications.  
- Experimenting with new algorithm variants by extending existing implementations.  

---

## ✅ Pros

- Easy to install and get started with (`pip install stable-baselines3`).  
- Supports a broad spectrum of popular RL algorithms.  
- Active community and regularly maintained with new features and bug fixes.  
- Extensive documentation and examples covering many use cases.  
- Compatible with OpenAI Gym, custom environments, and wrappers.  
- Built-in logging support (TensorBoard, WandB, etc.) and evaluation tools.  

---

## ❌ Cons

- Mainly supports PyTorch; users familiar with TensorFlow may face a learning curve.  
- Focused on single-agent RL; limited built-in support for multi-agent setups.  
- Requires some knowledge of RL concepts for effective use.  
- Some algorithms may still require careful hyperparameter tuning.  

---

## 📊 Comparison Table: Stable-Baselines3 vs Other RL Libraries

| Library              | Backend              | Algorithms Included              | Ease of Use | Community & Support      | Notes                                   |
| -------------------- | -------------------- | -------------------------------- | ----------- | ------------------------ | --------------------------------------- |
| Stable-Baselines3    | PyTorch              | PPO, DQN, SAC, TD3, A2C, DDPG... | High        | Active & growing         | Best for PyTorch users                  |
| [[RLlib]]            | TensorFlow & PyTorch | Wide, distributed RL support     | Moderate    | Very active, scalable    | Designed for large-scale/distributed RL |
| [[OpenAI Baselines]] | TensorFlow           | PPO, TRPO, DDPG, A2C, SAC        | Moderate    | Less active (superseded) | Original baseline implementations       |
| [[Tianshou]]         | PyTorch              | Extensive RL algorithms          | Moderate    | Growing                  | Emphasis on modularity and research     |
| [[Dopamine]]         | TensorFlow           | DQN variants                     | Simple      | Google-supported         | Focus on DQN and variants               |

---

## 🤖 In Robotics Context

| Application               | Why Use Stable-Baselines3                        |
|---------------------------|-------------------------------------------------|
| Robot control             | Robust algorithms (PPO, SAC) for continuous control tasks |
| Simulation training       | Efficient parallel environment support          |
| Algorithm benchmarking    | Reliable baseline implementations                |
| Education                 | Clear API suitable for RL teaching                |

---

## 🔧 Compatible Items

- [[Reinforcement Learning]] – Core RL concepts and algorithms  
- [[OpenAI Gym]] – Standard environment interface supported by SB3  
- [[PufferLib]] – Can be used alongside for custom RL environments  
- [[TensorBoard]] – Visualization and monitoring of training  
- [[RL Policy]] – Policies trained using SB3 algorithms  
- [[RL Agent]] – SB3 provides agent implementations  

---

## 🔗 Related Concepts

- [[PPO]] (Proximal Policy Optimization) (One of SB3’s flagship algorithms)  
- [[SAC]] (Soft Actor-Critic) (Popular off-policy continuous control algorithm)  
- [[DQN]] (Deep Q-Network) (Value-based RL algorithm supported by SB3)  
- [[RL Training]] (General RL workflow that SB3 facilitates)  
- [[RL Evaluation]] (Testing and benchmarking trained agents)  

---

## 📚 Further Reading

- [Stable-Baselines3 Documentation](https://stable-baselines3.readthedocs.io/)  
- [GitHub Repository](https://github.com/DLR-RM/stable-baselines3)  
- [Tutorial: Getting Started with Stable-Baselines3](https://stable-baselines3.readthedocs.io/en/master/guide/index.html)  
- [RL Algorithms Implemented in SB3](https://stable-baselines3.readthedocs.io/en/master/guide/algorithms.html)  
- [Comparison of RL Libraries (Medium Article)](https://medium.com/@mujardin_reinforcement/rl-libraries-comparison-71968d1faed8)  

---
