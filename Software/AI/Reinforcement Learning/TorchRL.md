# TorchRL

**TorchRL** is a PyTorch-native reinforcement learning (RL) library developed by Meta AI. It provides modular, high-performance components for building, training, and evaluating RL agents. Its deep integration with PyTorch makes it a powerful tool for researchers and practitioners looking to prototype or scale RL algorithms using familiar APIs.

---

## 🔍 Overview

- Built directly on top of PyTorch and [[TorchScript]] for efficient deployment.  
- Offers modular primitives for environments, data collection, replay buffers, policies, losses, transforms, and training loops.  
- Supports both on-policy and off-policy algorithms like [[PPO]], [[DQN]], [[SAC]], and [[TD Learning]].  
- Compatible with [[Gym]], [[DMControl]], [[Isaac Gym]], and [[EnvPool]] environments.  
- Easily integrates with [[TorchRL EnvWrappers]], [[Replay Buffers]], and distributed setups.

---

## 🧠 Core Concepts

- **Transform Pipelines**: Preprocessing pipelines for observations and rewards.  
- **TensorDict**: TorchRL’s core data structure for managing batched data.  
- **Replay Buffers**: Built-in support for prioritized and multi-step buffers.  
- **RLModules**: Modular actor-critic components for building custom policies.  
- **Loss Modules**: Plug-and-play implementation of common loss functions.  
- **Collector Classes**: Tools for managing on-policy/off-policy rollout strategies.  

---

## 🧰 Use Cases

- Prototyping novel RL algorithms with PyTorch-level flexibility.  
- Reproducible baseline training and benchmarking.  
- High-performance distributed training when paired with tools like [[moolib]] or TorchRL’s own launchers.  
- Academic and industrial research requiring low-level control with high-level structure.

---

## ✅ Pros

- Seamlessly integrates with PyTorch ecosystem.  
- Modular and extensible – components can be swapped in/out easily.  
- Accelerated by TorchScript and [[torch.compile]] for deployment or speed.  
- Rich environment support including vectorized and GPU-accelerated sim backends.  

---

## ❌ Cons

- Newer library, so some bugs and limitations in less-used features.  
- Smaller community compared to SB3 or RLlib.  
- May require a deeper PyTorch understanding for advanced customization.  

---

## 📊 Comparison Table: TorchRL vs Other RL Libraries

| Feature              | TorchRL           | [[Stable-Baselines3]]   | [[Ray RLlib]]            | [[moolib]]                |
|----------------------|-------------------|----------------------|-----------------------|------------------------|
| Language             | Python (PyTorch)  | Python (PyTorch)     | Python                | Python + C++           |
| Distribution Support | 🟡 Partial         | ❌ Minimal            | ✅ Strong             | ✅ Strong              |
| Extensibility        | ✅ High            | 🟡 Medium             | 🟡 Medium             | ✅ High               |
| Built-in Algorithms  | ✅ PPO, SAC, DQN   | ✅ PPO, A2C, DDPG     | ✅ Wide Range         | ❌ BYO Algorithms      |
| Envs Supported       | Gym, DMControl, EnvPool, Isaac Gym | Gym  | Gym, PettingZoo      | Gym                    |

---

## 🔧 Compatible Items

- [[PyTorch]] – Core foundation  
- [[EnvPool]], [[Isaac Gym]], [[Gym]] – TorchRL supports them all  
- [[Replay Buffer]] – Natively implemented with multiple variants  
- [[TensorDict]] – TorchRL’s container for batched data  
- [[Policy Gradient Methods]] – Directly supported  
- [[CARBS]] – For hyperparameter tuning of TorchRL agents  
- [[PufferLib]] – Can wrap TorchRL environments and agents  

---

## 🔗 Related Concepts

- [[Reinforcement Learning]] – TorchRL provides core infrastructure  
- [[RL Agent]], [[RL Environment]], [[RL Policy]], [[RL Trajectory]]  
- [[On-Policy]] and [[Off-Policy]] algorithms  
- [[Stable-Baselines3]] – Simpler alternative for smaller projects  
- [[moolib]] – Can be paired for distributed rollout + TorchRL training

---

## 📚 Further Reading

- [TorchRL GitHub Repository](https://github.com/pytorch/rl)  
- [TorchRL Documentation](https://pytorch.org/rl)  
- [TorchRL Blog Launch Post by Meta AI](https://ai.facebook.com/blog/introducing-torchrl-a-library-for-reinforcement-learning-in-pytorch/)  
- [TorchRL Tutorials and Notebooks](https://pytorch.org/rl/tutorials/)  

---
