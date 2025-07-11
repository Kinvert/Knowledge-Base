# Reinforcement Learning

**Reinforcement Learning (RL)** is a machine learning paradigm where agents learn to make decisions by interacting with an environment to maximize cumulative rewards. It differs from supervised and unsupervised learning in its focus on decision-making over time, where learning is guided by trial-and-error interactions rather than labeled datasets.

This document serves as the top-level reference for RL concepts, algorithms, libraries, and tools relevant to robotics, simulation, and engineering applications.

---

## 🧠 Core Concepts

- **Agent**: The learner/decision-maker  
- **Environment**: The system with which the agent interacts  
- **State**: The current situation of the agent  
- **Action**: The choices available to the agent  
- **Reward**: Feedback received after performing an action  
- **Policy (π)**: A strategy used by the agent to determine actions  
- **Value Function (V or Q)**: Estimates future rewards from states or state-action pairs  
- **Exploration vs Exploitation**: Balancing between trying new actions and leveraging known ones  

---

## ⚙️ RL Methods

| Type                    | Description                                                                 |
|-------------------------|-----------------------------------------------------------------------------|
| **Model-Free**          | Learns directly from interaction without modeling the environment          |
| **Model-Based**         | Builds a model of the environment for planning                             |
| **Value-Based**         | Focuses on learning value functions (e.g., Q-learning)                     |
| **Policy-Based**        | Directly learns the policy (e.g., REINFORCE, PPO)                          |
| **Actor-Critic**        | Combines value-based and policy-based methods                              |
| **Offline RL**          | Learns from fixed datasets instead of active exploration                   |
| **Multi-Agent RL**      | Involves multiple agents interacting in a shared environment               |
| **Hierarchical RL**     | Uses macro-actions or sub-policies to manage long-term behavior            |

---

## 🤖 In a Robotics Context

Reinforcement Learning is widely applied in:

- **Locomotion** – Teaching robots to walk, crawl, or fly  
- **Grasping and Manipulation** – Training robotic arms for pick-and-place tasks  
- **Navigation and Planning** – Learning how to reach goals in unknown environments  
- **Control Systems** – Learning optimal PID tuning or adaptive controllers  
- **Sim-to-Real Transfer** – Learning in simulation and deploying in real-world systems  

---

## 🧰 Tools & Libraries

### 🧪 Libraries and Frameworks

- [[PufferLib]] – Modular and backend-agnostic RL abstraction layer  
- [[PettingZoo]] – Multi-agent environments with standard interfaces  
- [[Gymnasium]] – Standard API for single-agent environments  
- [[Stable-Baselines3]] – Popular PyTorch-based implementation of common RL algorithms  
- [[RLlib]] – Distributed RL built on top of Ray  
- [[CleanRL]] – Minimal, readable RL implementations  
- [[Keras-RL]] – RL algorithms using TensorFlow/Keras  
- [[TorchRL]] – PyTorch-native RL toolkit from Meta  

### 🧠 Backends

- [[PyTorch]] – Widely used DL framework for implementing RL agents  
- [[JAX]] – Functional backend with just-in-time (JIT) compilation  
- [[TensorFlow]] – Often used with Keras-based RL workflows  

### 🧰 Other Tools

- [[Weights & Biases]] – Logging and experiment tracking  
- [[Docker]] – Containerize experiments for reproducibility  
- [[Conda]] / [[pip]] – Manage RL dependencies  
- [[rclpy]] – Can be used to interface with ROS 2 simulation environments  

---

## 📊 Comparison with Other ML Types

| Aspect                  | Supervised Learning     | Unsupervised Learning   | Reinforcement Learning   |
|--------------------------|------------------------|--------------------------|---------------------------|
| Input                   | Labeled data            | Unlabeled data           | Observations (state, reward) |
| Output                  | Direct predictions      | Pattern discovery        | Policy (decision rule)     |
| Feedback Type           | Ground truth labels     | Clustering, distributions| Delayed, sparse rewards    |
| Learning Style          | One-shot learning       | Statistical modeling     | Trial-and-error            |
| Applications            | Classification, regression | Dimensionality reduction | Control, planning, robotics |
| Temporal Dependency     | No                     | No                      | Yes                        |

---

## 📈 Common Algorithms

| Algorithm     | Type         | Notes                                               |
|---------------|--------------|-----------------------------------------------------|
| Q-Learning    | Value-Based  | Off-policy, discrete action spaces                 |
| DQN           | Value-Based  | Deep Q-Learning, uses neural networks              |
| SARSA         | Value-Based  | On-policy alternative to Q-Learning                |
| REINFORCE     | Policy-Based | Monte Carlo policy gradient                        |
| A2C / A3C     | Actor-Critic | Synchronous/asynchronous versions of actor-critic  |
| PPO           | Policy-Based | Stable on-policy method with clipping              |
| DDPG          | Actor-Critic | For continuous actions, deterministic policy       |
| TD3           | Actor-Critic | Twin-delayed version of DDPG                       |
| SAC           | Actor-Critic | Maximum entropy reinforcement learning             |

---

## 🧪 Benchmark Environments

- **Gymnasium** – [[CartPole]], [[LunarLander]], Pendulum  
- **PettingZoo** – Multi-agent RL tasks  
- **[[MuJoCo]]** – Simulated physics environments for robotics  
- **Isaac Gym** – GPU-accelerated robotics simulation by NVIDIA  
- **Unity ML-Agents** – Game engine-based simulations  
- **KSP (via KRPC)** – Kerbal Space Program automation via RPC  

---

## 🛰️ Fun and Exploratory Applications

- **Kerbal Space Program + KRPC**: Train agents to launch and dock rockets  
- **Simulated drone control** using AirSim  
- **RL with physical switches and dashboards** via ROS2 + Web UIs  
- **Teaching household tasks** in a simulated kitchen environment  
- **Autonomous racing bots** in Gym-compatible environments  

---

## ✅ Pros

- Handles sequential decision making  
- Requires less labeled data than supervised learning  
- Enables learning in dynamic and uncertain environments  
- Can outperform hard-coded strategies  
- Well-suited for robotics and control problems  

---

## ❌ Cons

- Sample inefficient (can require millions of interactions)  
- Sparse or delayed rewards are hard to learn from  
- Hyperparameter tuning is often critical  
- Sim-to-real gap can be large  
- Training can be unstable or non-deterministic  

---

## 🔧 Compatible Items

- [[PufferLib]] – Flexible RL training interface  
- [[PettingZoo]] – Multi-agent simulation environments  
- [[Gymnasium]] – Single-agent RL standard  
- [[PyTorch]] – Most common deep learning backend  
- [[JAX]] – Backend for functional programming and fast execution  
- [[Docker]] – Containerized experiments  

---

## 🔗 Related Concepts

- [[PufferLib]] (RL abstraction library)  
- [[PettingZoo]] (Multi-agent environments)  
- [[Gymnasium]] (Classic control environments)  
- [[JAX]] (Functional backend for RL research)  
- [[PyTorch]] (Neural network library)  
- [[Multi-Agent RL]] (Collaboration and competition in RL)  

---

## 📚 Further Reading

- [Spinning Up in Deep RL (OpenAI)](https://spinningup.openai.com/en/latest/)  
- [Deep RL Bootcamp](https://sites.google.com/view/deep-rl-bootcamp/lectures)  
- [RL Course by David Silver (DeepMind)](https://www.davidsilver.uk/teaching/)  
- [Awesome RL GitHub List](https://github.com/aikorea/awesome-rl)  
- [PettingZoo Paper](https://arxiv.org/abs/2009.14471)  
- [PufferLib Blog Intro](https://jxhe.github.io/pufferlib/posts/pufferlib-intro/)  

---
