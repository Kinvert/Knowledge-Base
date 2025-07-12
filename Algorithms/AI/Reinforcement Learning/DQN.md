# Deep Q-Network (DQN)

**Deep Q-Network (DQN)** is a foundational reinforcement learning algorithm that combines Q-learning with deep neural networks to handle high-dimensional state spaces. Introduced by DeepMind in 2015, DQN made it possible to learn control policies directly from raw sensory inputs like images, enabling breakthroughs in game playing and robotics.

---

## 🔍 Overview

- Extends traditional Q-learning by approximating the Q-value function with a deep neural network.  
- Learns a policy by estimating action-value function Q(s, a), representing expected future rewards.  
- Uses **experience replay** to break correlations in sequential data and stabilize training.  
- Employs a separate **target network** to stabilize Q-value updates.  
- Initially applied to Atari games from raw pixels, demonstrating human-level control.  

---

## 🧠 Core Concepts

- `Q-Value Function (Q(s, a))`: Expected return after taking action a in state s and following policy thereafter.  
- `Experience Replay`: Buffer storing past transitions sampled randomly during training to reduce data correlation.  
- `Target Network`: A periodically updated copy of the main network used to compute stable target Q-values.  
- `Bellman Equation`: Used as a learning target to iteratively update Q estimates.  
- `ε-greedy Policy`: Balances exploration and exploitation by choosing random actions with probability ε.  

---

## 🧰 Use Cases

- Discrete action spaces where states are high-dimensional (e.g., images).  
- Game AI development (e.g., Atari, board games).  
- Robotics tasks with discrete control or discretized continuous actions.  
- Benchmarking value-based RL methods.  
- Foundation for many improved RL algorithms like Double DQN and Dueling DQN.  

---

## ✅ Pros

- Can handle raw high-dimensional inputs using convolutional neural networks.  
- Effective for discrete action spaces.  
- Experience replay and target networks stabilize learning significantly.  
- Strong empirical results on challenging benchmarks.  
- Relatively simple to implement and extend.  

---

## ❌ Cons

- Limited to discrete action spaces; not suitable for continuous control.  
- Requires significant tuning of hyperparameters.  
- Sample inefficient compared to some modern off-policy methods.  
- Training can be unstable or diverge without careful design.  
- Experience replay buffers increase memory requirements.  

---

## 📊 Comparison Table: DQN and Related Value-Based Algorithms

| Algorithm              | Key Feature                          | [[Action Space]] | Stability                  | Use Case                              |
| ---------------------- | ------------------------------------ | ---------------- | -------------------------- | ------------------------------------- |
| DQN                    | Experience replay + target net       | Discrete         | Moderate                   | Games, robotics with discrete actions |
| [[Double DQN]]         | Reduces overestimation bias          | Discrete         | Improved                   | More stable learning                  |
| [[Dueling DQN]]        | Separate value and advantage streams | Discrete         | Improved                   | Better value estimation               |
| [[Prioritized Replay]] | Samples important experiences        | Discrete         | Improved sample efficiency | Faster convergence                    |
| [[Rainbow DQN]]        | Combines multiple improvements       | Discrete         | State-of-the-art           | Benchmarking                          |

---

## 🤖 In Robotics Context

| Scenario                  | Why Use DQN                              |
|---------------------------|-----------------------------------------|
| Discrete robotic tasks    | When control commands are discrete      |
| Vision-based navigation   | Processing raw images for decision-making |
| Simple manipulation tasks | Where discretized actions suffice       |

---

## 🔧 Compatible Items

- [[Reinforcement Learning]] – Core RL method for discrete action spaces  
- [[Experience Replay]] – Essential mechanism to stabilize DQN training  
- [[Target Network]] – Stabilizes learning targets  
- [[ε-greedy Policy]] – Exploration strategy used by DQN  
- [[Deep Q-Learning Variants]] – Extensions like Double DQN and Dueling DQN  

---

## 🔗 Related Concepts

- [[Q-Learning]] (Classical RL algorithm foundational to DQN)  
- [[Policy Gradient]] (Alternative RL approach using policy parameterization)  
- [[Replay Buffer]] (Memory to store and sample experiences)  
- [[Function Approximation]] (Use of neural networks to approximate Q-values)  
- [[Exploration vs Exploitation]] (ε-greedy method used for balance)  

---

## 📚 Further Reading

- [Playing Atari with Deep Reinforcement Learning (Mnih et al., 2013)](https://arxiv.org/abs/1312.5602)  
- [Human-level Control through Deep Reinforcement Learning (Nature, 2015)](https://www.nature.com/articles/nature14236)  
- [Deep Q-Learning Tutorial (Lil’Log)](https://lilianweng.github.io/lil-log/2018/04/08/policy-gradient-algorithms.html#deep-q-network-dqn)  
- [OpenAI Spinning Up – DQN](https://spinningup.openai.com/en/latest/algorithms/dqn.html)  
- [Double DQN](https://arxiv.org/abs/1509.06461)  
- [Dueling DQN](https://arxiv.org/abs/1511.06581)  

---
