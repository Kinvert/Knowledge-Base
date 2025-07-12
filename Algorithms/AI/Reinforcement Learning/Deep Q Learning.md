# Deep Q-Learning

**Deep Q-Learning** refers to the application of deep neural networks to approximate the Q-value function in reinforcement learning. It is the foundation of algorithms like [[DQN]] (Deep Q-Network), which made it possible to apply RL to complex environments with high-dimensional input spaces, such as video games and vision-based robotics.

---

## 🔍 Overview

- Traditional Q-learning uses a table to store Q-values for each `(state, action)` pair.  
- Deep Q-Learning replaces this table with a neural network that estimates `Q(s, a)` for all actions given a state `s`.  
- Enables the use of RL in environments with continuous, high-dimensional, or image-based observations.  
- Combines techniques like **experience replay** and **target networks** to stabilize learning.  

---

## 🧠 Core Concepts

- **Q-Network**: A neural network that predicts the expected future rewards for all possible actions in a given state.  
- **TD Target**:  
  `y = r + γ * max(Q_target(s′, a′))`  
  Used to train the Q-network to minimize the loss between prediction and this target.  
- **Loss Function**:  
  Mean Squared Error between `Q(s, a)` and the TD target.  
- **Experience Replay**: Random sampling from a memory buffer of past experiences `(s, a, r, s′)` to break correlations.  
- **Target Network**: A separate Q-network used to generate the TD target, updated less frequently than the main network.  

---

## 🧰 Use Cases

- Vision-based control tasks (e.g., learning from pixels).  
- Games like Atari, Doom, or gridworlds.  
- Simple robotic tasks where actions are discrete.  
- Educational examples and RL prototyping in simulation environments.  

---

## ✅ Pros

- Allows RL agents to handle high-dimensional, unstructured input (e.g., images).  
- Scalable to large state spaces.  
- Enables generalization across similar states.  
- Well-studied and forms the foundation of many advanced RL methods.  

---

## ❌ Cons

- Limited to **discrete action spaces** unless modified (e.g., with discretization or actor-critic methods).  
- Training is unstable without stabilization techniques.  
- Neural network approximations introduce additional hyperparameters.  
- Poor sample efficiency compared to some newer methods.  

---

## 📊 Comparison Table: Deep Q-Learning vs Related Methods

| Method                | Action Space       | Function Approximation | Stability Features             | Notes                         |
|-----------------------|--------------------|-------------------------|-------------------------------|------------------------------|
| Q-learning (tabular)  | Discrete           | No                      | None                          | Simple, low memory footprint |
| Deep Q-Learning       | Discrete           | Neural Networks         | Replay buffer, target net     | Scales to complex inputs     |
| Double DQN            | Discrete           | Neural Networks         | Reduces Q-value overestimation| Improves stability           |
| Dueling DQN           | Discrete           | Neural Networks         | State-value + advantage split | Better value estimation      |
| DDPG / SAC            | Continuous         | Neural Networks         | Actor-Critic, target nets     | For continuous actions       |

---

## 🔧 Compatible Items

- [[DQN]] – Canonical implementation of Deep Q-Learning  
- [[Replay Buffer]] – Stores past experiences for sampling  
- [[Target Network]] – Helps reduce instability  
- [[Q-Learning]] – The base algorithm behind Deep Q-Learning  
- [[TD Learning]] – Drives the update rules for Q-values  

---

## 🔗 Related Concepts

- [[Policy Gradient]] – Alternative approach, especially for continuous actions  
- [[Function Approximation]] – Deep networks as approximators  
- [[Exploration vs Exploitation]] – Typically uses `ε-greedy` policies  
- [[Q-Value Function]] – Core concept estimated by deep Q-networks  

---

## 📚 Further Reading

- [Playing Atari with Deep Reinforcement Learning (2013)](https://arxiv.org/abs/1312.5602)  
- [Human-level Control through Deep Reinforcement Learning (2015)](https://www.nature.com/articles/nature14236)  
- [Spinning Up: Q-learning](https://spinningup.openai.com/en/latest/algorithms/dqn.html)  
- [CS50 AI – Deep Q-Learning Lecture](https://cs50.harvard.edu/ai/2020/weeks/7/)  
- [Deep RL Bootcamp (UC Berkeley)](https://sites.google.com/view/deep-rl-bootcamp/)  

---
