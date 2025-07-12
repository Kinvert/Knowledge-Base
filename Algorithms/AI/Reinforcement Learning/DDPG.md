# DDPG

**Deep Deterministic Policy Gradient (DDPG)** is an off-policy actor-critic reinforcement learning algorithm designed for environments with continuous action spaces. It combines deterministic policy gradients with deep neural networks to learn high-dimensional control policies efficiently.

---

## 🔍 Overview

- Uses an **actor network** to output deterministic continuous actions.  
- Employs a **critic network** to estimate Q-values for state-action pairs.  
- Off-policy algorithm utilizing a **replay buffer** for experience reuse.  
- Introduces **target networks** for both actor and critic to stabilize training.  
- Effective for continuous control tasks like robotics, games, and simulations.

---

## 🧠 Core Concepts

- **Deterministic Policy Gradient**: Unlike stochastic policies, outputs a specific action for each state, which reduces variance in updates.  
- **Actor-Critic Architecture**: Separate networks for policy (actor) and Q-value estimation (critic).  
- **Replay Buffer**: Stores transitions for off-policy learning.  
- **Target Networks**: Slow-moving copies of actor and critic used for stable target estimation.  
- **Exploration Noise**: Adds noise (e.g., Ornstein-Uhlenbeck) to actor’s output to encourage exploration during training.

---

## 🧰 Use Cases

- Control of robotic arms, drones, and other continuous control systems.  
- Simulated physics environments like MuJoCo or robotics simulators.  
- Tasks where sample efficiency and stable learning are important.  
- Baseline algorithm for continuous action problems in RL research.

---

## ✅ Pros

- Handles high-dimensional, continuous action spaces effectively.  
- Sample efficient due to off-policy learning and experience replay.  
- Relatively simple architecture compared to more complex algorithms.  
- Widely studied and serves as a foundation for newer methods like [[TD3]].

---

## ❌ Cons

- Can suffer from overestimation bias leading to suboptimal policies.  
- Requires careful tuning of exploration noise and hyperparameters.  
- Deterministic policies can struggle in highly stochastic environments.  
- Less stable than newer algorithms like [[SAC]] and [[TD3]].

---

## 📊 Comparison Table: DDPG vs Other Continuous Control Algorithms

| Algorithm          | On/Off-Policy | Policy Type         | Exploration          | Stability      | Sample Efficiency |
|--------------------|---------------|---------------------|---------------------|----------------|-------------------|
| DDPG               | Off-policy    | Deterministic       | Added noise (OU)    | Moderate       | Moderate          |
| TD3                | Off-policy    | Deterministic       | Delayed policy updates | High         | High              |
| SAC                | Off-policy    | Stochastic          | Entropy maximization | High           | High              |
| PPO                | On-policy     | Stochastic          | Clipped objective   | High           | Moderate          |

---

## 🔧 Compatible Items

- [[Replay Buffer]] – Essential for off-policy training  
- [[Actor Critic]] – DDPG uses this architecture  
- [[Exploration Noise]] – Often Ornstein-Uhlenbeck noise for temporally correlated exploration  
- [[Continuous Action Space]] – Target domains for DDPG  
- [[Target Networks]] – For stable Q-value and policy updates  

---

## 🔗 Related Concepts

- [[Deterministic Policy Gradient]] – The theoretical foundation of DDPG  
- [[Replay Buffer Sampling]] – Strategy for minibatch updates  
- [[TD Learning]] – Critic trained using temporal difference updates  
- [[Ornstein-Uhlenbeck Process]] – Common noise model for exploration  
- [[TD3]] (Twin Delayed DDPG) – An improved variant of DDPG  

---

## 📚 Further Reading

- [DDPG Paper - Lillicrap et al., 2015](https://arxiv.org/abs/1509.02971)  
- [Spinning Up - DDPG](https://spinningup.openai.com/en/latest/algorithms/ddpg.html)  
- [TD3 Paper - Fujimoto et al., 2018](https://arxiv.org/abs/1802.09477)  
- [OpenAI Baselines](https://github.com/openai/baselines)  

---
