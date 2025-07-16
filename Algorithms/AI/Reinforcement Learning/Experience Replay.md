# Experience Replay

**Experience Replay** is a reinforcement learning (RL) technique in which past experiences (state transitions) are stored and later sampled during training. It allows an agent to break the correlation between sequential data, smooth out learning updates, and improve sample efficiency. It is most commonly used in **off-policy** deep RL algorithms like [[DQN]], [[DDPG]], and [[SAC]].

Experience Replay is implemented using a data structure called a [[Replay Buffers]], which stores transitions of the form `(state, action, reward, next_state, done)`.

---

## 🧠 Overview

Reinforcement learning data is inherently sequential and highly correlated. Experience replay mitigates this by:

- Randomizing the order of training data  
- Reusing past data multiple times  
- Allowing training to proceed more stably, even in noisy environments

Replay mechanisms can include:
- **Uniform Sampling**: Every transition has equal sampling probability  
- **Prioritized Sampling**: Important transitions (e.g., high TD error) are sampled more frequently  

---

## 🧪 Use Cases

- Training [[Deep Q Learning]] agents  
- Improving sample efficiency in continuous action spaces  
- Reducing variance in training in noisy or sparse-reward tasks  
- Multi-agent learning with shared replay buffers  
- Accelerating convergence in simulation-based environments

---

## 📊 Comparison Table

| Replay Strategy         | Sampling Method    | Used In           | Key Benefit                       | Notes                                |
|-------------------------|--------------------|-------------------|-----------------------------------|--------------------------------------|
| Experience Replay       | Uniform            | [[DQN]], [[DDPG]] | Decorrelation + memory reuse      | Common baseline                      |
| Prioritized Replay      | TD-error weighted  | PER-enhanced DQN  | Focuses updates on high error     | Needs importance sampling correction |
| On-Policy (no replay)   | N/A                | [[PPO]], [[A3C]]  | No stale data                     | More sample-inefficient              |
| Episodic Replay         | Full episodes      | [[Neural MMO]]    | Long-term credit assignment       | Used in trajectory-based training    |

---

## ⚙️ Core Components

- **Buffer**: Stores past experiences  
- **Sample Function**: Retrieves batches for training  
- **Insert/Replace Logic**: Handles memory capacity limits (e.g., FIFO)  
- **Prioritization (optional)**: Focuses on meaningful samples  
- **Importance Sampling**: Adjusts for biased sampling if prioritization is used  

---

## ✅ Pros

- Stabilizes and smooths learning  
- Makes better use of environment interactions  
- Enables batch training in deep learning frameworks  
- Supports parallelism and distributed agents

---

## ❌ Cons

- Introduces data staleness  
- Adds complexity (especially with prioritization)  
- Ineffective in on-policy algorithms  
- Can consume large amounts of memory

---

## 🔗 Related Concepts

- [[Replay Buffers]]  
- [[DQN]]  
- [[DDPG]]  
- [[SAC]]  
- [[TD Learning]]  
- [[Off-Policy]]  
- [[RL Step]]  
- [[RL Trajectory]]   
- [[Policy Gradient Methods]]

---

## 📚 Further Reading

- [DeepMind DQN Paper (2015)](https://www.nature.com/articles/nature14236)  
- [Prioritized Experience Replay (Schaul et al.)](https://arxiv.org/abs/1511.05952)  
- [Spinning Up – Experience Replay](https://spinningup.openai.com/en/latest/spinningup/rl_intro.html#experience-replay)  
- Implementations in: Stable-Baselines3, CleanRL, Ray RLlib

---
