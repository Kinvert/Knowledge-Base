# SAC

**Soft Actor-Critic (SAC)** is a state-of-the-art [[off-policy]] deep reinforcement learning algorithm designed for [[Continuous Action Space]]s. It combines the benefits of maximum entropy reinforcement learning with actor-critic methods to achieve both sample efficiency and stable training.

---

## 🔍 Overview

- SAC maximizes a trade-off between expected reward and policy entropy, encouraging exploration.  
- It uses a **stochastic policy** (actor) and two Q-value critics to mitigate overestimation bias.  
- Off-policy nature allows sample-efficient learning with replay buffers.  
- Widely used for robotic control, simulated locomotion, and continuous control tasks.

---

## 🧠 Core Concepts

- **Maximum Entropy Objective**: Adds an entropy term to the reward to encourage exploration and robustness.  
- **Actor-Critic Architecture**: Separate networks for policy (actor) and value estimation (critics).  
- **Twin Q-Networks**: Two Q-functions to reduce overestimation by taking the minimum of both.  
- **Replay Buffer**: Stores past transitions to reuse for training.  
- **Automatic Entropy Tuning**: Adjusts the entropy coefficient dynamically for balancing exploration/exploitation.

---

## 🧰 Use Cases

- Robotics control (manipulators, locomotion).  
- Simulated continuous control benchmarks (e.g., MuJoCo tasks like HalfCheetah, Hopper).  
- Tasks requiring stable learning and robust exploration.  
- Any continuous action domain benefiting from off-policy sample reuse.

---

## ✅ Pros

- Sample efficient compared to many on-policy methods like PPO.  
- Stable and reliable training due to twin Q-networks and entropy regularization.  
- Encourages effective exploration through entropy maximization.  
- Works well across a broad range of continuous control tasks.  

---

## ❌ Cons

- More complex than vanilla policy gradient methods; requires tuning.  
- Requires neural network function approximators, adding compute overhead.  
- May be less straightforward to implement than simpler RL algorithms.

---

## 📊 Comparison Table: SAC vs Other Continuous Control Algorithms

| Algorithm | On/Off-Policy | Exploration Strategy            | Sample Efficiency | Stability | Use Case Focus                    |
| --------- | ------------- | ------------------------------- | ----------------- | --------- | --------------------------------- |
| SAC       | Off-policy    | Max entropy (stochastic policy) | High              | High      | Continuous control                |
| [[PPO]]   | On-policy     | Clipped objective, stochastic   | Moderate          | High      | General RL, continuous & discrete |
| [[DDPG]]  | Off-policy    | Deterministic policy + noise    | Moderate          | Medium    | Continuous control                |
| [[TD3]]   | Off-policy    | Twin critics + delayed policy   | High              | High      | Continuous control                |
| [[A3C]]   | On-policy     | Stochastic policy               | Low               | Medium    | General RL                        |

---

## 🔧 Compatible Items

- [[Replay Buffer]] – For off-policy experience reuse  
- [[Actor Critic]] – SAC is an actor-critic method  
- [[Entropy Regularization]] – Central to SAC’s exploration  
- [[Continuous Action Space]] – SAC is designed for these environments  
- [[RL Agent]] – Implements SAC as the learning algorithm  

---

## 🔗 Related Concepts

- [[Policy Gradient]] – SAC uses stochastic policy gradients  
- [[TD Learning]] – Critics are trained with temporal difference learning  
- [[Automatic Entropy Tuning]] – Dynamically adjusts exploration-exploitation tradeoff  
- [[Off-Policy Learning]] – SAC’s learning paradigm  
- [[Twin Q Networks]] – Technique for reducing overestimation bias  

---

## 📚 Further Reading

- [Soft Actor-Critic Paper - Haarnoja et al., 2018](https://arxiv.org/abs/1801.01290)  
- [Spinning Up - Soft Actor-Critic](https://spinningup.openai.com/en/latest/algorithms/sac.html)  
- [OpenAI Baselines and Stable-Baselines3](https://stable-baselines3.readthedocs.io/en/master/modules/sac.html)  
- [Deep Reinforcement Learning Course by David Silver](https://www.davidsilver.uk/teaching/)  

---
