# Discrete Action Space

A **Discrete Action Space** is a type of action space in reinforcement learning (RL) where the agent selects actions from a **finite set** of possible options. This is in contrast to a [[Continuous Action Space]], where actions can take any value within a range.

Discrete action spaces are common in many RL problems, especially in game environments, navigation tasks, and simpler robotic control systems. Algorithms like [[DQN]], [[A2C]], and [[PPO]] are well-suited for discrete action domains.

---

## 🧠 Overview

In a discrete action space:
- The total number of actions `n` is finite
- The agent's policy maps states to a probability distribution over the `n` actions
- Actions are indexed numerically: `{0, 1, 2, ..., n-1}`

For example:
- In a 4-directional grid world: `['UP', 'DOWN', 'LEFT', 'RIGHT']`
- In a game: `['Jump', 'Shoot', 'Crouch']`

---

## 🧪 Use Cases

- Video game playing (e.g. Atari, Mario, Doom)  
- Discrete control of robots or drones (e.g. move forward/backward)  
- Turn-based strategy simulations  
- Reinforcement learning benchmarks like [[Gym]] environments  
- Any task with clearly defined, mutually exclusive action choices

---

## 📊 Comparison Table

| Action Space Type      | Action Representation | Algorithms              | Example Task                |
|------------------------|------------------------|--------------------------|-----------------------------|
| Discrete Action Space  | Finite set (e.g., 0–3) | [[DQN]], [[PPO]], [[A2C]] | Atari Breakout              |
| [[Continuous Action Space]] | Real-valued vectors      | [[DDPG]], [[SAC]], [[TRPO]] | Robotic arm joint control   |
| Hybrid                 | Mix of both types       | Advanced custom setups    | Game with buttons + cursor  |

---

## ⚙️ Key Properties

- Actions are often mapped to integers internally
- Policies output softmax distributions over actions
- Often easier to train and debug than continuous domains
- Naturally supports tabular or one-hot encoding

---

## ✅ Pros

- Simple and efficient to represent  
- Compatible with many standard RL algorithms  
- Easier policy interpretation and debugging  
- Supports stochastic action selection (via softmax)

---

## ❌ Cons

- Limited precision for complex control tasks  
- Not suitable for real-valued control without discretization  
- Explodes in size for combinatorial multi-discrete actions  
- Requires careful action design to avoid redundancy or ambiguity

---

## 🔗 Related Concepts

- [[Continuous Action Space]]  
- [[Action Space]]  
- [[Policy]]  
- [[DQN]]  
- [[A2C]]  
- [[PPO]]  
- [[Actor Critic]]  
- [[Gym]]  
- [[RL Environment]]

---

## 📚 Further Reading

- [OpenAI Gym – Discrete Action Environments](https://www.gymlibrary.dev)  
- [Spinning Up – Discrete vs. Continuous](https://spinningup.openai.com/en/latest/spinningup/rl_intro.html#environments)  
- DeepMind Control Suite and ALE (Atari Learning Environment)

---
