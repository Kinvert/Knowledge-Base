# RL Return

In **Reinforcement Learning (RL)**, the *return* is the total accumulated reward an agent receives from a given timestep onward, often denoted as `G_t`. It is a foundational concept that underpins value functions, policy gradients, and the agent’s goal of maximizing long-term reward.

The return may be finite (for episodic tasks) or infinite (for continuing tasks) and is typically computed as a **discounted sum** of future rewards using a discount factor `γ`.

---

## 📚 Overview

At each timestep `t`, the return `G_t` is defined as:

`G_t = R_{t+1} + γ * R_{t+2} + γ² * R_{t+3} + ...`

where:
- `R_t` is the reward at timestep `t`
- `γ` (0 ≤ γ ≤ 1) is the **discount factor** that controls the importance of future rewards

A higher γ emphasizes long-term rewards, while lower γ prioritizes immediate gains.

---

## 🧠 Core Concepts

- `Return (G_t)`: The total reward from timestep `t` onward  
- `Discount Factor (γ)`: Reduces the weight of future rewards  
- `Episodic Tasks`: Return is the sum of rewards until the episode ends  
- `Continuing Tasks`: Return may be infinite, requiring discounted sums  
- `Monte Carlo Estimation`: Return is estimated via full episode rollouts  
- `Temporal-Difference Learning`: Bootstraps return using current estimates  

---

## 🧰 Use Cases

- Defining training objectives for value and policy functions  
- Measuring agent performance across episodes  
- Comparing different policies’ long-term benefits  
- Reward shaping and trajectory evaluation  
- Tracking cumulative success in goal-based environments  

---

## ✅ Pros

- Directly represents the agent’s learning objective  
- Simple to compute with episode data  
- Works well with both value-based and policy-based methods  
- Allows comparison across different tasks or policies  

---

## ❌ Cons

- High variance in sparse reward settings  
- Long episodes can complicate return calculation  
- Sensitive to choice of discount factor  
- Delayed rewards can make credit assignment harder  

---

## 📊 Comparison Table: Return vs Related Quantities

| Quantity         | Description                              | Depends On       | Usage Example                |
|------------------|------------------------------------------|------------------|------------------------------|
| Reward (`R_t`)   | Immediate feedback from environment       | Current state/action | Learning signal for step   |
| Return (`G_t`)   | Cumulative future reward from `t`         | Sequence of rewards | Used in Monte Carlo targets |
| Value (`V(s)`)   | Expected return from state `s`            | Policy & state    | Critic or baseline function |
| Q-Value (`Q(s,a)`) | Expected return from state-action pair | Policy, s, a      | Used in action selection     |

---

## 🤖 In Robotics Context

| Task                     | Return Interpretation                            |
|--------------------------|--------------------------------------------------|
| Pick-and-place           | Total success/failure feedback over trajectory   |
| Navigation               | Total distance reward and penalty for collisions |
| Drone hovering           | Stability rewards accumulated over time          |
| Human interaction        | Long-term success in task cooperation            |
| Multi-agent swarm        | Shared or individual agent return measures       |

---

## 🔧 Compatible Items

- [[Reinforcement Learning]] – Return is central to most algorithms  
- [[RL Reward]] – Return is a sum of rewards  
- [[RL Agent]] – Optimizes for maximum return  
- [[RL Step]] – Each step contributes to the return  
- [[Policy Gradient Methods]] – Use return to weight policy updates  
- [[Actor Critic]] – Return can be a target for the critic  

---

## 🔗 Related Concepts

- [[Value Function]] – Estimates expected return  
- [[Monte Carlo Methods]] – Estimate return via full episode samples  
- [[Temporal Difference Learning]] – Approximates return using bootstrapping  
- [[RL Episode]] – Return is computed over one or more episodes  
- [[Reward Shaping]] – Alters return by modifying reward signals  

---

## 📚 Further Reading

- [Sutton & Barto RL Book – Returns](http://incompleteideas.net/book/the-book.html)  
- [OpenAI Spinning Up: Return](https://spinningup.openai.com/en/latest/spinningup/rl_intro.html#returns-and-value-functions)  
- [Monte Carlo vs TD Methods](https://www.deeplearningbook.org/)  
- [RL Algorithms and Discounting](https://lilianweng.github.io/lil-log/2018/04/08/policy-gradient-algorithms.html)  

---
