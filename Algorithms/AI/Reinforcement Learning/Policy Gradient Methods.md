# Policy Gradient Methods

**Policy Gradient Methods** are a family of reinforcement learning algorithms that directly optimize the policy by estimating gradients of expected return with respect to policy parameters. Unlike value-based methods, these focus on learning stochastic or deterministic policies that map states to actions.

---

## 🔍 Overview

- Aim to find the optimal policy by maximizing expected cumulative reward via gradient ascent.  
- Handle both discrete and continuous action spaces effectively.  
- Often parameterize policies with neural networks (deep RL).  
- Can optimize stochastic policies (probability distributions) or deterministic policies.  
- Basis for many advanced algorithms like [[PPO]], [[TRPO]], [[DDPG]], and [[SAC]].

---

## 🧠 Core Concepts

- **Policy Parameterization**: Policy π(a|s; θ) depends on parameters θ.  
- **Objective Function**: Maximize expected return J(θ) = E[sum of rewards].  
- **Gradient Estimation**: Use the policy gradient theorem to compute ∇θJ(θ).  
- **Stochastic Policies**: Represent probability distributions over actions for exploration.  
- **Deterministic Policies**: Map states directly to actions (used in continuous control).  
- **Variance Reduction**: Techniques like baseline functions or advantage estimation reduce gradient variance.

---

## 🧰 Use Cases

- Tasks with continuous action spaces (robotics, control).  
- Situations where the policy must be stochastic (games, exploration-heavy tasks).  
- Complex environments where value function approximation alone is insufficient.  
- Large-scale deep RL setups requiring stable policy updates.

---

## ✅ Pros

- Naturally handle continuous and high-dimensional action spaces.  
- Can learn stochastic policies enabling principled exploration.  
- Theoretically grounded with convergence guarantees under certain conditions.  
- Flexible with a variety of policy architectures and parameterizations.

---

## ❌ Cons

- High variance in gradient estimates can slow learning.  
- Sample inefficient compared to some off-policy methods.  
- Requires careful tuning of learning rates and exploration parameters.  
- May suffer from local optima due to non-convex optimization landscape.

---

## 📊 Comparison Table: Policy Gradient vs Value-Based Methods

| Aspect            | Policy Gradient Methods        | [[Value-Based Methods]]               |
| ----------------- | ------------------------------ | ------------------------------------- |
| Policy Type       | Directly parameterized         | Implicit via value function           |
| Action Space      | Discrete and continuous        | Typically discrete (extensions exist) |
| Exploration       | Built into stochastic policies | Often external (ε-greedy, softmax)    |
| Sample Efficiency | Generally lower                | Generally higher                      |
| Stability         | Can be unstable without tricks | Usually more stable                   |
| Examples          | PPO, TRPO, DDPG, SAC           | DQN, Double DQN                       |

---

## 🔧 Compatible Items

- [[Stochastic Policy]] – Core to many policy gradient methods  
- [[Advantage Function]] – Used for variance reduction  
- [[Baseline Function]] – Helps stabilize training  
- [[GAE]] (Generalized Advantage Estimation) – Advanced advantage estimation  
- [[Deterministic Policy Gradient]] – Variant for deterministic policies  

---

## 🔗 Related Concepts

- [[Reinforcement Learning]] – Overall domain  
- [[On-Policy]] and [[Off-Policy]] – Policy gradient methods can be either  
- [[Exploration vs Exploitation]] – Managed via policy stochasticity  
- [[Temporal Difference Learning]] – Can be combined with policy gradients  
- [[TD Learning]]
- [[Replay Buffer]] – Used in off-policy actor-critic variants  

---

## 📚 Further Reading

- [Policy Gradient Methods - Sutton et al., 1999](http://incompleteideas.net/papers/PG-main.pdf)  
- [Reinforcement Learning: An Introduction - Sutton & Barto (Chapters on Policy Gradient)](http://incompleteideas.net/book/the-book-2nd.html)  
- [Proximal Policy Optimization (PPO) Paper - Schulman et al., 2017](https://arxiv.org/abs/1707.06347)  
- [Spinning Up in Deep RL - Policy Gradient](https://spinningup.openai.com/en/latest/algorithms/ppo.html)  

---
