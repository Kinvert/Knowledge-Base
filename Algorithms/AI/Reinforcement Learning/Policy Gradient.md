# Policy Gradient

**Policy Gradient** methods are a class of reinforcement learning algorithms that optimize the policy directly by performing gradient ascent on the expected cumulative reward. Instead of estimating value functions and deriving policies indirectly, these methods parameterize the policy and improve it using gradients computed from interactions with the environment.

Policy gradients enable learning in complex, high-dimensional, and continuous action spaces and are foundational to modern RL algorithms like [[PPO]], [[TRPO]], and [[A2C]].

---

## 🔍 Overview

- Policy Gradient methods represent the policy as a parameterized function (often a neural network).  
- The goal is to maximize the expected return by adjusting the policy parameters in the direction of the performance gradient.  
- The gradient is estimated using sampled trajectories and the **policy gradient theorem**.  
- Can be used with both stochastic and deterministic policies.  

---

## 🧠 Core Concepts

- `Policy π_θ(a|s)`: Probability distribution over actions given state, parameterized by θ  
- `Objective J(θ)`: Expected cumulative reward under policy π_θ  
- `REINFORCE Algorithm`: Monte Carlo based policy gradient estimator  
- `Advantage Function A(s, a)`: Used to reduce variance in gradient estimates  
- `Baseline`: Value function or other estimator to reduce variance without bias  
- `Stochastic vs Deterministic`: Policy can be probabilistic or deterministic  

---

## 🧰 Use Cases

- Continuous control tasks in robotics  
- Complex decision-making environments with large or continuous action spaces  
- Games with large action/state spaces  
- Situations where explicit value function modeling is difficult or inefficient  
- Part of actor-critic architectures  

---

## ✅ Pros

- Directly optimizes the policy for better performance  
- Naturally handles stochastic policies, useful for exploration  
- Works well in continuous and high-dimensional action spaces  
- Compatible with function approximation (e.g., deep neural networks)  
- Can incorporate variance reduction techniques (baselines, advantage estimators)  

---

## ❌ Cons

- High variance in gradient estimates can slow learning  
- Sample inefficient compared to some off-policy methods  
- Sensitive to hyperparameters like learning rate  
- Can converge to local optima without careful tuning  

---

## 📊 Comparison Table: Policy Gradient vs Value-Based Methods

| Aspect           | Policy Gradient                 | Value-Based (e.g., Q-Learning)  |
|------------------|--------------------------------|--------------------------------|
| Policy Type      | Explicitly parameterized policy | Derived implicitly from value  |
| Action Space     | Discrete or continuous          | Typically discrete             |
| Exploration     | Built-in via stochastic policies| Often requires ε-greedy or others|
| Sample Efficiency| Moderate to low                 | Generally higher               |
| Stability       | Can be unstable without tricks | More stable with bootstrapping |
| Applicability   | Good for continuous control    | Mostly discrete actions        |

---

## 🤖 In Robotics Context

| Scenario                  | Role of Policy Gradient                      |
|---------------------------|----------------------------------------------|
| Manipulation tasks        | Learning smooth and adaptive motor policies  |
| Legged locomotion        | Optimizing complex gaits with continuous outputs |
| Drone flight control     | Handling high-dimensional continuous actions  |
| Multi-agent coordination | Training decentralized stochastic policies    |

---

## 🔧 Compatible Items

- [[Reinforcement Learning]] – Core method for policy optimization  
- [[Advantage Function]] – Used to improve gradient estimation  
- [[Actor Critic]] – Combines policy gradient with value function learning  
- [[GAE]] – Popular advantage estimator for policy gradients  
- [[PPO]], [[TRPO]], [[A2C]] – Algorithms built on policy gradient principles  
- [[RL Trajectory]] – Used to compute gradients from experience  

---

## 🔗 Related Concepts

- [[REINFORCE Algorithm]] (Basic Monte Carlo policy gradient method)  
- [[Stochastic Policy]] (Policy gradients commonly use probabilistic policies)  
- [[Deterministic Policy Gradient]] (Variant for deterministic policies)  
- [[Baseline]] (Reduces variance in gradient estimates)  
- [[Exploration vs Exploitation]] (Stochastic policies encourage exploration)  

---

## 📚 Further Reading

- [Sutton & Barto – Chapter 13](http://incompleteideas.net/book/the-book.html)  
- [Policy Gradient Theorem (Sutton et al. 2000)](https://papers.nips.cc/paper/1713-policy-gradient-methods-for-reinforcement-learning-with-function-approximation.pdf)  
- [OpenAI Spinning Up – Policy Gradient](https://spinningup.openai.com/en/latest/algorithms/vpg.html)  
- [Deep Reinforcement Learning with Policy Gradients](https://arxiv.org/abs/1312.5602)  
- [Deterministic Policy Gradient (Silver et al. 2014)](https://arxiv.org/abs/1509.02971)  

---
