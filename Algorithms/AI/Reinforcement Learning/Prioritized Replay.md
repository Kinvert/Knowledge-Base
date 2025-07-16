# Prioritized Replay

**Prioritized Replay** is an extension of [[Experience Replay]] used in off-policy reinforcement learning (RL) algorithms. Instead of sampling past experiences uniformly, transitions are sampled **based on priority**, typically measured by the **temporal-difference (TD) error**. This ensures that more "informative" experiences (those with higher learning potential) are revisited more often, improving learning speed and convergence.

This technique was introduced in the 2015 paper "Prioritized Experience Replay" by Schaul et al., which extended [[DQN]] to perform better in sparse and complex environments.

---

## 🧠 Overview

In standard experience replay, transitions are sampled uniformly from a [[Replay Buffers]]. However, not all transitions contribute equally to learning. Prioritized Replay addresses this by:

- Assigning a **priority** to each transition (e.g. based on TD error)
- Sampling transitions with higher priorities more frequently
- Correcting the bias introduced by non-uniform sampling using **importance sampling weights**

There are two main variants:
- **Proportional Prioritization**: Priority directly proportional to TD error  
- **Rank-Based Prioritization**: Sorts transitions by TD error and samples based on rank

---

## 🧪 Use Cases

- Improves learning in environments with sparse or misleading rewards  
- Speeds up convergence in off-policy deep RL algorithms  
- Commonly used in [[DQN]], [[DDPG]], [[SAC]] (especially in high-dimensional spaces)  
- Can be integrated into [[Multi-agent RL]] scenarios for shared learning improvements

---

## 📊 Comparison Table

| Method                 | Sampling Strategy       | Importance Sampling | Stability      | Complexity |
|------------------------|--------------------------|----------------------|----------------|------------|
| Uniform Replay         | Equal for all samples    | Not required         | Stable         | Low        |
| Prioritized Replay     | Based on TD error        | Required             | More sample-efficient | Medium     |
| Rank-Based Prioritized | Based on sorted TD error | Required             | Robust to outliers | Higher      |

---

## ⚙️ Key Features

- **Priority Score (pᵢ)**: Typically `|TD error| + ε` for numerical stability  
- **Sampling Probability**: `P(i) = pᵢ^α / Σpⱼ^α` (α controls prioritization strength)  
- **Importance Sampling Weight**: Corrects bias via `wᵢ = (1 / N * 1 / P(i))^β`  
- **Annealing of β**: Increases over time to reduce bias toward end of training  
- Can be implemented with data structures like sum trees for efficient sampling

---

## ✅ Pros

- Accelerates learning by focusing on informative transitions  
- Helps overcome sparse reward issues  
- Efficient reuse of limited experiences  
- Can reduce sample complexity

---

## ❌ Cons

- Adds overhead and complexity to buffer implementation  
- Needs careful tuning of `α`, `β`, and `ε`  
- Can lead to instability if high-priority transitions dominate  
- Introduces sampling bias (requiring correction)

---

## 🔗 Related Concepts

- [[Experience Replay]]  
- [[Replay Buffers]]  
- [[TD Learning]]  
- [[DQN]]  
- [[SAC]]  
- [[DDPG]]  
- [[Importance Sampling]]  
- [[Off-Policy]]  
- [[RL Transition]]

---

## 📚 Further Reading

- [Prioritized Experience Replay (Schaul et al., 2015)](https://arxiv.org/abs/1511.05952)  
- [OpenAI Baselines – PER Implementation](https://github.com/openai/baselines)  
- [CleanRL and SB3 implementations of PER]  
- [Spinning Up — Discussion on PER](https://spinningup.openai.com/en/latest/spinningup/rl_intro.html#experience-replay)

---
