# Entropy Regularization

**Entropy Regularization** is a technique in reinforcement learning (RL) used to encourage **exploration** by favoring policies that have higher entropy (i.e., more randomness). It prevents premature convergence to suboptimal deterministic policies by ensuring the agent continues to try a variety of actions, especially during early training phases.

It is especially common in **policy gradient** methods and **soft actor-critic (SAC)**-style algorithms.

---

## 🧠 Overview

Entropy is a measure of **uncertainty** in a probability distribution. In the context of RL, entropy regularization modifies the reward function to:

`J(π) = ExpectedReturn + α * Entropy(π)`

- Where `α` is the **temperature coefficient** (or entropy coefficient), tuning the tradeoff between **exploitation** and **exploration**.
- A higher `α` encourages more exploration, while a lower `α` focuses more on maximizing return.

---

## 🧪 Use Cases

- **Stabilizing training** in policy gradient algorithms  
- **Avoiding deterministic policies** too early during training  
- **Improving performance** in continuous control environments  
- **Soft Q-learning and SAC** rely on this as a core principle  
- Multi-agent systems where agent diversity is important

---

## 📊 Comparison Table

| Regularization Type     | Purpose                        | Used In                     | Tunable? | Notes                                   |
|--------------------------|--------------------------------|------------------------------|----------|------------------------------------------|
| Entropy Regularization   | Encourages exploration         | [[PPO]], [[SAC]], A3C        | ✅        | Adds stochasticity to policies           |
| L2 Regularization        | Prevents overfitting           | Supervised learning          | ✅        | Not exploration-related                  |
| KL Divergence Penalty    | Constrains policy updates      | [[TRPO]], [[PPO]]            | ✅        | Prevents big policy jumps                |
| Temperature Annealing    | Controls randomness over time  | [[SAC]], curriculum learning | ✅        | Often used alongside entropy regularization |

---

## ✅ Pros

- Prevents early convergence to suboptimal behaviors  
- Encourages broad exploration of action space  
- Can improve stability in sparse-reward environments  
- Simple to implement in most policy gradient frameworks

---

## ❌ Cons

- Requires careful tuning of entropy coefficient `α`  
- May cause instability if exploration is overemphasized  
- Adds noise that can slow convergence in deterministic tasks

---

## 🔗 Related Concepts

- [[RL Policy]]  
- [[Stochastic Policy]]  
- [[Policy Gradient Methods]]  
- [[Actor Critic]]  
- [[SAC]] (Soft Actor-Critic)  
- [[Exploration vs Exploitation]]  
- [[RL Reward]]  
- [[KL Divergence]]

---

## 📚 Further Reading

- [Soft Actor-Critic Paper (Haarnoja et al.)](https://arxiv.org/abs/1801.01290)  
- [Spinning Up by OpenAI – Entropy](https://spinningup.openai.com/en/latest/algorithms/sac.html)  
- RLlib, CleanRL, and Stable-Baselines3 implementations of entropy-regularized algorithms

---
