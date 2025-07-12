# TRPO (Trust Region Policy Optimization)

**Trust Region Policy Optimization (TRPO)** is a policy gradient method in **Reinforcement Learning (RL)** that focuses on making policy updates safe and stable by ensuring that each update stays within a *trust region*. This prevents drastic policy changes that can destabilize learning.

TRPO uses a theoretically grounded approach based on **constrained optimization**, ensuring that the updated policy does not deviate too far from the current policy, measured using KL divergence.

---

## 📚 Overview

The core idea of TRPO is to **maximize a surrogate objective function** while **constraining the change in policy**. The optimization problem is:

`maximize: E[ (π_θ(a|s) / π_θ_old(a|s)) * A(s, a) ]`  
`subject to: D_KL(π_θ_old || π_θ) ≤ δ`

Where:
- `A(s, a)` is the advantage function  
- `π_θ` is the new policy  
- `π_θ_old` is the old policy  
- `D_KL` is the KL divergence between the old and new policies  
- `δ` is a small constant controlling how much the policy is allowed to change

---

## 🧠 Core Concepts

- `Policy Gradient`: Optimizes the policy directly using gradient ascent  
- `Trust Region`: Limits policy change using a KL divergence constraint  
- `Surrogate Objective`: Approximation of expected reward  
- `Conjugate Gradient`: Efficiently solves the constrained optimization  
- `Line Search`: Ensures KL constraint is respected after update  
- `Advantage Estimation`: Typically uses [[GAE]] to reduce variance  

---

## 🧰 Use Cases

- High-precision control tasks where stability is critical  
- Robotics environments with sensitive dynamics  
- Environments with sparse or delayed rewards  
- Simulation environments where retraining is expensive  
- Any scenario where conservative, stable policy updates are preferred  

---

## ✅ Pros

- Theoretically grounded for monotonic policy improvement  
- Ensures stable learning through constrained updates  
- Reduces catastrophic policy degradation  
- Handles both discrete and continuous action spaces  
- Empirically effective on many benchmark tasks  

---

## ❌ Cons

- Computationally intensive (uses second-order optimization)  
- More complex to implement than simpler methods like PPO  
- On-policy: less sample efficient  
- Limited scalability to very large neural networks  
- Slower training compared to PPO in practice  

---

## 📊 Comparison Table: TRPO vs PPO vs DDPG

| Algorithm | Policy Type | On/Off Policy | Sample Efficiency | Optimization Type | Stability | Notes                         |
|-----------|--------------|---------------|-------------------|--------------------|-----------|-------------------------------|
| TRPO      | Stochastic   | On-policy      | Medium            | Constrained (KL)   | Very High | Theoretical guarantees         |
| PPO       | Stochastic   | On-policy      | Medium            | Clipped Objective  | High      | Faster & simpler than TRPO     |
| DDPG      | Deterministic| Off-policy     | High              | Q-learning + actor | Medium    | Works well in continuous space |

---

## 🤖 In Robotics Context

| Task                  | TRPO Application                                 |
|-----------------------|--------------------------------------------------|
| Gripper precision     | Prevents overfitting or oscillating policies     |
| Legged locomotion     | Maintains safe updates to preserve gait stability|
| Drone maneuvering     | Avoids erratic behavior from unstable policies   |
| Multi-step tasks      | Ensures stable convergence over long sequences   |
| Sim-to-real transfer  | Preserves behaviors when moving across domains   |

---

## 🔧 Compatible Items

- [[Reinforcement Learning]] – TRPO is a high-stability RL algorithm  
- [[Policy Gradient Methods]] – Core family TRPO belongs to  
- [[Advantage Function]] – Used to scale policy gradients  
- [[GAE]] – Typically used to estimate `A(s, a)`  
- [[Actor Critic]] – TRPO uses separate actor and value networks  
- [[TD Learning]] – For bootstrapping critic values  

---

## 🔗 Related Concepts

- [[PPO]] (Simplified version of TRPO with clipped objective)  
- [[Kullback-Leibler Divergence]] (Measures policy deviation)  
- [[Constrained Optimization]] (Foundation of TRPO's update step)  
- [[Second Order Methods]] (Used in solving TRPO's update)  
- [[Trust Region Methods]] (Class of methods using bounded updates)  

---

## 📚 Further Reading

- [TRPO Original Paper (Schulman et al. 2015)](https://arxiv.org/abs/1502.05477)  
- [Spinning Up: TRPO](https://spinningup.openai.com/en/latest/algorithms/trpo.html)  
- [OpenAI Baselines: TRPO](https://github.com/openai/baselines)  
- [TRPO vs PPO Comparison](https://stable-baselines.readthedocs.io/en/master/guide/algos.html#ppo-vs-trpo)  

---
