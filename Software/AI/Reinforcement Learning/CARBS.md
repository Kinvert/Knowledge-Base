# CARBS

**CARBS** (short for *Cost-Aware Randomized Benchmark Search*) is a hyperparameter tuning framework tailored for reinforcement learning workloads. It focuses on efficient exploration of hyperparameter spaces by considering both performance and computational cost, making it well-suited for expensive and stochastic training tasks in RL.

---

## 🔍 Overview

- Designed for **automated hyperparameter optimization** in reinforcement learning.  
- Emphasizes **cost-awareness**, balancing training performance with computation time.  
- Developed to outperform naive grid or random search in noisy, expensive RL settings.  
- Works well for tuning PPO, SAC, DQN, and other algorithms with many sensitive parameters.

---

## 🧠 Core Concepts

- **Multi-Fidelity Optimization**: Uses partial training runs to estimate final performance.  
- **Cost-Aware Sampling**: Prefers configurations that are both performant and efficient.  
- **Bandit-Based Strategy**: Inspired by successive halving and Hyperband-like methods.  
- **Stochasticity-Resilience**: Can cope with noisy performance metrics from RL environments.  
- **Meta-Optimization**: Automates what would otherwise be manual trial-and-error.

---

## 🧰 Use Cases

- Tuning PPO's learning rate, GAE lambda, entropy coefficient, and clip range.  
- Adjusting replay buffer size, batch size, and target network updates for DQN or SAC.  
- Running low-budget optimization for experiments on limited hardware.  
- Efficient sweeping of environments with high variance in rewards (e.g., [[Puffer Environments]]).

---

## ✅ Pros

- Avoids full-length training for each config, saving compute.  
- Handles noisy metrics better than standard Bayesian optimizers.  
- Adapts budget usage based on cost-performance trade-offs.  
- Strong performance on real RL workloads from benchmark suites like MuJoCo and Atari.

---

## ❌ Cons

- Less widely adopted than popular tools like Optuna or Ray Tune.  
- Requires deeper integration with RL training scripts than basic grid search.  
- May need tuning of its own meta-hyperparameters for best results.

---

## 📊 Comparison Table: CARBS vs Other HPO Methods

| Method           | Handles RL Stochasticity | Budget-Aware | Multi-Fidelity | Popular Use in RL |
|------------------|--------------------------|--------------|----------------|--------------------|
| CARBS            | ✅ Yes                   | ✅ Yes       | ✅ Yes         | 🟡 Growing         |
| [[Grid Search]]  | ❌ No                    | ❌ No        | ❌ No          | ✅ Common          |
| [[Random Search]]| ❌ No                    | ❌ No        | ❌ No          | ✅ Common          |
| Bayesian Opt     | 🟡 Sometimes             | 🟡 Partial   | 🟡 Optional    | ✅ Popular         |
| [[Hyperband]]    | ✅ Yes                   | ✅ Yes       | ✅ Yes         | ✅ Common          |

---

## 🔧 Compatible Items

- [[PPO]] – Policy Gradient method with sensitive hyperparameters  
- [[SAC]] – Off-policy method requiring tuning of alpha, tau, buffer size, etc.  
- [[Replay Buffer]] – Parameters such as size and sample strategy can be tuned  
- [[GAE]] – Tuning lambda impacts performance/stability  
- [[Batch Processing]] – Related to batch size and update frequency

---

## 🔗 Related Concepts

- [[Hyperparameter Optimization]] – General category CARBS belongs to  
- [[RL Training Pipelines]] – Where CARBS is integrated  
- [[On-Policy]] / [[Off-Policy]] – CARBS can optimize both types  
- [[RL Algorithm Configuration]] – What CARBS automates  
- [[RL Reward Signal]] – The objective CARBS helps maximize

---

## 📚 Further Reading

- [CARBS GitHub Repository](https://github.com/facebookresearch/CARBS)  
- [Facebook Research Blog on CARBS](https://ai.facebook.com/research/publications/cost-aware-randomized-benchmark-search-for-hyperparameter-optimization/)  
- [Paper: Cost-Aware Randomized Benchmark Search](https://arxiv.org/abs/2111.09179)  
- [Hyperparameter Tuning in RL – Survey](https://arxiv.org/abs/2007.03966)  

---
