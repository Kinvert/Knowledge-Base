# 🔴 REDQ

**REDQ (Randomized Ensembled Double Q-Learning)** is an off-policy deep RL algorithm that uses an ensemble of Q-functions to improve sample efficiency and reduce value overestimation in continuous control.

---

## 📚 Overview

REDQ is relevant for robotics because real robot data is expensive. Its goal is to get more learning per environment step by doing many gradient updates per collected transition while using critic ensembles to maintain stability.

---

## 🧠 Core Concepts

- **Critic Ensemble**: Multiple Q-networks trained together.
- **Random Subset Min**: Uses a random subset of critics and takes the minimum for targets.
- **High Update-To-Data Ratio**: Performs many updates per environment step.
- **Off-Policy Learning**: Uses replay buffers for sample reuse.
- **SAC-Style Actor**: Often paired with maximum-entropy stochastic policies.

---

## 📊 Comparison Chart

| Algorithm | Data Efficiency | Compute Cost | Main Strength | Main Weakness |
|---|---|---|---|---|
| [[SAC]] | High | Medium | Strong baseline | Tuning sensitive |
| **REDQ** | Very high | High | More updates per sample | Critic ensemble cost |
| [[TD3]] | High | Medium | Stable DDPG successor | Manual exploration |
| TQC | High | Medium-high | Distributional critic | More complex |
| [[PPO]] | Medium | Medium | Robust training | On-policy data cost |
| [[DDPG]] | Medium-high | Medium | Simple | Brittle |

---

## ✅ Pros

- Designed for sample efficiency.
- Useful idea for data-limited robotics.
- Reduces overestimation with critic ensembles.
- Shows why update-to-data ratio matters.
- Pairs with SAC-style continuous control.

---

## ❌ Cons

- More compute per environment step.
- More implementation complexity.
- Less standard than PPO or SAC in robotics frameworks.
- Ensemble methods can be memory-heavy.
- Real robot performance still depends on dataset quality and safety.

---

## 🔗 Related Notes

- [[SAC]]
- [[TD3]]
- [[Replay Buffer]]
- [[Off-Policy]]
- [[Offline RL for Robotics]]
- [[Continuous Action Space]]

---

## 🌐 External Resources

- REDQ Paper: https://arxiv.org/abs/2101.05982

---

## 📝 Summary

REDQ is useful for understanding sample-efficient continuous control. For robotics, its ideas matter most when environment interaction is expensive and compute is cheaper than data.
