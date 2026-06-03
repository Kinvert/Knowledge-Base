# 📉 TQC

**TQC (Truncated Quantile Critics)** is an off-policy continuous-control RL algorithm that extends ideas from [[SAC]] using distributional critics and truncation to reduce overestimation.

---

## 📚 Overview

TQC is relevant to robotics because it is a strong continuous-control algorithm in benchmark settings. It combines maximum-entropy RL with distributional value estimates, then drops high quantiles to avoid overly optimistic Q-values.

---

## 🧠 Core Concepts

- **Distributional Critic**: Predicts a distribution of value estimates instead of one scalar.
- **Quantile Regression**: Learns value quantiles.
- **Truncation**: Drops upper quantiles to reduce overestimation.
- **Maximum Entropy Policy**: Uses SAC-style stochastic exploration.
- **Off-Policy Learning**: Reuses data from a replay buffer.

---

## 📊 Comparison Chart

| Algorithm | Main Idea | Strength | Weakness | Robotics Fit |
|---|---|---|---|---|
| [[SAC]] | Max entropy actor-critic | Strong general baseline | Sensitive tuning | High |
| **TQC** | Truncated distributional critics | Strong benchmark performance | More complex | Medium-high |
| [[TD3]] | Twin delayed critics | Stable deterministic control | Manual exploration | Medium-high |
| REDQ | Critic ensemble | Sample efficiency | More compute | Medium-high |
| [[PPO]] | Clipped on-policy updates | Robust | Less sample efficient | Very high |
| [[DDPG]] | Deterministic actor-critic | Simple lineage | Brittle | Historical |

---

## ✅ Pros

- Reduces Q-value overestimation.
- Strong performance on continuous-control benchmarks.
- Builds on SAC-style exploration.
- Useful algorithmic comparison for robotics tasks.
- Good example of distributional RL in control.

---

## ❌ Cons

- More complex than SAC or TD3.
- Less standard in robotics deployment stacks.
- Distributional critic details add implementation burden.
- Benchmark gains may not matter if simulator or dataset is the bottleneck.

---

## 🔗 Related Notes

- [[SAC]]
- [[TD3]]
- [[Off-Policy]]
- [[Replay Buffer]]
- [[Continuous Action Space]]
- [[MuJoCo]]

---

## 🌐 External Resources

- TQC Paper: https://arxiv.org/abs/2005.04269

---

## 📝 Summary

TQC is a strong continuous-control method that improves SAC-style value estimation with truncated distributional critics. It is useful to know, but not usually the first robotics algorithm to deploy.
