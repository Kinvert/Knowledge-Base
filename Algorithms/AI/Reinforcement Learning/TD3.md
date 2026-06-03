# 🎯 TD3

**TD3 (Twin Delayed Deep Deterministic Policy Gradient)** is an off-policy actor-critic algorithm for [[Continuous Action Space]] control. It improves on [[DDPG]] with twin critics, delayed actor updates, and target policy smoothing.

---

## 📚 Overview

TD3 is important in robotics because it is a strong baseline for continuous control. It is usually less exploratory than [[SAC]], but it is simpler in some ways and helps explain the design lineage from [[DDPG]] to modern off-policy algorithms.

---

## 🧠 Core Concepts

- **Twin Critics**: Uses two Q-networks and takes the minimum to reduce overestimation.
- **Delayed Policy Update**: Updates the actor less often than the critics.
- **Target Policy Smoothing**: Adds clipped noise to target actions for more stable Q-learning.
- **Replay Buffer**: Reuses past transitions for sample efficiency.
- **Deterministic Actor**: Produces one action rather than a full stochastic distribution.

---

## 📊 Comparison Chart

| Algorithm | Policy | On/Off Policy | Strength | Weakness |
|---|---|---|---|---|
| [[DDPG]] | Deterministic | Off-policy | Simple continuous control | Overestimation, brittle |
| **TD3** | Deterministic | Off-policy | Stable DDPG successor | Exploration via noise only |
| [[SAC]] | Stochastic | Off-policy | Strong exploration | More moving parts |
| [[PPO]] | Stochastic | On-policy | Robust and common | Less sample efficient |
| TQC | Stochastic | Off-policy | Strong continuous control | More complex |
| REDQ | Stochastic | Off-policy | Very sample efficient | Ensemble compute |

---

## ✅ Pros

- Strong continuous-control baseline.
- More stable than DDPG.
- Sample efficient because it is off-policy.
- Useful comparison point for SAC.
- Works with MuJoCo-style robotics benchmarks.

---

## ❌ Cons

- Exploration depends on manually added noise.
- Sensitive to replay buffer and hyperparameters.
- Less common than PPO or SAC in many robotics stacks.
- Deterministic policy can struggle in multimodal tasks.

---

## 🔗 Related Notes

- [[DDPG]]
- [[SAC]]
- [[Replay Buffer]]
- [[Continuous Action Space]]
- [[Off-Policy]]
- [[MuJoCo]]

---

## 🌐 External Resources

- TD3 Paper: https://arxiv.org/abs/1802.09477
- Spinning Up TD3: https://spinningup.openai.com/en/latest/algorithms/td3.html

---

## 📝 Summary

TD3 is the cleaner, more stable successor to DDPG. It is worth knowing as a continuous-control baseline and as background for more modern off-policy robotics algorithms.
