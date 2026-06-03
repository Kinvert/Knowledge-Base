# 💾 Offline RL for Robotics

**Offline RL for Robotics** learns policies from fixed datasets instead of collecting new experience online. It is useful when real robot interaction is expensive, dangerous, or slow, but it is sensitive to dataset quality and distribution shift.

---

## 📚 Overview

Offline RL tries to improve beyond behavior cloning while staying within the support of logged data. In robotics, this matters because datasets from teleoperation, scripted policies, or previous runs can be reused without risking hardware. The main danger is extrapolation: the value function may assign high value to actions the dataset never tested.

---

## 🧠 Core Concepts

- **Dataset Support**: State-action region covered by the logged data.
- **Extrapolation Error**: Bad value estimates for unseen actions.
- **Conservatism**: Penalizing out-of-distribution actions.
- **Behavior Policy**: Policy that generated the dataset.
- **Advantage-Weighted Learning**: Imitates actions weighted by estimated value.
- **Replay Dataset**: Fixed buffer of transitions or trajectories.

---

## 📊 Comparison Chart

| Method | Type | Strength | Weakness | Robotics Use |
|---|---|---|---|---|
| Behavior Cloning | Supervised IL | Simple and stable | Cannot improve much | First baseline |
| CQL | Offline RL | Conservative Q-values | Can be too conservative | Manipulation datasets |
| IQL | Offline RL | Stable and practical | Sensitive to data | Robotic control |
| AWAC | Offline-to-online | Good fine-tuning path | Needs value estimates | Real robot adaptation |
| TD3+BC | Offline continuous control | Simple strong baseline | Hyperparameter sensitive | MuJoCo-style tasks |
| Diffusion Policy | Imitation | Strong visuomotor demos | Not value-based RL | Robot manipulation |

---

## ✅ Pros

- Reuses expensive real robot data.
- Safer than online exploration on hardware.
- Can learn from mixed-quality datasets.
- Works well with teleoperation pipelines.
- Useful bridge from imitation to fine-tuning.

---

## ❌ Cons

- Dataset coverage dominates performance.
- Hard to know when policy leaves the data distribution.
- Evaluation in the real world is still required.
- Poor rewards or missing terminal labels hurt learning.
- Algorithms can be less stable than behavior cloning.

---

## 🧰 Robotics Workflow

1. Collect demonstrations or scripted rollouts.
2. Validate timestamps, observations, actions, rewards, and terminal labels.
3. Train behavior cloning first.
4. Train an offline RL baseline such as IQL, CQL, AWAC, or TD3+BC.
5. Evaluate offline metrics and real robot rollouts separately.
6. Fine-tune cautiously with safety constraints if needed.

---

## 🔗 Related Notes

- [[Imitation Learning]]
- [[Replay Buffer]]
- [[Experience Replay]]
- [[LeRobot]]
- [[robomimic]]
- [[Robot Dataset Formats]]
- [[Robot Data Collection and Teleoperation]]

---

## 🌐 External Resources

- D4RL: https://github.com/Farama-Foundation/D4RL
- robomimic: https://robomimic.github.io/
- Offline RL Tutorial: https://offline-rl.github.io/

---

## 📝 Summary

Offline RL is attractive for robotics because robot data is expensive. Its main challenge is staying close enough to the dataset that learned value estimates remain trustworthy.
