# 🟣 Stable Baselines3 (SB3)

**Stable Baselines3 (SB3)** is a set of reliable implementations of reinforcement learning (RL) algorithms in Python, built using [[PyTorch]]. It is a successor to the original Stable Baselines (which was built on TensorFlow) and provides easy-to-use, well-tested, and reproducible RL tools.

---

## 🧠 Summary

- Built on [[PyTorch]]
- Implements key RL algorithms (PPO, DQN, A2C, TD3, SAC, etc.)
- Designed for compatibility with [[OpenAI Gym]] / [[Gymnasium]]
- Actively maintained and widely used in academia and industry
- Excellent documentation and community support

---

## 🚀 Key Features

| Feature                   | Description                                                                 |
|---------------------------|-----------------------------------------------------------------------------|
| Modular Design            | Clean and extensible object-oriented architecture                           |
| Callback System           | Supports training hooks (logging, saving models, evaluation, etc.)         |
| Evaluation Utilities      | Includes easy model evaluation and hyperparameter tuning tools              |
| Vectorized Environments   | Parallel environment execution for efficient training                       |
| Integration Ready         | Works well with [[OpenAI Gym]], [[Gymnasium]], [[PettingZoo]], [[RLlib]]   |
| Exportable Models         | Models can be saved/loaded easily with `.zip` files                         |

---

## 📦 Supported Algorithms

| Algorithm | Type              | Use Case Example                        |
|----------|-------------------|-----------------------------------------|
| PPO      | On-policy         | Stable policy optimization, general use |
| A2C      | On-policy         | Advantage-based policy gradient         |
| DQN      | Off-policy        | Discrete action spaces                  |
| TD3      | Off-policy        | Continuous actions with twin critics    |
| SAC      | Off-policy        | Sample-efficient stochastic policy      |
| DDPG     | Off-policy        | Deprecated in favor of TD3/SAC          |

---

## 🏗️ Training Flow Overview

- Define environment: `gym.make()`
- Create model: `PPO("MlpPolicy", env)`
- Train model: `model.learn(total_timesteps=100000)`
- Save/load: `model.save()` and `model.load()`
- Evaluate: Use `evaluate_policy()` or callbacks

---

## 🧪 Comparison with Other RL Frameworks

| Feature / Framework  | SB3                    | [[Ray RLlib]]         | [[TF-Agents]]         | [[Unity ML-Agents]]    |
|----------------------|------------------------|------------------------|------------------------|------------------------|
| Backend              | PyTorch                | TensorFlow / PyTorch  | TensorFlow            | C# / Python            |
| Ease of Use          | ✅ Easy                 | ⚠️ More complex         | ⚠️ More boilerplate     | ✅ High for Unity users |
| Multi-agent Support  | 🚫 Limited             | ✅ Yes                 | ✅ Yes                 | ✅ Yes                 |
| Custom Environments  | ✅ Easy                | ✅ Moderate            | ✅ Moderate            | ✅ Yes                 |
| Parallel Training    | ✅ With VecEnv         | ✅ Native              | ⚠️ Requires setup      | ✅ Yes                 |

---

## 🏆 Strengths

- Easy to use with minimal boilerplate
- Great documentation and examples
- Reproducible and well-tested
- Clean codebase and active development

---

## ⚠️ Weaknesses

- No native multi-agent support (use wrappers or PettingZoo)
- Less flexible for highly customized training workflows
- Some algorithms (e.g. DDPG) are deprecated

---

## 📈 Use Cases

- Benchmarking with [[OpenAI Gym]] or [[Gymnasium]]
- Training agents for games, robotics, control systems
- Research in RL algorithm performance
- Educational purposes and tutorials

---

## 🔗 Related Notes

- [[Reinforcement Learning]]
- [[OpenAI Gym]]
- [[PyTorch]]
- [[Gymnasium]]
- [[Ray RLlib]]
- [[PettingZoo]]
- [[MuJoCo]]
- [[Simulation Environments]]

---

## 🌐 External Resources

- [GitHub - Stable Baselines3](https://github.com/DLR-RM/stable-baselines3)
- [Official Docs](https://stable-baselines3.readthedocs.io/)
- [Colab Tutorials](https://colab.research.google.com/github/DLR-RM/stable-baselines3)
- [Zoo of Pretrained Agents](https://github.com/DLR-RM/rl-baselines3-zoo)

---
