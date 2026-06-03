# 🧠 Model-Based RL

**Model-Based RL** learns or uses a model of environment dynamics to improve planning, policy learning, or sample efficiency. In robotics, it connects reinforcement learning with [[Trajectory Optimization and MPC]], system identification, and world models.

---

## 📚 Overview

Model-free RL learns from trial and error without explicitly predicting future states. Model-based RL learns a dynamics model or uses a known simulator to predict outcomes. This can reduce data needs, but model errors can compound and mislead the policy.

---

## 🧠 Core Concepts

- **Dynamics Model**: Predicts next state from current state and action.
- **Planning**: Uses the model to choose actions.
- **Dyna-Style Learning**: Trains on real and model-generated experience.
- **Model Predictive Control**: Replans using a dynamics model.
- **Uncertainty Estimation**: Measures confidence in model predictions.
- **Model Bias**: Error caused by learning from an imperfect model.

---

## 📊 Comparison Chart

| Approach | Uses Model | Strength | Weakness | Robotics Fit |
|---|---|---|---|---|
| [[PPO]] | No | Robust model-free baseline | Data hungry | High in sim |
| [[SAC]] | No | Sample-efficient model-free | Still needs data | High |
| PETS | Learned ensemble model | Sample efficient | Planning compute | Medium-high |
| MBPO | Short model rollouts | Efficient learning | Model bias | Medium-high |
| Dreamer | Latent world model | Learns in imagination | Complex | High research value |
| MPC | Known model | Constraint-aware | Model required | Very high |

---

## ✅ Pros

- Can reduce environment interaction.
- Connects naturally to robotics dynamics and MPC.
- Useful when real robot data is expensive.
- Enables planning and counterfactual rollout.
- Can incorporate uncertainty for safer behavior.

---

## ❌ Cons

- Learned models can be wrong in important states.
- Contact-rich robotics is hard to model.
- Planning can be computationally expensive.
- Model error compounds over horizon.
- Often more complex than model-free baselines.

---

## 🔗 Related Notes

- [[World Models for Control]]
- [[Trajectory Optimization and MPC]]
- [[System Identification for Sim2Real]]
- [[Robot Dynamics and Spatial Algebra]]
- [[SAC]]
- [[PPO]]

---

## 🌐 External Resources

- PETS Paper: https://arxiv.org/abs/1805.12114
- MBPO Paper: https://arxiv.org/abs/1906.08253
- DreamerV3 Paper: https://arxiv.org/abs/2301.04104

---

## 📝 Summary

Model-based RL is attractive in robotics because robot data is expensive and dynamics matter. Its main challenge is keeping model errors from becoming policy errors.
