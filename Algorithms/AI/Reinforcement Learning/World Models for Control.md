# 🌍 World Models for Control

**World Models for Control** learn compact predictive models of an environment and use them for planning or policy learning. They are especially relevant to visual robotics, long-horizon control, and sample-efficient reinforcement learning.

---

## 📚 Overview

A world model usually compresses observations into a latent state, predicts future latent states, and trains a policy using imagined rollouts. This is useful when raw observations are high-dimensional, such as camera images, but the policy needs to reason about future outcomes.

---

## 🧠 Core Concepts

- **Latent State**: Compact representation of images, proprioception, and task context.
- **Dynamics Model**: Predicts future latent states.
- **Reward Model**: Predicts rewards or task success.
- **Imagination Rollouts**: Model-generated trajectories used for policy learning.
- **Recurrent State-Space Model**: Common architecture for temporal prediction.
- **Model Error**: Prediction error that can mislead planning or policy learning.

---

## 📊 Comparison Chart

| Method | Model Type | Strength | Weakness | Robotics Relevance |
|---|---|---|---|---|
| PlaNet | Latent dynamics | Early visual control model | Older baseline | Historical |
| Dreamer | Latent imagination | Strong visual RL | Complex | High |
| DreamerV3 | Scaled world model | General RL capability | Heavy implementation | High research |
| PETS | Ensemble dynamics | Uncertainty-aware | Mostly state-based | Medium-high |
| MBPO | Short model rollouts | Good sample efficiency | Model bias | Medium-high |
| DrQ-v2 | No world model | Simpler pixel RL | More interaction | Comparison baseline |

---

## ✅ Pros

- Good fit for image-based control.
- Can improve sample efficiency.
- Enables planning or learning in imagined rollouts.
- Useful conceptual bridge between RL and generative modeling.
- Can combine proprioception, images, and task context.

---

## ❌ Cons

- Complex training pipelines.
- Model errors compound over time.
- Contact-rich manipulation is difficult to predict.
- Real robot distribution shifts can break latent models.
- Harder to debug than model-free RL.

---

## 🧰 Robotics Use Cases

- Visual continuous control in simulation.
- Sample-efficient learning from robot rollouts.
- Learning dynamics priors for MPC.
- Predicting task outcomes from camera observations.
- Comparing model-based RL with imitation methods like [[Diffusion Policy]].

---

## 🔗 Related Notes

- [[Model-Based RL]]
- [[DrQ-v2]]
- [[Trajectory Optimization and MPC]]
- [[Manipulation RL]]
- [[Camera Calibration]]
- [[JAX]]
- [[PyTorch]]

---

## 🌐 External Resources

- World Models Paper: https://arxiv.org/abs/1803.10122
- DreamerV3 Paper: https://arxiv.org/abs/2301.04104

---

## 📝 Summary

World models are a promising path for visual and sample-efficient control, but they add model-learning complexity. For robotics, they are most useful when interaction is expensive and observations are high-dimensional.
