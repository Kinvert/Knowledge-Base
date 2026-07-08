# TD-MPC

`TD-MPC` (Temporal Difference Model Predictive Control) is a model-based RL method that optimizes actions using a learned latent dynamics model plus a terminal value estimate learned with temporal-difference losses.

---

## Overview

- Core paper: `Temporal Difference Learning for Model Predictive Control` (`arXiv:2203.04955`, ICML 2022).
- Main idea: do short-horizon planning in latent space and use a learned terminal value function for long-horizon return extrapolation.
- Family status:
  - `TD-MPC`: original single-task/low-to-medium scale method.
  - `TD-MPC2`: scalable successor (`arXiv:2310.16828`) that adds robustness and supports large multi-task datasets with one hyperparameter set.
- Key distinction from tokenized next-token methods:
  - it is planning-first and world-model-first, not sequence-LLM-first.

---

## Core mechanism

1. Learn a task-oriented latent dynamics model (state encoding + action-conditioned transition).
2. Learn reward and terminal value head(s) in the same latent framework with TD-style updates.
3. At inference, run trajectory optimization (`H`-step) in latent space using an optimizer (often MPPI-style sampling/updates).
4. Execute the first action and re-plan at every step (receding horizon).

`TD-MPC2` keeps the same conceptual pipeline while changing objective/architecture details for better robustness and scaling across domains, embodiments, and action spaces.

---

## Why this is not your usual next-token swap

- The training target includes latent dynamics and value heads, not only token prediction.
- The controller loops through explicit planning repeatedly.
- This means it is a strong baseline when you need sample efficiency and physical control quality, but integration is heavier than pure token-only pipelines.

---

## Comparison chart

| Method | Core object | Controller | Generalization strategy | Data/mode | Main strength | Main weakness |
|---|---|---|---|---|---|---|
| **TD-MPC** | latent dynamics + TD value | latent trajectory optimization + receding horizon | task-oriented latent model | online or replay-rich | strong sample efficiency in complex control | compute burden from planning each step |
| **TD-MPC2** | scaled/improved TD-MPC | same style with larger/robust world models | single hyperparameter set across many tasks and embodiments | online/large multi-task corpora | scaling + robustness improvements, single set of hyperparameters | larger implementation and training complexity |
| **Decision Transformer** | reward/state/action token sequence | one-pass/autoregressive action generation | fixed-token stack | offline | easy sequence baseline, minimal planning stack | weaker explicit planning than model-predictive loops |
| **Behavior Transformer / VQ-BeT** | action tokenization + BC | autoregressive behavior decoding | demonstration-limited | offline BC datasets | good for imitation and high modality control | less explicit dynamics foresight |
| **ACT / ALOHA** | chunk action imitation | chunk decode + overlap replan | teleop-centric | moderate demos | stable real-robot local behavior | no explicit model-based long-horizon search |
| **Dreamer / world-model actor-critic** | latent generative world model + actor-critic/imaginary rollouts | latent imagination and policy optimization | model-based latent dynamics | large-scale RL (often offline+online) | broad task support and stable training ecosystems | less direct as a plug-in compared with TD-MPC planner |

---

## Where TD-MPC2 is especially relevant

- One codebase and one set of hyperparameters for 100+ tasks has become attractive for continuous-control scaling.
- Paper claim highlights:
  - outperforms prior methods on 104 continuous-control tasks using consistent hyperparameters,
  - scales to a 317M model that handles around 80 tasks across multiple domains/embodiments.
- Useful if your question is “can a single model reason across many tasks with shared planning core?”

---

## Pros

- Better sample efficiency than many model-free methods in complex, high-dimensional action tasks.
- Explicit planning loop gives better control under sparse-reward and long-horizon conditions.
- Works with latent world models where direct pixel reconstruction is not required.
- `TD-MPC2` demonstrates practical scalability plus zero-tuning behavior across many environments.

## Cons

- Higher runtime cost due to recurrent optimization at inference.
- More architecture and hyperparameter surface than one-pass token policies.
- Requires planning-specific engineering (optimizer, rollout horizon, iterations) for each deployment.
- Not a pure “next-token output” framework.

---

## Practical implementation notes

- Start with a smaller `H` and low planning iterations to profile real-time viability.
- Keep reward/value model heads aligned with your normalization and reward scale.
- Evaluate safety on first-action behavior, not full rollout quality only.
- In robotics, keep hard constraints outside the planner:
  - torque/joint limits,
  - rate limits,
  - collision/obstacle checks,
  - watchdog stop logic.

Minimal loop sketch:

```text
observe s_t
z_t = encode(s_t)
plan z_{t:t+H} = argmax over candidate action sequence using latent model + terminal value
execute a_t = first action from plan
insert new observation and repeat
update latent model/replay buffers
```

---

## External references

- Original: https://arxiv.org/abs/2203.04955
- TD-MPC2: https://arxiv.org/abs/2310.16828
- Original repo: https://github.com/nicklashansen/tdmpc
- TD-MPC2 repo: https://github.com/nicklashansen/tdmpc2
- Open-source index mentioned by paper: https://tdmpc2.com

---

## Related notes

- [[Decision Transformer]]
- [[Trajectory Transformer]]
- [[Behavior Transformer]]
- [[Dreamer]]
- [[Diffusion Policy]]
- [[FAST]]
- [[RT-2]]
- [[OpenVLA]]
