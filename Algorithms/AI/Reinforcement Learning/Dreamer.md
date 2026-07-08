# Dreamer

`Dreamer` is a family of world-model RL methods that learns dynamics in a latent state space, then plans/controls via imagined rollouts rather than direct behavior cloning.

Canonical development:
- `Dreamer` (2019): latent dynamics + actor-critic on imaginary trajectories.
- `DreamerV2` (2020): stabilized recurrent state-space models and Atari-scale improvements.
- `DreamerV3` (2023): scaling tricks and simplified implementation targeted at performance stability.

---

## Overview

- Core paper: `Dream to Control: Learning Behaviors by Latent Imagination` (`arXiv:1912.01603`).
- Core objective: reduce sample complexity by learning a latent transition model (`s_t` in latent space), reward model, and value/policy heads.
- Core difference from token-only methods:
  - it is a model-based RL stack, not only next-token sequence decoding.
- Strong fit when you care about planning, not just one-step imitation.

---

## Core mechanism

1. Learn a latent encoder for observations to state representation `z_t`.
2. Learn latent transition dynamics `z_{t+1} = f(z_t, a_t)`.
3. Learn reward and value predictors in latent space.
4. Run policy optimization through imagined rollouts in the learned world model.
5. Execute only the first action of each imagined plan and replan on new observations.

This loop gives Dreamer long-horizon behavior with fewer environment interactions than many on-policy baselines.

---

## Why it is not a drop-in NTP replacement

- There is no pure `next_token -> action` simplification:
  - multiple heads (model, reward, value, actor),
  - latent model training dynamics,
  - planning / optimization loop at rollout time.
- Useful as a strong reference architecture when sequence models start failing on sparse-reward or long-horizon dynamics.

---

## Comparison chart

| Method | Core model | Controller type | Long-horizon behavior | Training profile | Main weakness |
|---|---|---|---|---|---|
| **Dreamer / DreamerV2 / V3** | latent world model + value + actor | imagination-based actor-critic | strong long-horizon via latent rollouts | model + actor/value updates | complex stack and tuning cost |
| **TD-MPC / TD-MPC2** | latent dynamics + TD value + MPC planner | receding-horizon optimization | strong sample efficiency with explicit planner | model+planning heavy | higher inference compute at each step |
| **Trajectory Transformer** | token sequence model | autoregressive + beam/rollout decode | moderate, sequence-rollout based | offline-first, simpler | inference search cost |
| **Decision Transformer** | return-conditioned token predictor | autoregressive token action | short-to-mid horizon in token domain | straightforward seq modeling | weaker explicit dynamics reasoning |
| **Diffusion Policy** | denoising generative policy | iterative sampling per chunk | multi-modal smooth behavior | denoiser optimization | inference latency from diffusion steps |
| **Behavior Transformer / VQ-BeT** | behavior token model | autoregressive action tokens | local multi-modal behavior | direct offline imitation | no full latent planning loop |

---

## Practical implementation notes

- Keep model and policy in separate modules with explicit interface contracts.
- Start with small latent dimensions for quick stability checks.
- Use strong validation checks:
  - latent reconstruction quality,
  - rollout consistency,
  - value-plausibility calibration.
- For real robots, combine imagination policy with runtime guards:
  - action/torque bounds,
  - environment-aware fail-safe,
  - periodic replans and emergency stop.

Pseudo-workflow:

```text
train:
  encode observations -> latent z
  train transition model, reward model, value model, actor
rollout:
  z_t = encode(current_observation)
  imagined_rollout = simulate(z_t, candidate_actions)
  optimize action via actor/value
  execute first action
  repeat
```

---

## Pros

- Strong for sparse reward and long-horizon planning settings.
- Can outperform pure token imitation methods where environment dynamics matter most.
- Good sample efficiency profile compared to value-only policy optimization in many control domains.

## Cons

- Complex training stack and more failure modes (model error, imagination error).
- Significant compute overhead versus direct sequence policy baselines.
- Harder to diagnose than policy-only losses.

---

## External references

- Core paper: https://arxiv.org/abs/1912.01603
- DreamerV3 abstract/references: https://arxiv.org/abs/2301.04104
- DreamerV2 related: https://arxiv.org/abs/2006.07830

---

## Related notes

- [[TD-MPC]]
- [[Trajectory Transformer]]
- [[Decision Transformer]]
- [[Behavior Transformer]]
- [[Diffusion Policy]]
- [[Model-Based RL]]
- [[Offline RL for Robotics]]

## Summary

`Dreamer` is the nearest full-stack alternative to your tokenized-control roadmap when you need explicit latent dynamics and long-horizon planning. It is much richer than a single next-token swap and belongs in comparison charts as a model-based baseline with higher complexity and stronger planning potential.
