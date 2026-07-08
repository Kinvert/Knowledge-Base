# Decision Transformer

`Decision Transformer` is the first widely used “transformers instead of RL” baseline: offline trajectories are converted into token sequences so next action tokens can be predicted with supervised learning, without fitting a standard Bellman value update.

---

## Overview

- Paper: `Decision Transformer: Reinforcement Learning via Sequence Modeling` (`arXiv:2106.01345`, 2021).
- Core idea: predict next action from the trajectory stream of `return-to-go + state + action`.
- Standard token ordering: `R_1, s_1, a_1, R_2, s_2, a_2, ...`
- Control knob: initial return-to-go token (`R_0`) adjusts style/aggressiveness.

---

## Core mechanism

1. Build offline windows from logs/demos.
2. Tokenize each step as return-state-action (`R_t, s_t, a_t`), optionally with done/termination markers.
3. Train a causal transformer in teacher-forced mode on next-action prediction.
4. At inference, feed the recent state and a target return, then autoregressively emit actions and execute them online.

Compared to value-based RL, this removes Bellman backups from the core objective and turns control into sequence decoding.

---

## Why this is usually the first stop for tokenized control

- Strongest conceptual match to your stack: sequence modeling and token prediction.
- No need for environment interaction to start training (offline-first).
- Very easy to benchmark against other token models because everything is one data pipeline.

What it is not:
- A learned dynamics model for planning.
- A true MPC/search policy optimizer.
- A method for heavy multi-agent or language-conditioned semantics by itself.

---

## Comparison chart

| Method | Objective | Inputs | Planning behavior | Training cost | Good fit for |
|---|---|---|---|---|---|
| **Decision Transformer** | next-action prediction on return-state-action stream | `R_t, s_t, a_t` windows | direct autoregressive rollout, return-conditioned | medium | baseline for offline tokenized control |
| **Trajectory Transformer** | joint sequence modeling of trajectories | state/action/reward tokens | search/beam style rollout over future trajectory tokens | medium-high | tasks benefiting from explicit rollout planning |
| **Behavior Transformer / VQ-BeT** | action code autoregression | behavior/action/state token stream | local autoregressive action generation | medium | imitation-heavy continuous control |
| **ACT / ALOHA** | chunked behavior prediction | action chunks from teleop examples | overlap-replan execution | medium | contact/ dexterous teleoperation |
| **FAST** | action tokenizer + decoder swap | compressed action tokens | unchanged transformer head with better action codec | low-medium | reducing raw action tokenization pain |
| **Diffusion Policy** | denoising in action space | observation + action trajectories | iterative sampling per step | high | multi-modal action surfaces where smoothness matters |
| **TD-MPC** | latent model + value + MPC | state-action-reward transitions + value targets | explicit latent planning | high | model-based tasks needing rollout optimization |
| **Dreamer** | latent world model + actor/critic | image/state trajectories | imagination rollouts via model | high | long-horizon control with rich dynamics |

---

## Practical implementation notes

- Sequence length:
  - short windows (e.g., 20–40 steps) for first stability,
  - increase once returns improve and memory allows.
- Normalize returns if task reward scales vary over time.
- Keep action encoding exactly identical between train and deploy.
- For continuous actions, if using quantized tokens, validate de-quantization latency and clipping behavior.
- Add safety caps after the model outputs tokens:
  - command-rate limits,
  - action magnitude bounds,
  - watchdog stop behavior.

Minimal flow:

```text
build offline dataset of trajectories
for each trajectory:
  compute returns-to-go R_t
  create windows of [R_t, s_t, a_t]
train transformer with cross-entropy / action regression loss

deploy:
  choose target return R_0
  loop:
    append latest state
    predict next action token
    run safety filters
    execute action
```

---

## Pros

- Lowest-friction token-first RL baseline for trajectory data.
- Clearly interpretable control conditioning with `R_0`.
- Easy to compare against BC and offline RL baselines.
- Fits both small and larger repositories because the stack is simple.

## Cons

- No planning module built in.
- OOD generalization is limited by offline coverage.
- Poor return signals can overfit to noisy token contexts.
- Continuous control requires careful token codec decisions.

---

## External references

- Paper: https://arxiv.org/abs/2106.01345
- Code: https://github.com/kzl/decision-transformer

---

## Related notes

- [[Trajectory Transformer]]
- [[Behavior Transformer]]
- [[ACT Action Chunking Transformer]]
- [[FAST]]
- [[Diffusion Policy]]
- [[TD-MPC]]
- [[Dreamer]]

---

## Summary

Use `Decision Transformer` when you want a practical “next-token” policy baseline with minimal architecture churn. It is your best fit for early-stage tokenized control experiments, then you can compare against heavier stack methods (`Trajectory Transformer`, `TD-MPC`, `Dreamer`) to decide whether planning/model-based layers are worth the complexity.
