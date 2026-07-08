# ACT and ALOHA

**ACT** = **Action Chunking with Transformers**.  
**ALOHA** = **A Low-cost Open-source Hardware System for Bimanual Teleoperation**.

ACT is a real-robot imitation method that predicts short action chunks, then executes only part of each chunk and replans, reducing immediate compounding error compared to one-step behavior cloning.

---

## Overview

- Paper: `ACT: Action Chunking with Transformers` (`arXiv:2304.13705`, 2023).
- Core idea: predict a block of future actions at each step, overlap execution windows, and recondition constantly on fresh observations.
- Typical pairing: demonstration data gathered through the ALOHA teleoperation ecosystem.
- Why it stays near your topic:
  - token-oriented sequence view,
  - explicit chunking/runtime loop,
  - easy swap point for action tokenizers.

---

## Core mechanism

1. Collect teleop/demonstration trajectories with history windows.
2. Encode short action segment `a_t ... a_{t+H-1}` as an action chunk.
3. Train a CVAE-style model (or equivalent encoder/decoder) + transformer to predict chunks from current context.
4. At inference:
   - run model each step,
   - execute first `k` actions from the predicted chunk,
   - shift window and re-predict.

The overlap-replan pattern is the key behavioral stabilization trick.

---

## How this compares in your tokenized-control stack

- If your pipeline has high control jitter or one-token drift, chunking smooths action trajectories.
- If your deployment needs strict one-token interfaces, add an action tokenizer in front of ACT and keep the same chunk runtime semantics.
- ACT is less about exploration and more about low-latency imitation behavior from high-quality demonstrations.

---

## Comparison chart

| Method | Output unit | Training style | Decoding behavior | Primary strength | Primary weakness |
|---|---|---|---|---|---|
| **ACT / ALOHA** | action chunks (`H` steps) | imitation (teleop demos) with CVAE/transformer | overlapping chunk execute-replan loop | stable control in contact-rich manipulation | chunk-horizon tuning is sensitive |
| **Decision Transformer** | single action token | offline return-conditioned sequence modeling | one-step action decoding | simple control knob with target return | more one-step drift in real-time settings |
| **Trajectory Transformer** | `s,a,r` token sequence | trajectory-level autoregression + search | beam/trajectory decoding | planning-style long-horizon rollouts | higher inference latency |
| **Behavior Transformer / VQ-BeT** | action token sequence | behavior cloning + action tokenization | auto-regressive behavior tokens | strong for multimodal action styles | no explicit return conditioning |
| **Diffusion Policy** | action trajectory or action chunk | denoising imitation objective | iterative denoising steps | smoother trajectories in contact settings | slower per-step inference |
| **TD-MPC / MPC-backed methods** | latent state-action plans | model-based latent dynamics + planning | online optimization at each step | improved long-horizon handling | significantly more components than ACT |

---

## Practical implementation notes

- Key runtime knobs:
  - chunk length `H`,
  - execution ratio `k / H` (how many of predicted actions you execute before replan),
  - context window size.
- Smaller `k` means safer feedback, more compute.
- Keep policy and encoder normalization identical between train and deploy.
- Always run guard layers outside the model:
  - joint/torque clipping,
  - workspace and speed limits,
  - watchdog stop.

Minimal loop:

```text
for each control tick:
  x_t = encode(observation, proprio, recent actions)
  chunk = model.predict_chunk(x_t)
  action = chunk[0:k]          # execute a short prefix only
  run_safety_filters(action)
  execute(action)
  update history
```

---

## Pros

- High practical relevance to real robots (ALOHA-style teleop pipelines).
- Better short-horizon robustness than strict step-by-step action heads.
- Clear deployment pattern: predict, execute prefix, reobserve, repeat.
- Friendly starting point for tokenized action pipelines where continuous control is noisy.

## Cons

- Not reward-learning first; depends heavily on demonstration quality.
- Chunk-length and overlap-ratio are very sensitive hyperparameters.
- Inference stack is heavier than single-step policy heads.
- Quantization/tokenization layers still need explicit design if your core stack is token-first.

---

## External references

- Paper: https://arxiv.org/abs/2304.13705
- ALOHA project page: https://tonyzhaozh.github.io/aloha/
- ALOHA code: https://github.com/tonyzhaozh/aloha
- LeRobot ecosystem context: https://github.com/huggingface/lerobot

---

## Related notes

- [[Decision Transformer]]
- [[Trajectory Transformer]]
- [[Behavior Transformer]]
- [[FAST]]
- [[Diffusion Policy]]
- [[TD-MPC]]
- [[ACT Action Chunking Transformer]]

---

## Summary

`ACT / ALOHA` is a practical real-robot chunked-control baseline: it keeps the model interface close to sequence prediction while reducing one-step drift via overlap execution. It is a strong second step after plain next-token baselines when continuous-control smoothness matters.
