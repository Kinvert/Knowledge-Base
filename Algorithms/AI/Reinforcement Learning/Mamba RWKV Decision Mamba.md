# Mamba, RWKV, and Decision Mamba

This family of approaches keeps a tokenized-sequence RL idea but swaps the **sequence backbone** from attention-heavy Transformers to efficient recurrent or state-space variants.

---

## Overview

- Mamba (`arXiv:2312.00752`) is a selective state-space model architecture for sequence modeling.
- RWKV (`arXiv:2305.13048`) is a linear-attention RNN that can run like a parallelized Transformer during training and like an RNN at inference.
- Decision Mamba (`arXiv:2406.00079`) applies that sequence-modeling shift directly to tokenized trajectory control workflows (typically in Decision Transformer-style pipelines).

This gives you the same "next token" abstraction (especially for action token streams), but with different compute and memory behavior.

---

## Core idea in tokenized RL pipelines

For tokenized observation/action/control stacks, these methods replace the policy backbone:

1. Observation/action/reward history is encoded as a token sequence.
2. A sequence model predicts the next action tokens (or action-code chunks).
3. You decode and execute with the same control/runtime wrapper you already use.

The change is usually at the model block level:

- Keep tokenizer + dataset + rollout loop.
- Swap transformer decoder blocks for Mamba-style selective SSM blocks or RWKV blocks.
- Optionally preserve some transformer-like local context if needed (Decision Mamba-style hybrid setups).

---

## Mamba in this context

- Mamba is a **state-space approach** (`selective scan`) built around input-dependent state transitions.
- It targets near-linear scaling in sequence length, especially for long contexts.
- It is strongest where long-horizon credit and temporal patterns are important and sequence length is a bottleneck.
- It is commonly used with continuous-time embeddings, with actions and states discretized or quantized for token-form inputs.

Practical consequence:
- Same tokenized control framing as Decision/Trajectory-style methods.
- Better candidate when long-window attention is too expensive.
- More aggressive tuning needed for short-horizon low-latency tasks where attention can already fit.

---

## RWKV in this context

- RWKV uses a recurrent mechanism with a linear-attention formulation.
- It trains with mostly parallelizable structure but can run with constant per-token inference complexity in recurrent mode.
- It is useful if you want inference latency and memory guarantees similar to RNN-like systems, but still want transformer-like quality for sequence prediction.

Practical consequence:
- Good for real-time robotics loops that repeatedly condition on fresh token prefixes.
- Useful as a drop-in replacement candidate in experiments where sequence horizon is moderate-to-large.

---

## Decision Mamba

Decision Mamba is the direct bridge piece for your current roadmap:

- Starts from Decision Transformer-style supervised sequence modeling.
- Replaces or augments Transformer blocks with Mamba blocks.
- Can be used as:
  - **pure Mamba replacement** (end-to-end selective SSM policy),
  - or **hybrid** setup where Mamba gives long-horizon context and local transformer blocks help short-term correction/rollout.

Useful when you:
- already run tokenized-policy experiments,
- need longer context than standard transformer budgets,
- want to keep dataset/decoder logic while changing sequence core.

---

## 📊 Comparison chart

| Method | Sequence core | Objective view | Scaling behavior | Inference profile | Best fit for tokenized RL |
|---|---|---|---|---|---|
| **Decision Transformer** | Transformer blocks | Next-token action prediction conditioned on return/state/history | Quadratic attention in sequence length | One-pass autoregressive | Strong baseline for return-conditioned control |
| **Trajectory Transformer** | Transformer + beam decoding | Joint state/action/reward token modeling | Quadratic attention during planning steps | Beam-search rollout | Strong when explicit trajectory decoding search matters |
| **Mamba** | Selective state-space (SSM) blocks | Next-token modeling over trajectory stream | Near-linear time for long sequences | Recurrent/stateful-friendly | Strong when long contexts dominate compute cost |
| **RWKV** | Receptance-weighted linear recurrent core | Next-token language-style policy modeling | Parallelizable training + linear/constant inference growth | Constant memory-like behavior during generation | Real-time token control where per-step budget is tight |
| **Decision Mamba** | Mamba (and optionally transformer-hybrid head) in DT layout | Same as DT but with selective SSM core | Longer effective context than pure transformer in similar budgets | Recurrent-friendly if streaming | Best match for your "tokenize then predict next token" agenda |
| **VQ-BeT / Behavior Transformer** | GPT-style with action quantization | Action-code autoregression | Standard transformer scaling | One-step/short-context autoregressive | Better when multimodal continuous-action mode coverage is the dominant issue |

---

## ✅ Pros

- Keeps your existing tokenized control framing mostly intact.
- Can reduce long-sequence compute pressure.
- Handles long trajectories with cleaner memory growth patterns than full attention stacks.
- Inference can be easier to stream because states can be carried across time steps.
- Useful research baseline before adding richer planner/value stacks.

---

## ❌ Cons

- Not a free lunch: you still need a stable tokenization and reward/goal conditioning setup.
- Ecosystem tooling is less mature than plain transformer baselines.
- Hyperparameters are still sensitive (state size, state-space width, chunking/segmentation choices).
- Long-context wins are not guaranteed on short horizon, noisy, or highly discontinuous tasks.
- Token reconstruction and safety fallback layers remain separate concerns.

---

## Implementation notes

If you are writing this in practice:

- Start by cloning the existing Decision Transformer trainer and replacing backbone only.
- Keep tokenizer deterministic:
  - fixed token vocabulary per sensor/action stream,
  - fixed chunk shape,
  - identical state normalization at train/infer.
- Add a recurrent-state reset policy:
  - clear per-episode or when task resets,
  - keep context state for mid-episode continuation.
- Compare:
  - 1k/10k-step horizon behavior,
  - rollout success,
  - wall-clock per control step,
  - failure modes near out-of-distribution transitions.

Minimal planning loop:

```text
for each trajectory in offline dataset:
  tokenize states/actions/rewards into sequence
  train:
    input tokens -> predict next action token(s)

deploy:
  set tokenizer+observation stream
  carry recurrent state across steps
  at each tick: predict next action tokens -> execute -> update context
```

---

## New related topics found while writing

These are strong follow-ups to add to the backlog later if you want stronger breadth:

- `Mamba2` / improved selective SSM variants.
- `Jamba` (hybrid transformer-SSM family in the broader SSM space).
- `Mamba-2D / VSS Mamba` for spatial-temporal vision-like streams.

---

## 🌐 External resources

- [Mamba arXiv](https://arxiv.org/abs/2312.00752)
- [RWKV arXiv](https://arxiv.org/abs/2305.13048)
- [Decision Mamba arXiv](https://arxiv.org/abs/2406.00079)
- [state-spaces/mamba GitHub](https://github.com/state-spaces/mamba)
- [RWKV GitHub](https://github.com/BlinkDL/RWKV-LM)
- [Hugging Face: Mamba docs](https://huggingface.co/docs/transformers/main/model_doc/mamba)
- [Hugging Face: RWKV docs](https://huggingface.co/docs/transformers/main/model_doc/rwkv)

---

## 🔗 Related notes

- [[Decision Transformer]]
- [[Trajectory Transformer]]
- [[Behavior Transformer]]
- [[ACT Action Chunking Transformer]]
- [[FAST]]
- [[Diffusion Policy]]
- [[Imitation Learning]]
- [[Offline RL for Robotics]]

---

## Summary

If your roadmap is “tokenize observations and actions, then predict next token,” this is the most practical next family to add.  
It changes compute geometry without forcing a new training contract, so you can test token compression, action heads, and rollout quality with one controlled architecture swap.
