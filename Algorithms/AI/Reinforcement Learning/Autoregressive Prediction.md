---
title: "Autoregressive Prediction"
aliases: [AR Prediction, Autoregressive Modeling]
tags:
  - reinforcement-learning
  - sequence-modeling
  - control
  - transformer
---

# Autoregressive Prediction

Autoregressive (AR) prediction is a modeling setup where the next token is predicted from the full past context.
In control and RL, those tokens can represent observations, actions, returns, or state markers.

---

## Core idea

For a token stream `x_1, x_2, ..., x_t`, the model learns:

```math
p(x_{t+1}
\mid x_1, x_2, ..., x_t)
```

At inference, you generate one step at a time and append each sampled token back into the context before the next step.

---

## Why this fits tokenized-control workflows

- Matches closed-loop control behavior: no future tokens are available at action time.
- Uses a single supervised-style objective: next-token prediction.
- Lets you reuse mature infrastructure from language model training.
- Works naturally with tokenizers over action space (quantized continuous actions, codebooks, bins).

In practice, many RL methods are AR in this sense with different token layouts and optional conditioning channels.

---

## What it is **not**

- Not a dedicated planner/MPC module unless you add explicit rollout or search logic.
- Not a full model-based RL method by itself.
- Not a guarantee of good long-horizon safety; it can drift with compounding rollout error.

---

## Typical control flow

1. Build a sequence format, usually a window over recent context.
2. Train with teacher forcing so the model predicts token `t+1` for each position.
3. Deploy in an online loop:
   - encode latest observation + recent history
   - decode next action token(s)
   - apply filters/safety checks
   - execute

```text
while episode_active:
    window = build_context(obs_history, action_history, optional_return_tokens)
    next_action_token = model.predict_next(window)
    action = decode_token(next_action_token)
    action = safety_filter(action)
    execute(action)
    append(obs, action)
```

---

## Comparison chart

| Method | Prediction target | Key modeling choice | Planning behavior | Compute profile | Good first use |
|---|---|---|---|---|---|
| **Autoregressive Prediction** | Next token in history | causal factorization | implicit rollout only | low-medium | baseline policy model for token streams |
| **Causal Transformer** | Next token with strict attention mask | decoder-only transformer | implicit rollout | low-medium | when you need explicit causal-attention mechanics |
| **Decision Transformer** | Next action in `(R, s, a)` sequence | return-conditioned trajectory tokens | implicit rollout with return target | medium | offline-first control where return conditioning helps |
| **Trajectory Transformer** | Next state/action/reward token(s) | sequence model over trajectories | longer-horizon rollout via decode strategy | medium-high | planning-like behavior from trajectory priors |
| **Behavior Transformer / VQ-BeT** | Next behavior/action code token | behavior tokenizer + AR head | local autoregressive action generation | medium | imitation-heavy and high-frequency control |
| **ACT / ALOHA** | Action chunks and latent behavior tokens | chunked action decoding | periodic receding-horizon behavior | medium | stable multi-step action emission |
| **FAST** | Same model, action token codec swap | learned action tokenizer | AR decoding after codec compression | low-medium | if raw action tokenization is your bottleneck |
| **Diffusion Policy** | Denoised next action trajectory | iterative denoising not pure AR | explicit iterative refinement | high | smoother multi-modal action posteriors |
| **TD-MPC / Dreamer** | Latent next states + value/cost | model-based loop with value learning | explicit planning/search | high | tasks where rollout lookahead matters |

---

## Common failure modes

- **Exposure bias**: training uses true context, deployment uses model-generated context.
- **Compounding errors**: one bad action can knock the agent into unseen states.
- **Tokenization mismatch**: a codec difference between train and deploy creates unstable behavior.
- **Rate/collision latency**: one token per step can be slower than direct actor output.

Mitigations:

- Keep context windows short at first, then scale with confidence.
- Add recovery or recovery-action tokens to teach safe rollbacks.
- Use explicit action clamps, speed/rate limits, and watchdog overrides.
- Evaluate with rollout success and intervention rate, not only token loss/perplexity.

---

## Pros

- Clear, composable objective.
- Great prototyping baseline for sequence-to-policy experiments.
- Easy to compare with other AR/sequence-based control methods.
- Works well with CUDA training pipelines and modern tokenizers.

## Cons

- No explicit planning depth without extra components.
- Slowest option for very high-rate control if not optimized.
- Can become over-confident in out-of-distribution states.

---

## Related notes

- [[Causal Transformer]]
- [[Decision Transformer]]
- [[Trajectory Transformer]]
- [[Behavior Transformer]]
- [[ACT Action Chunking Transformer]]
- [[FAST]]
- [[Diffusion Policy]]
- [[TD-MPC]]
- [[Dreamer]]

---

## External resources

- https://arxiv.org/abs/1706.03762 (Transformer self-attention foundations)
- https://arxiv.org/abs/2106.01345 (Decision Transformer)
- https://arxiv.org/search/?query=Trajectory+Transformer+Reinforcement+Learning&searchtype=all
- https://huggingface.co/docs/transformers/en/tasks/language_modeling
