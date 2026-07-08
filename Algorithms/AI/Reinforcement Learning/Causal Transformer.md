# Causal Transformer

`Causal Transformer` means a Transformer whose self-attention is masked so each token can only attend to earlier tokens (no future context), which makes it a strict auto-regressive sequence model.

---

## Overview

- Core idea: enforce `t -> t-1` directionality during attention.
- Common form: decoder-only architecture (GPT-style), where the attention mask is triangular and blocks future positions.
- Why it matters:
  - supports next-token prediction,
  - matches how policies are sampled in robotics/control token streams,
  - prevents leakage from future tokens during training.
- Typical RL use: action/state token modeling where the policy emits one action token at a time from past context.

---

## Core mechanism

1. Build sequence of tokens (text, states, actions, or return/action/state tokens).
2. Use transformer blocks with causal attention mask:
   - token `i` can only attend to tokens `< i`.
3. Train with teacher forcing to predict the next token at each position.
4. At inference, generate sequentially:
   - emit token `t+1`,
   - append to sequence,
   - repeat.

This differs from bidirectional attention (full visibility) because causality gives you a proper rollout model.

---

## Comparison chart

| Method | Context access | Decoding | Best use | Strength | Weakness |
|---|---|---|---|---|---|
| **Causal Transformer** | past tokens only | one token at a time (autoregressive) | next-token policies, tokenized control, language modeling | correct for rollout tasks, easy sequence training objective | slower generation for long horizons |
| **BERT-style bidirectional Transformer** | full sequence context | masked-token filling (not true rollout) | representation learning, classification, embedding | richer context at each position | not suitable for online generation without extra decoding strategy |
| **Decision Transformer** | causal token order over `(R, s, a)` | one-step action generation | offline tokenized control baseline | strong return-conditioning and easy deployment | planning is implicit, not search-heavy |
| **Trajectory Transformer** | tokenized `s,a,r` sequence with rollout decoding | returns a full trajectory via decode strategy | planning-heavy offline RL | sequence-level planning behaviors | higher decode cost (beam/search) |
| **Behavior Transformer / VQ-BeT** | token/latent action streams | autoregressive behavior generation | imitation learning from demonstration | handles multi-modal action modes | no direct return optimization by default |

---

## How this applies to your tokenized-control workflow

- If your policy is trained to predict `next action` from history, the default stack is a causal transformer.
- For better sample quality in robotics:
  - tune token vocab and granularity,
  - use action-conditioning windows (image/state + past actions),
  - evaluate error propagation with closed-loop rollout metrics (not only token accuracy).

---

## Pros

- Matches real-time control assumptions (future unavailable).
- Simple objective: next-token cross-entropy / regression on tokenized action output.
- Strong transfer from LLM tooling and libraries.
- Easy to combine with trajectory/tokenization modules.

## Cons

- Sequential sampling increases inference latency.
- Long-horizon errors can compound if rollout horizon is long.
- Not a planning or value-learning engine by itself; it is a sequence policy backbone.
- May still need external safety and clipping at deployment.

---

## Practical notes

- Keep causal mask explicit in config:
  - `causal=True` / triangular mask,
  - cache keys/values for fast generation if latency is important.
- Start with small context windows, then scale.
- For continuous-control outputs:
  - use action tokenizers (`VQ-BeT`, `FAST`, quantization) before transformer head.
- Evaluate with:
  - rollout success,
  - first-failure step,
  - token perplexity only as secondary signal.

Minimal rollout pattern:

```text
while active:
  context = [history_state_tokens, action_tokens, optional return]
  a_t = model.next_token(context)
  execute(safety_filter(a_t))
  append(a_t) and continue
```

---

## External references

- Causal masking idea in attention: https://arxiv.org/abs/1706.03762
- GPT-style causal LM architecture: https://openai.com/research/language-unsupervised
- Hugging Face causal language modeling docs: https://huggingface.co/docs/transformers/tasks/language_modeling

---

## Related notes

- [[Transformer]]
- [[Decision Transformer]]
- [[Trajectory Transformer]]
- [[Behavior Transformer]]
- [[ACT Action Chunking Transformer]]
- [[FAST]]
