# FAST

**FAST** in this note means **F**requency-space **A**ction **S**equence **T**okenization as introduced by `Pertsch et al.`, and should be treated as an action-tokenizer module, not a complete control policy.

---

## Overview

- Paper: `FAST: Efficient Action Tokenization for Vision-Language-Action Models` (`arXiv:2501.09747`, 2025).
- Core goal: reduce redundant token load in high-rate action streams before sequence modeling.
- Mechanism: compress action chunks in frequency domain (per-dimension temporal coefficients), then discretize and model tokens.
- Why it exists:
  - per-step per-axis binning often creates huge token sequences with strong local correlation,
  - next-token models then spend capacity reconstructing smoothness instead of behavior.

In short: FAST is often a better front-end for policies that already have a token predictor.

---

## Core mechanism

1. Split continuous control into fixed-length chunks.
2. Normalize each action dimension consistently.
3. Apply a time-domain transform per action dimension (for example, DCT-style transform) and keep top coefficients.
4. Quantize coefficient sets into tokens.
5. During inference, predict next coefficient tokens and decode back to action time series.

Important separation:
- `FAST` changes *how actions are represented*.
- your policy model (`Decision Transformer`, `Behavior Transformer`, VLM/VLA backbone, etc.) still produces the decoded tokens.

---

## Practical meaning for tokenized-next-token RL

- This is the first thing to try if your biggest issue is noisy next-action prediction at high frequency.
- It is especially useful when action tokens are dominated by small per-step deltas.
- It does not replace reward/goal conditioning, planning, or safety policy layers.
- It combines well with chunked policies (`ACT`) and token-only behavior models (`BeT / VQ-BeT`).

---

## Comparison chart

| Method | Representation | Decoder role | Autoregressive compatibility | Sequence quality benefit | Main downside |
|---|---|---|---|---|---|
| **FAST (action tokenizer)** | DCT-frequency coefficients + quantization | inverse transform + denormalize + reshape | high (drop-in tokenization) | high compression for smooth/high-frequency actions | extra tokenizer tuning/metadata bookkeeping |
| **Naive per-step axis binning** | per-axis per-timepoint bins | direct decode to continuous | high | low; high redundancy | large token count and weak long-horizon structure |
| **VQ-VAE behavior tokenizers** | learned codebook over action snippets | learned codebook decoder | medium | compact semantic codes | extra reconstruction training, sensitive codebook collapse |
| **BeT / VQ-BeT tokenizers** | action k-means or residual VQ codes | decode residual/action codes | medium | good for multimodal behavior manifolds | quantizer tuning and reconstruction drift |
| **Diffusion policy action stack** | continuous trajectory latent/chunk | iterative denoise decoder | low (not token-first by default) | smooth trajectories without explicit symbolic codes | slower inference and extra denoise steps |
| **Decision/Trajectory-style discretization** | fixed discretization across state/action dims | action projection | high | straightforward integration | often less efficient for smooth high-rate control |

---

## Pros

- Reduces redundant tokens for high-frequency movement.
- Preserves smoothness priors directly in representation.
- Minimal change for teams already using token predictors.
- Can improve sample efficiency in imitation-heavy pipelines.

## Cons

- It is only a tokenizer, not a complete policy design.
- Compression choices can erase short high-frequency impulses.
- Wrong quantization metadata (chunk length, dimension order, scales) breaks decoding silently.
- Less useful for low-rate or already sparse action streams.

---

## Implementation notes

- Keep chunk duration and normalization fixed and stored with the checkpoint.
- Track three metadata fields with every sample:
  - chunk horizon `H`,
  - action dimension `D`,
  - coefficient count kept per channel.
- Sweep coefficient truncation:
  - too low hurts fine movements,
  - too high returns token explosion.
- Always validate **reconstruction error** and **control rollout error** separately.
- Use safety clamps after decode before sending to hardware.

Example flow:

```text
raw_actions = collect_actions(window=t:t+H)
chunk_codes = FAST_tokenizer.encode(raw_actions)
pred_tokens = policy(next_token_model, chunk_codes)
pred_chunk = FAST_tokenizer.decode(pred_tokens)
apply_safety_filters(pred_chunk[0])
execute(pred_chunk[0])
```

---

## Related notes to keep in the roadmap

- [[Decision Transformer]]
- [[Trajectory Transformer]]
- [[Behavior Transformer]]
- [[ACT Action Chunking Transformer]]
- [[Diffusion Policy]]

---

## External references

- ArXiv: https://arxiv.org/abs/2501.09747
- PI/FAST download page: https://www.physicalintelligence.company/download/fast.pdf
- Hugging Face FAST card: https://huggingface.co/physical-intelligence/fast

---

## Summary

`FAST` is a practical and often high-leverage way to make token-based policies less brittle in high-rate control: compress action chunks in a structured domain first, then let your policy learn on the compressed token stream.
