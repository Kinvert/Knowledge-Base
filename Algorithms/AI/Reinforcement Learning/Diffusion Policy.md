# Diffusion Policy

`Diffusion Policy` is an imitation-learning style policy family that generates action trajectories by denoising, instead of directly regressing one-step actions or doing one-step next-token prediction.

---

## Overview

- Paper: `Diffusion Policy` (`arXiv:2303.04137`, 2023).
- Core idea: model action trajectories as a denoising process conditioned on observations (vision/proprioception), usually in chunks.
- Why it became popular: strong performance in manipulation where there are many valid action trajectories for the same task.
- Important fit note: it is not a pure next-token policy by default; it is a generative diffusion decoder wrapped into a policy.

---

## Core mechanism

1. Collect demonstration trajectories and align observation-action windows.
2. Add noise progressively to action chunks (forward diffusion).
3. Train a network to reverse the process conditioned on current observation context.
4. At inference, sample a denoised action chunk from random noise and execute its first part (receding horizon).

The key difference from tokenized transformers is sampling cost and generation loop structure.

---

## Why this sits as a partial-fit for tokenized-next-token planning

Use this when:
- your tasks are multi-modal (many valid grasps, approach strategies),
- smoothness is more important than ultra-fast one-step token decoding.

Avoid as default if:
- your stack is strictly next-token and you want a minimal architectural swap,
- low-latency one-token/short-latency response is non-negotiable.

---

## Comparison chart

| Method | Objective | Action output style | Inference style | Best scenario | Main cost |
|---|---|---|---|---|---|
| **Diffusion Policy** | denoising trajectory generation | action chunks or trajectories | iterative reverse diffusion | contact-rich manipulation with multi-modality | high compute/inference latency |
| **Decision Transformer** | next-token action prediction | token/action next-step | autoregressive | low-latency token stack baseline | simpler training/inference than diffusion |
| **Trajectory Transformer** | sequence token rollout | `s,a,r` token trajectories | autoregressive + beam/rollout | planning-style offline sequence control | decode latency from search |
| **Behavior Transformer / VQ-BeT** | action token generation | quantized action tokens | autoregressive | behavior cloning and mode coverage in demos | codebook + quantization tuning |
| **ACT / ALOHA** | chunk prediction with overlap replanning | chunk tokens/actions | chunk decode + execute prefix | continuous real-robot manipulation where jitter is problematic | runtime scheduling + chunk tuning |
| **FAST** | action-tokenizer improvement | tokenized frequency coefficients | token prediction with transformed code space | high-frequency smooth control tasks | tokenizer/codec overhead |
| **TD-MPC / Dreamer** | model-based planning or world-model imagination | latent action plans | online planning with value/world model | hard long-horizon tasks | much larger system complexity |

---

## Pros

- Handles multi-modal action distributions naturally.
- Produces smooth action sequences in vision-conditioned tasks.
- Often strong transfer in household/manipulation settings with rich demos.
- Can express richer futures than single-step BC.

## Cons

- Slow inference due to multiple denoise steps.
- Higher implementation complexity than one-step token methods.
- Harder to fit into strict “predict one token” latency budgets.
- Needs careful rollout scheduling and safety filtering for real robots.

---

## Practical implementation notes

- Start with a chunk size that your control loop can tolerate (`H` too long increases jitter sensitivity).
- Tune denoising steps aggressively for hardware deployment; early-stop sampling is often necessary.
- Keep observation normalization locked (especially image preprocessing and action scales).
- Combine with hard filters:
  - action clipping and collision checks,
  - rate limits and watchdog stop,
  - recovery policy for failed denoise/sample events.

Minimal workflow:

```text
collect demos D = {(obs_t, act_t:t+H)}
train denoiser with conditional observation embeddings
at rollout:
  sample latent/noisy token or action chunk
  run reverse denoising steps
  execute first action(s) of decoded chunk
  reobserve and repeat
```

---

## External references

- Paper: https://arxiv.org/abs/2303.04137
- Project page: https://diffusion-policy.cs.columbia.edu/

---

## Related notes

- [[ACT Action Chunking Transformer]]
- [[Behavior Transformer]]
- [[Trajectory Transformer]]
- [[Decision Transformer]]
- [[FAST]]
- [[TD-MPC]]
- [[Dreamer]]

---

## Summary

`Diffusion Policy` is a strong option when multimodality and smooth visuomotor behavior matter more than minimal-token architecture. It is usually a partial-fit for the “next-token-only” roadmap because inference is iterative and compute-heavy compared with transformer token decoders.
