# Behavior Transformer

`Behavior Transformer` and `VQ-BeT` are tokenized behavior-cloning approaches that model robot control as sequence prediction over demonstrations, then decode actions into continuous commands through a quantization pipeline.

---

## Overview

- BeT paper: `Behavior Transformers: Cloning k modes with one stone` (`arXiv:2206.11251`, NeurIPS 2022).
- VQ-BeT paper: `Behavior Generation with Latent Actions` (`arXiv:2403.03181`, ICML 2024).
- Core idea: convert demonstrations into a token stream and let a causal transformer predict next behavior steps.
- Difference:
  - BeT uses action tokens built with `k`-means + residual residual correction.
  - VQ-BeT adds a learned Residual VQ front-end for richer action codebooks before sequence modeling.
- These methods are behavior-cloning oriented; reward modeling is not the first-class objective.

---

## Core mechanism

1. Collect demonstration trajectories `(o_t, a_t, g_t?)`.
2. Build a consistent action codec:
   - BeT: quantize actions with centroid codebook, retain residual.
   - VQ-BeT: train residual vector quantizer, then encode actions as discrete code sequences.
3. Train a causal transformer on token sequences with next-token loss.
4. Decode predicted action tokens back to continuous commands for robot execution.

VQ-BeT helps when action distributions are highly multi-modal or high dimensional because the tokenizer is learned, not fixed.

---

## Why this is close to your tokenized-next-token workflow

- Both methods avoid hand-building reward heads and policy-value decomposition.
- They operate where action space is your bottleneck (continuous actions become discrete tokens).
- They are strongest when demonstrations are rich in strategy variation, not when reward is clean and dense.

Practical implication:
- Good baseline for “next-token control” experiments where you need to compare **raw continuous BC vs tokenized BC** before adding model-based layers.

---

## Comparison chart

| Method | Objective | Action modeling | Data requirements | Decoder style | Best use | Main weakness |
|---|---|---|---|---|---|---|
| **Behavior Transformer (BeT)** | next behavior-token prediction | k-means action code + residual | multimodal behavior demonstrations | direct autoregressive generation | low/medium-dim action spaces, compact datasets | codebook sensitivity to quantization choices |
| **VQ-BeT** | next latent action-code prediction | learned Residual VQ tokenizer + transformer | larger, high-dimensional behavior datasets | token-sequence decoding + VQ decode | higher fidelity in complex action manifolds | extra training stage, more tuning |
| **Behavior Cloning (MLP/BC)** | state→action regression | continuous direct action | expert demos | single-step action output | quick baseline | poor multi-modality and compounding drift |
| **ACT / ALOHA** | chunk-level action prediction | latent action chunks | teleop datasets with temporal structure | chunk decode + overlap-replan | contact-rich real-robot skills | extra runtime schedule complexity |
| **Decision Transformer** | return-conditioned token policy | `R, s, a` token stream | offline trajectories with rewards | return-conditioned rollout | reward-aware offline control | return-to-go noise sensitivity |
| **Diffusion Policy** | denoising action trajectories | trajectory/action latents | imitation-heavy data | iterative sampling | very smooth multi-modal actions | slower decoding per step |
| **TD-MPC / Dreamer** | model-based planning objective | latent dynamics + policy/value | larger offline/online data | explicit dynamics/policy loops | strong long-horizon planning | far beyond single-token model swap |

---

## Pros

- Strong match to imitation-first data pipelines with no reward labels.
- Better multi-modal action handling than direct regression.
- Small stack shift if your system already supports token streams.
- VQ-BeT often wins in high-dimensional action spaces.

## Cons

- Quantization quality dominates quality; bad codebooks fail hard.
- Decoder drift in long horizons without replanning.
- Not inherently reward-aware; cannot replace reward-conditioned methods directly.
- VQ training adds a front-end complexity layer and additional evaluation criteria.

---

## Practical implementation notes

- Lock codec version:
  - save codebook checkpoints,
  - version action encoder/decoder separately from transformer weights.
- Start with BeT-style fixed centroid quantization first to check pipeline.
- Move to VQ-BeT only when you can justify added training overhead.
- Keep rolling context bounded (especially for high-rate robots) to avoid token growth.
- Add post-decoder guards:
  - velocity/torque/pwm caps,
  - collision-rate limits,
  - watchdog stop conditions.

Minimal pseudo-flow:

```text
collect demonstrations D = {(obs_t, act_t, goal_t?)}
train action encoder (BeT: k-means | VQ-BeT: residual VQ)
tokenize demonstration trajectories
train causal transformer for next-token prediction
deploy:
  encode current history
  predict next action token/code
  decode to continuous command
  filter/safety-check
  execute and repeat
```

---

## External resources

- BeT paper: https://arxiv.org/abs/2206.11251
- VQ-BeT paper: https://arxiv.org/abs/2403.03181
- VQ-BeT official repo: https://github.com/jayLEE0301/vq_bet_official
- Goal-conditioned extension: https://arxiv.org/abs/2210.10047
- Related comparison entry: https://github.com/jannerm/trajectory-transformer

---

## Related notes

- [[Decision Transformer]]
- [[Trajectory Transformer]]
- [[ACT Action Chunking Transformer]]
- [[FAST]]
- [[Diffusion Policy]]
- [[RT-2]]
- [[OpenVLA]]

---

## Summary

For tokenized robotics control, `Behavior Transformer` is the core BC-style baseline; `VQ-BeT` is the stronger, learned-token variant when action distributions are complex. Use BeT when you want a fast baseline, and use VQ-BeT when discretization quality is your main bottleneck.
