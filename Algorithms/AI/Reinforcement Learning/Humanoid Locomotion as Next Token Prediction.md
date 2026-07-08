# Humanoid Locomotion as Next Token Prediction

`Humanoid Locomotion as Next Token Prediction` is an ICML/NeurIPS 2024 style sequence-modeling approach that treats real-world humanoid walking control as a language-model-like autoregressive task: predict the next sensorimotor token from history.

---

## Overview

- Paper: `Humanoid Locomotion as Next Token Prediction` (`arXiv:2402.19469`)  
- Appears in NeurIPS 2024 main conference track and later project release.
- Core claim: with enough sensorimotor trajectory data and robust tokenization, a causal transformer can learn a generalist humanoid locomotion policy by next-token prediction.

From the authors:
- Model is trained autoregressively on mixed trajectories.
- Training supports data with missing modalities (for example, human video trajectories with no action stream).
- They report real-world zero-shot transfer and urban walking generalization from relatively limited pure-action-labeled locomotion data.

---

## Core mechanism

1. Tokenize multimodal humanoid time-series data per step (sensory + action channels).
2. Train a causal transformer to predict shifted tokens (next-token prediction on the full multimodal stream).
3. Use a **modality-aligned objective**:
   - predict state token from history of each modality,
   - predict action token from action-related input context.
4. At inference, run closed-loop rollouts from recent token history and execute predicted first action(s).

The paper uses data from multiple sources:
- prior policies and controllers,
- model-based controller traces,
- motion-capture traces,
- human demonstrations from video-derived trajectories.

This mixture lets the model learn shared structure beyond a single source policy.

---

## Why this is not just “another RL policy”

Compared with value-based or actor-critic RL:
- objective is generative token modeling of full trajectories,
- policy emerges through sequence modeling rather than explicit maximization of Bellman backup,
- strong emphasis on sequence representation and multimodal token alignment,
- inference uses autoregressive rollout behavior from learned sensorimotor priors.

The approach is close to your tokenized-next-token idea, but it pushes farther than action-only token models by modeling state and action streams together.

---

## Comparison chart

| Method | Objective | Data requirement | Tokenization style | Strength | Weakness | Notes |
|---|---|---|---|---|---|---|
| **Humanoid Locomotion as Next Token Prediction** | autoregressive sequence prediction of full sensorimotor tokens | mixed complete and incomplete trajectories (policy traces, controller traces, motion capture, video) | multimodal tokens (state + action tokens) | strong zero-shot transfer and urban robustness from token modeling + mixed data | long-horizon stability depends on context/training scale | best fit when data is multimodal and noisy |
| **RT-1 / RT-1-X** | language-conditioned action token policy | language + vision datasets for manipulation tasks | task/action tokenization + vision-text conditioning | strong closed-loop manipulation transfer | not tuned for pure full-body locomotion and dynamics-heavy gait contexts | different modality focus (manipulation) |
| **Decision Transformer** | return-conditioned action token sequence | offline rewarded trajectories | state/action/return token windows | very simple supervised-form control baseline | weaker for raw multimodal high-dimensional robot gait streams | strong conceptual fit but usually narrower than this work |
| **Trajectory Transformer** | autoregressive sequence over state/action/reward | offline trajectories | tokenized state/action trajectories + beam/return-aware decoding | planning-style trajectory generation and flexible horizon behavior | heavier inference and token discretization tuning | shares “sequence-first” ethos |
| **Behavior Transformer / VQ-BeT** | behavior token autoregression | demo-only trajectories | learned action tokenization + residual/latent decode | robust multi-modal behavior capture in imitation settings | not inherently cross-modality missing-data modeling | no explicit missing-modality masking across modality streams |
| **LocoGPT (related)** | GPT-style policy transformer across humanoids | humanoid locomotion datasets | structured action tokenization | improves multi-task transfer and gait stability in comparative humanoid settings | different training objective and reported scope | relevant competitor if you stay in full-body locomotion only |
| **Diffusion Policy** | denoising action trajectories | demonstrations | continuous/action trajectory sampling | smooth trajectories, rich multi-modality | slower iterative sampling; not strictly token-first | good fallback when tokenization is hard |

---

## Pros

- Strong demonstration of one-stack tokenized control for high-dimensional physical systems.
- Handles partially missing modalities by masking/preserving unified sequence format.
- Fits real deployment constraints: closed-loop execution of generated tokens.
- Useful benchmark for testing whether your next-token stack can move from manipulation to whole-body locomotion.

## Cons

- Humanoid locomotion is hardware- and dynamics-sensitive; deployment risk is higher than simpler manipulation.
- Token design choices become critical (`granularity`, context window, modality alignment).
- Inference and memory cost can be high for long horizons.
- It is not a drop-in replacement for token prediction models that assume clean reward-conditioned control loops.

---

## Practical implementation notes

If you are building this style of policy:

- Start with a strict multimodal schema:
  - `[obs_1, obs_2, ..., act_1, act_2 ...]` with consistent clipping/normalization.
- Keep modality-aware token masks so incomplete trajectories can still enter the same batch.
- Run scaling studies on:
  - context length,
  - trajectory chunk length,
  - token bins per modality.
- Add runtime safety gates (joint limits, tilt/torque thresholds), regardless of model quality.

Pseudo-flow:

```text
collect trajectories from:
  - simulator policies + controllers
  - mocap + human videos (for supplementary structure)
tokenize all streams into modality-aware tokens
train causal transformer next-token objective (with masked losses on missing modalities)
deploy:
  stream current sequence window
  predict next action token
  execute first action
  append actual state, repeat
```

---

## Related notes to create next

This paper introduces a useful “locomotion tokenization stress test” between:
- `LocoGPT`,
- `RT-2-X` / `OpenVLA`,
- and other sequence-first robotics baselines.

---

## External references

- [Project page](https://humanoid-next-token-prediction.github.io/)
- [ArXiv abstract](https://arxiv.org/abs/2402.19469)
- [NeurIPS listing](https://proceedings.neurips.cc/paper_files/paper/2024/hash/90afd20dc776bc8849c31d61a0763a0b-Abstract-Conference.html)
- [LocoGPT example](https://www.researchgate.net/publication/400560069_LocoGPT_GPT-Based_Multi-Humanoid-Task_Policy_for_Humanoid_Locomotion)

---

## Summary

Humanoid Locomotion as Next Token Prediction is a good example of pushing tokenized control beyond manipulation into full-body mobility: a single next-token transformer over multimodal trajectories plus modality masking enables zero-shot real-world transfer and broad generalization signals. It is powerful, but also a high-complexity, hardware-sensitive variant of the tokenized-control idea.
