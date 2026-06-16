---
title: State Representation Learning
aliases:
  - Representation Learning
  - SRL
tags:
  - machine-learning
  - reinforcement-learning
  - embeddings
  - control
---

# State Representation Learning

**State representation learning** learns a compact latent state `z` from observations `o` so a policy can reason about dynamics and rewards without depending directly on raw high-dimensional inputs.

This is especially useful when raw observations are noisy, high-dimensional, or partially observed.

---

## Core goal

Transform observations into latent states that preserve task-relevant information:
- Markovianity-like behavior (future prediction)
- Control-relevant features
- Invariance to nuisance factors

---

## Common families

- **Autoencoder-based**: train compression and reconstruction objectives.
- **Contrastive methods**: pull together temporally or semantically related states.
- **Predictive models**: optimize next-state prediction or dynamics consistency.
- **Causal/informational methods**: prioritize intervention sensitivity and action-relevant factors.
- **World models**: combine latent state with dynamics for imagined rollouts.

---

## Comparison table

| Method | Objective | Interpretation | Compute Cost | RL Benefit |
|---|---|---|---|---|
| Autoencoder | Reconstruction | Medium | Low | Useful for denoising |
| Contrastive learning | Similarity objective | Medium | Medium | Better invariance with augmentations |
| Predictive coding | Next-step prediction | Medium | Medium-High | Supports planning/latent rollouts |
| Variational models | Probabilistic latent | Medium | High | Better uncertainty tracking |
| World models | Dynamics + representation | High | High | Strong for imagination-based control |

---

## In RL pipelines

- Combine with `Replay Buffers` for sample-efficient representation updates.
- Use in **POMDP** style tasks where observations are partial.
- Share encoder weights between policy and value networks when stable.
- Use representation regularization to avoid overfitting to simulator artifacts.

---

## Failure modes

- Over-compression that removes critical task signals.
- Misaligned augmentations causing representation drift.
- Latent collapse with weak objectives.
- Poor evaluation if rollout quality is not tested under sensor noise.

---

## Related notes

- [[Observation Space]]
- [[POMDP]]
- [[Belief State]]
- [[World Models for Control]]
- [[Asymmetric Actor Critic]]
