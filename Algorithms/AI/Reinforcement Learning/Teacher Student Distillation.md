---
title: Teacher Student Distillation
aliases:
  - Teacher-Student Distillation
  - Student Distillation
  - Policy Distillation
tags:
  - reinforcement-learning
  - robotics
  - sim-to-real
  - policy-learning
---

# Teacher Student Distillation

**Teacher-student distillation** trains a deployable student policy from a stronger teacher, often under reduced input channels, lower latency, or smaller compute budget.

It is common in robotics workflows where simulation can support a much larger teacher, while hardware limits the final policy footprint.

---

## Typical pattern

- **Teacher**: Access to richer supervision (larger model, privileged observations, stronger planning).
- **Student**: Constrained architecture and observations for deployment.
- **Distillation loss**: Match teacher behavior (actions, logits, or feature representation).

This is a natural pairing with [[Asymmetric Actor Critic]], where privileged simulation signals are intentionally removed before student deployment.

---

## Relationship to other methods

- Different from **imitation learning**: both can use demonstrations, but teacher-student distillation uses a trained teacher as teacher signal.
- Different from simple **behavior cloning**: BC uses human/expert data directly.
- Related to **model compression**: both reduce runtime complexity.

---

## Comparison table

| Method | Supervision Source | Student Constraint | Deployment Alignment | Typical Benefit |
|---|---|---|---|---|
| Behavior Cloning | Expert demonstrations | Optional | High | Easier setup with high-quality data |
| Teacher Student Distillation | Teacher model output | Explicit | Very High | Keep policy quality while shrinking model |
| Asymmetric AC + Privileged Critic | Privileged critic signals | Optional | Very High | Better training stability + transfer |
| Offline RL | Offline dataset | Moderate | Medium | Safer offline learning loop |
| Fine-tuning | Labeled/new environment data | Optional | Medium | Adapts existing policy to new domain |

---

## Practical guidance

1. Keep teacher checkpoints stable before distillation.
2. Use deployment-like observation normalization and preprocessing in student pipeline.
3. Train on diverse states to avoid student overfitting to narrow regions.
4. Evaluate student safety/robustness under domain and sensor noise shifts.

---

## Pros / Cons

### ✅ Pros
- Smaller runtime and lower energy envelope.
- Keeps much of teacher skill when carefully tuned.
- Fits naturally with sim-to-real staged pipelines.

### ⚠️ Cons
- Can transfer teacher bias and blind spots.
- Requires high-quality teacher outputs and careful hyperparameters.
- Distilled behavior may fail when deployment drift exceeds distillation training distribution.

---

## Related notes

- [[Knowledge Distillation]]
- [[Asymmetric Actor Critic]]
- [[Privileged Information]]
- [[Imitation Learning]]
- [[Sim2Real]]
