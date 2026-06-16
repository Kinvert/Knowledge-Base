---
title: Knowledge Distillation
aliases:
  - Model Distillation
  - Distillation
  - Teacher Student Learning
tags:
  - machine-learning
  - model-compression
  - training
---

# Knowledge Distillation

**Knowledge distillation** transfers behavior from a high-capacity **teacher** model to a smaller **student** model.

The student approximates the teacher’s output or internal structure so inference becomes cheaper, faster, or easier to deploy.

---

## Why distill at all

- Large teachers often have better task performance but are too slow for deployment.
- Students can preserve most performance with much smaller runtime cost.
- Distillation can be used for architecture compression or for adapting policies to weaker hardware.

---

## Common variants

- **Logit distillation**: Match softened teacher output probabilities.
- **Feature distillation**: Align intermediate representations.
- **Attention map distillation**: Align attention patterns in transformers.
- **Self-distillation**: Distill from ensemble or later checkpoints into smaller student.
- **Cross-domain distillation**: Transfer from sim-rich teacher to deployed-limited student.

---

## Comparison table

| Technique | Signal | Cost | Typical Setup | Deployment Fit | Risk |
|---|---|---|---|---|---|
| Model pruning | Weights | Low | Same architecture | Medium | Accuracy drops if aggressive |
| Quantization | Weights/activations | Low | Calibrate per target | Very High | Accuracy and calibration risk |
| Knowledge distillation | Teacher signals | Medium | Teacher + student | Very High | Requires careful temperature and matching |
| Fine-tuning | Labeled data | Medium-High | Supervised target dataset | High | Data and overfitting risk |
| Behavior cloning | Expert trajectories | Medium | Demonstrations | Medium | Covariate shift and compounding error |

---

## Workflow

1. Train or select a high-performance teacher.
2. Freeze teacher and define student architecture constraints.
3. Choose distillation loss (`KL`, L2 on logits/features, etc.).
4. Track student accuracy and robustness across deployment-like evaluation.
5. Iterate on temperature, loss weights, and data augmentations.

---

### When to prefer distillation

Use it when:
- compute budget is fixed and must shrink.
- latency budgets are strict.
- you need to move policies/models from research to embedded targets.

Avoid when:
- you need strong guarantees on exact teacher-level behavior.
- interpretability of student internals is required.
- teacher model is itself miscalibrated.

---

## Related notes

- [[Model Compression]]
- [[Offline RL for Robotics]]
- [[Teacher Student Distillation]]
- [[Distillation (machine learning)]]

