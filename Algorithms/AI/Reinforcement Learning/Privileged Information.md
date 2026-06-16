---
title: Privileged Information
aliases:
  - Privileged State
  - Privileged Observations
  - Training-Time Privileged Signals
tags:
  - reinforcement-learning
  - sim-to-real
  - actor-critic
  - policy-learning
---

# Privileged Information

**Privileged information** is supervision or state data available during training but not available during deployment.

In RL, this often appears as full simulator ground truth provided to critics, planners, or teachers, while actors and deployed policies only see partial, noisy observations.

---

## Why it helps

- Improves critic training by lowering variance in value estimates.
- Enables auxiliary losses based on unobservable simulation variables.
- Supports distillation pipelines where the student learns deployable behavior from a privileged teacher.

Common sources:
- contact states
- exact object poses
- true dynamics parameters
- deterministic collision flags

---

## Common mistakes

- Leaking privileged channels into actor inputs.
- Letting evaluation accidentally include privileged data.
- Treating privileged features as robust features instead of training-time aids.

---

## Related terms

- **Observable state**: what the final deployment can access.
- **Hidden state**: model-internal latent or filtered estimate of unobserved variables.
- **Belief state**: probability distribution over possible states for partial observability.

---

## Comparison table

| Design Choice | Training Signal | Deployment Input | Transfer Risk | Use Case |
|---|---|---|---|---|
| Fully symmetric | Same state for actor and critic | Same as training | Low | Simple control tasks with matching sim/real sensing |
| Fully partial | Same limited observation | Same limited observation | Low | Baseline and fairness in ablations |
| Privileged asymmetric | Rich simulator state for critic/teacher | Partial/realistic for actor | Medium | Sim-to-real robotics |
| Latent-state methods | Learned latent | Learned latent | Medium-High | Complex vision-based tasks |

---

## Practical checklist

1. Define strict actor input contract (`o_deploy`).
2. Keep privileged variables isolated behind an explicit training-only boundary.
3. Run ablations where privileged channels are removed at test time.
4. Distill to student/actor and revalidate under deployment noise.

---

## Related notes

- [[Asymmetric Actor Critic]]
- [[POMDP]]
- [[Belief State]]
- [[Teacher Student Distillation]]
- [[Sim2Real]]
