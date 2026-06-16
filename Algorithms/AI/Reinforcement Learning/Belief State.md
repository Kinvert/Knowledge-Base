---
title: Belief State
aliases:
  - Bayesian State
  - State Belief
  - Belief Distribution
tags:
  - reinforcement-learning
  - pomdp
  - state-estimation
---

# Belief State

In partial observability, a **belief state** is a probability distribution over possible true states after processing observations and actions.

It replaces unknown true state with uncertainty-aware statistics and is a central construct in [[POMDP]]-style planning.

---

## Why belief states matter

When the agent cannot see the full state, deterministic policies on raw observations can become brittle.

Belief states allow:
- maintaining uncertainty over unobserved variables
- updating with Bayes-like filtering
- planning under partial observability

---

## Update process

Given:
- previous belief `b(s_t)`
- action `a_t`
- new observation `o_{t+1}`

the new belief is computed by:
- transition prediction
- observation likelihood
- normalization

This is the same core logic behind many recursive filters and Bayesian estimators.

---

## Connection map

- [[State Estimation]] (practical Bayesian filtering family)
- [[Kalman Filter]] (Gaussian specializations)
- [[Particle Filter]] (nonlinear / non-Gaussian cases)
- [[POMDP]]

---

## Comparison table

| Approach | Representation | Computation | Accuracy in nonlinear systems | Hardware cost |
|---|---|---|---|---|
| Full state (MDP) | Known state vector | Low | N/A | Lowest |
| Belief state | Distribution over states | Medium | High | Medium |
| Recurrent policy | Learned hidden vector | Low-Medium | Variable | Low |
| Ensemble planning | Multiple sampled hypotheses | High | Very High | High |
| Deterministic fallback | Point estimate | Low | Low in uncertainty-heavy domains | Very Low |

---

## Practical caveat

Belief state tracking is not free:
- dimension explosion for large state spaces
- approximation errors can destabilize long horizon planning
- requires careful resampling/normalization strategy

For small robotics tasks, lightweight filters can provide most of the benefit.

---

## Related notes

- [[POMDP]]
- [[Privileged Information]]
- [[Observation Space]]
- [[State Representation Learning]]

