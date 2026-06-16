---
title: Asymmetric Actor-Critic
aliases:
  - Asymmetric Actor Critic
  - Asymmetric AC
  - Asymmetric Actor Critic (RL)
  - Asymmetric Advantage Actor-Critic
---
tags:
  - reinforcement-learning
  - actor-critic
  - sim-to-real
  - robotics
  - policies
---

# Asymmetric Actor-Critic

**Asymmetric Actor-Critic** is a training pattern where the actor and critic use different observation views during training:

- The **actor** is kept constrained to what is available at deployment (e.g., image-only, partial state, sparse sensor inputs).
- The **critic** can use richer privileged state during training (e.g., full simulator state).

This asymmetry can improve learning speed and value estimation quality while preserving deployability on lower-observability real systems.

---

## 🧠 Why this exists

Classic actor-critic setups usually give actor and critic the same observation stream. In real robots, the actor often gets limited sensor information while a simulator can provide full state.

Asymmetric AC exploits this mismatch deliberately:

- **Actor** learns a policy that works under realistic sensing constraints.
- **Critic** learns with stronger supervision from privileged features, giving lower variance value targets and more stable gradients.

The result is usually a better sample-efficient learning signal in tasks with sparse observations, partial observability, or sim-to-real transfer.

---

## Core architecture

A minimal asymmetric AC loop has:

- `π(a|o_partial)` actor trained from deployable observations `o_partial`.
- `Q(s_privileged, a)` or `V(s_privileged)` critic trained with richer state info `s_privileged` in simulation.
- Shared or partially shared encoder structure depending on implementation.

This split is especially useful when privileged features include:

- Full scene state
- Exact contact/velocity/force channels
- Ground-truth object poses unavailable on hardware
- Internal task simulator metadata

---

## Strengths and limitations

### ✅ Pros

- Faster value learning when critic has privileged information.
- More stable critic gradients compared with pure partial-observation critics.
- Strong fit for sim-to-real workflows where real deployments are sensor-limited.

### ⚠️ Cons

- Careful to avoid leaking privileged information into the actor path.
- Can overfit to simulator-only features if training protocol is not audited.
- If the critic relies too heavily on privileged signals, transfer to real hardware may weaken unless actor-only constraints are enforced strongly.

---

## Practical use in robotics workflows

1. Define a clean sensor model for real deployment (this is actor input).
2. Define privileged simulator channels only for training-time critic supervision.
3. Train policy and critic jointly while preventing actor from using privileged encoders.
4. Validate rollout performance only through actor-observable inputs.
5. Compare against on-policy baselines (`PPO`, `A2C`) and off-policy baselines (`SAC`, `DDPG`) under same hardware constraints.

This is a strong fit when you have reliable sim-state channels but a constrained real-world sensing stack.

---

## Comparison table

| Method | Actor observation | Critic observation | Sim-to-real benefit | Risk if misconfigured |
|---|---|---|---|---|
| Symmetric actor-critic | Same (sim-like) | Same | Moderate | Low |
| **Asymmetric Actor-Critic** | Deployment-limited | Privileged simulator state | High | Moderate (leakage/overfitting) |
| Behavior cloning + critic fine-tune | Policy imitation target | Usually limited or none | Low | Medium |
| Domain randomization only | Deployment-limited | Deployment-limited | Medium | Low |
| Fully latent-space methods | Learned latent state | Learned latent state | Variable | High (interpretability + mismatch risk) |

---

## When to use

Use asymmetric AC when:

- You can get clean full-state data in simulation.
- The final policy must run on limited sensing.
- You need faster and more stable training than symmetric AC gives.

Avoid it when:

- Simulator state is close to real deployment state.
- You cannot enforce a strict actor-only input boundary.
- Real deployment has high sensor noise requiring robust actor-only representation work first.

---

## Related notes

- [[PPO]]
- [[A2C]]
- [[SAC]]
- [[Reinforcement Learning]]
- [[PufferLib]]
- [[PufferLib Robotics Fit and Limits]]
- [[Asymmetric AC in PufferLib]]
- [[Sim-to-Real Transfer]]

---

## Further reading

- Asymmetric actor-critic papers and implementations from robotics RL literature.
- Sim-to-real policy learning resources in this vault (especially Puffer-style env design).
- Robust critic training and partial-observation policy design notes.
