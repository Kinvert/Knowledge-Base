---
title: Catastrophic Forgetting
tags:
  - reinforcement-learning
  - machine-learning
  - continual-learning
  - stability
  - optimization
aliases:
  - Catastrophic Interference
  - Forgetting in Sequential Learning
---

# Catastrophic Forgetting

**Catastrophic Forgetting** (also called **catastrophic interference**) is the tendency of neural systems to lose previously learned behavior when trained sequentially on new tasks or data distributions.

In reinforcement learning, this is especially painful when you continue fine-tuning a policy from a replay buffer swap, domain shift, or staged curriculum and the model becomes much worse on older environments.

---

## 🧠 What it is

Catastrophic forgetting shows up when optimization updates shift shared parameters toward a new task objective and effectively overwrite representations that were useful before.

- **Single model, multiple stages**: training in sequence with shared weights
- **Non-stationary data**: task changes, changing reward structures, or evolving dynamics
- **Strong updates**: high learning rate, large policy changes, or overfitting to new samples

Unlike normal trade-off in optimization, forgetting here can be abrupt and asymmetric: you can become good at the newest regime while losing broad competence.

---

## 🧪 Why it appears in RL

Reinforcement learning amplifies this effect because your data is usually:

- **On-policy / environment-coupled**: sample distribution changes with policy
- **Reward-shaped and sparse**: feedback can over-constrain updates
- **Curriculum-ordered**: tasks are often presented in increasing complexity
- **Distribution-shift prone**: sim-to-real, terrain changes, and parameter sweeps

When RL code reuses one policy network across phases, each phase can drag the policy/value landscape toward a local optimum that is no longer compatible with older tasks.

---

## ⚠️ Symptoms

In practice, this often looks like:

- New behavior improves, old behavior collapses
- Q-value / value baseline drifts unexpectedly after stage transitions
- Policy “forgets” rare but critical failure-handling modes
- High variance in episodic returns when task switches back
- Fine-tuning speed increases while long-horizon competence decays

---

## 🧰 Mitigation strategies

## 1) Replay-based approaches

- Keep representative data from prior tasks (`replay buffers`, `reservoir buffers`, `episodic memory`).
- Mix old and new transitions when training.
- In RL, include old-policy behavior snapshots when possible for stability.

## 2) Regularization methods

- Penalize weight drift from previous task optima.
- Prevent important parameters from moving too far while learning new tasks.
- This category includes constraints and importance-weighted losses around previously useful parameters.

## 3) Architectural strategies

- Reserve capacity per task (e.g., separate adapters/heads, progressive expansion).
- Use expert routing, gating, or context-conditioned policies.
- Freeze shared layers and train task-specific adapters when throughput matters.

## 4) Data and task scheduling

- Interleave old and new tasks instead of strict sequential blocks.
- Use controlled curricula with periodic rehearsal phases.
- Evaluate after every phase with a fixed holdout from prior tasks.

## 5) Evaluation discipline

- Always track old-task return curves after each update cycle.
- Keep an explicit stability metric in addition to reward progression.
- Treat forgetting as first-class regression in CI/pipeline gating.

---

## 📉 Why it matters for Puffer/robot workflows

In robotic RL loops, forgetting is usually a deployment risk, not just a training artifact:

- A policy that forgets old failure modes can appear stable in short benchmarks and fail in rare real-world conditions.
- Transfer across environments should be staged with clear backward-compatibility tests.
- For sim-to-real loops, maintain at least one “legacy task set” as a non-negotiable regression target.

---

## 📊 Comparison table

| Method type | Memory cost | Plasticity | Forgetting reduction | When to use |
|---|---:|---|---|---|
| Plain fine-tuning | Low | Very high | Poor | Only when no prior-task stability needed |
| Replay mixing | Medium-High | Medium | Good | Sequential task training with bounded memory |
| Regularization-based | Low | Medium | Moderate | Resource-limited setups with stable deployment constraints |
| Architectural isolation | High | Medium | Very good | Multi-task stacks where task boundaries are known |
| Multi-task joint training | Medium-High | Medium | Good (if enough coverage) | When tasks can be co-trained safely |

---

## 🔧 Practical pattern (small setup)

1. Train task A and checkpoint policy/value encoder.
2. Evaluate and archive representative trajectories.
3. Train task B with replay from both B and a sampled subset of archived A data.
4. Re-run A-regression tests at checkpoints.
5. If forgetting rises above threshold, reduce update aggressiveness or increase rehearsal.

---

## ⚙️ Related notes

- [[PufferLib]]
- [[PufferLib Robotics Fit and Limits]]
- [[OpenAI Gym]]
- [[PufferLib C99 Environment Authoring]]

---

## 📚 Further Reading

- [Catastrophic Interference (overview)](https://en.wikipedia.org/wiki/Catastrophic_interference)
- Continual learning notes, replay and regularization papers from the literature
- [ContinualAI / Avalanche](https://github.com/ContinualAI/avalanche)
