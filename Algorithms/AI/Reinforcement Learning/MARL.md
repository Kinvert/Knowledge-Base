# MARL (Multi-Agent Reinforcement Learning) 🤖🤝🤖

MARL stands for **Multi-Agent Reinforcement Learning**, a subfield of RL where multiple agents learn and interact within the same environment. These agents may cooperate, compete, or act independently, and the complexity of their interactions makes MARL especially relevant to robotics, swarm behavior, autonomous vehicles, and strategy games.

---

## 🧠 Core Concepts

- **Decentralized vs Centralized Training:** Centralized critics can observe all agents; decentralized setups train agents independently.
- **Cooperative vs Competitive:** Agents can be aligned (shared reward) or opposed (zero-sum).
- **Observability:** Environments may be fully or partially observable from each agent's perspective.
- **Policy Sharing:** Agents can share policy networks to reduce compute and improve generalization.
- **Communication:** Some agents may pass messages or signals during training or execution (e.g., CommNet, DIAL).

---

## 🧪 Popular Environments

- `PettingZoo`: Standardized API for multi-agent RL
- `Multi-Agent Particle Envs (MPE)`
- `StarCraft Multi-Agent Challenge (SMAC)`
- `Overcooked-AI`
- `IsaacGym` with custom MARL tasks

---

## 🧠 Common Algorithms

| Algorithm      | Type           | Centralized Critic | Communication | Notes |
|----------------|----------------|---------------------|----------------|-------|
| [[MADDPG]]         | Mixed          | ✅                  | ❌             | Popular baseline |
| [[QMIX]]           | Cooperative    | ✅ (value mixing)   | ❌             | For shared reward |
| [[VDN]]            | Cooperative    | ✅ (value sum)      | ❌             | Simpler variant of QMIX |
| [[MAPPO]]          | Any            | ✅/❌               | ❌             | Multi-agent PPO variant |
| [[COMA]]           | Cooperative    | ✅                  | ❌             | Uses counterfactual advantage |
| [[DIAL]]           | Cooperative    | ✅                  | ✅             | Differentiable inter-agent communication |

---

## 🔁 Use Cases

- Swarm robotics and drone coordination
- Autonomous vehicle interaction
- Game AI with multiple agents
- Multi-robot warehouse systems
- Simulated economies and auctions

---

## ✅ Strengths

- Models real-world interaction and coordination
- Enables emergent strategies
- Can simulate complex social behavior
- Useful in both cooperative and adversarial settings

---

## ❌ Challenges

- Non-stationarity (environment changes as agents learn)
- Credit assignment in shared reward settings
- Scaling to many agents
- Communication bottlenecks and design

---

## 🔗 Tooling and Frameworks

- [[PettingZoo]] (Standard MARL environments)
- [[SuperSuit]] (Wrappers for PettingZoo)
- [[PufferLib]] (Lightweight Gym wrapper with MARL support)
- [[Stable-Baselines3]] (with limited MARL via custom wrappers)
- [[Ray RLlib]] (Built-in MARL support and scalability)

---

## 🧩 Related Notes

- [[Reinforcement Learning]] (RL basics)
- [[PettingZoo]] (Multi-agent environments)
- [[PufferLib]] (MARL-friendly environment wrappers)
- [[Isaac Gym]] (Supports multi-agent setups)
- [[OpenAI Gym]] (Single-agent base environments)
- [[Ray RLlib]] (Distributed and MARL RL framework)

---
