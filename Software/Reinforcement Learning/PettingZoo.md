# 🟣 PettingZoo

**PettingZoo** is a Python library for **multi-agent reinforcement learning (MARL)** environments. It provides a unified API to access a wide variety of multi-agent environments in a way that mirrors the structure of [[OpenAI Gym]], making it easy to switch between environments and compatible with common RL libraries like [[Stable Baselines3]], [[RLlib]], and [[Supersuit]].

---

## 🧠 Summary

- A standard API for **multi-agent** environments.
- Inspired by [[OpenAI Gym]], but supports more than one agent.
- Compatible with vectorization, wrappers, and monitoring tools.
- Developed and maintained by Farama Foundation.

---

## 📦 Environment Categories

| Category       | Description                                                      | Example Environments                    |
|----------------|------------------------------------------------------------------|------------------------------------------|
| `atari`        | Multi-agent Atari games via ROM hacks                            | `pong_v2`, `boxing_v1`, `space_invaders` |
| `butterfly`    | Cooperative social dilemmas and coordination tasks               | `knights_archers_zombies_v10`           |
| `classic`      | Traditional board/card games                                     | `chess_v5`, `connect_four_v3`           |
| `magent`       | Large-scale agent simulations (via MAgent backend)               | `battle_v3`, `adversarial_pursuit_v3`   |
| `mpe`          | Multi-agent particle environments                                | `simple_spread_v2`, `simple_tag_v2`     |
| `sisl`         | Safety-oriented RL tasks                                         | `multiwalker_v7`                        |
| `toy_text`     | Simple text-based games                                          | `prison_v3`, `rps_v2`                   |

---

## 🧪 How It Works

PettingZoo follows an **Agent Environment Cycle (AEC)**:

1. Environment initialized.
2. Each agent takes turns making decisions.
3. Environment steps forward when all agents have acted.

This differs from Gym's vectorized simultaneous actions approach but matches many real-world multi-agent scenarios.

---

## 🛠️ Common Tools and Integration

- **[[Supersuit]]** – Wrappers for preprocessing PettingZoo environments.
- **[[Stable Baselines3]]** – Some limited support via wrappers or custom integration.
- **[[Ray RLlib]]** – Good compatibility via parallel environment wrappers.
- **[[Gymnasium]]** – PettingZoo environments can be converted to Gym-compatible formats.

---

## 🔁 API Structure

Each environment exposes:

- `env.reset()`
- `env.step(action)`
- `env.render()`
- `env.observe(agent)`
- `env.agent_selection`
- `env.agents`

It supports wrappers and tools in the Gym ecosystem.

---

## 🧪 Comparison Table

| Feature             | PettingZoo       | [[OpenAI Gym]] | [[Gymnasium]] | [[Unity ML-Agents]] | [[Multi-Agent Particle Envs]] |
|---------------------|------------------|----------------|----------------|----------------------|-------------------------------|
| Multi-agent Support | ✅ Native         | 🚫 Single agent | 🚫 Single agent | ✅ Native             | ✅ Native                     |
| Standard API        | ✅ Unified        | ✅              | ✅              | ⚠️ Unity-specific      | ❌ Non-standard               |
| Environment Variety | ✅ Broad          | ✅              | ✅              | ✅                    | ⚠️ Limited                   |
| Wrappers Support    | ✅ Supersuit      | ✅              | ✅              | 🚫                    | 🚫                           |

---

## 🏆 Strengths

- Well-structured API for turn-based multi-agent scenarios.
- Wide variety of environments across genres.
- Supports reproducibility and wrappers (via [[Supersuit]]).
- Cross-compatibility with many RL libraries.

---

## ⚠️ Weaknesses

- No native support for simultaneous moves (requires parallel wrapper).
- Requires rethinking standard Gym-based training pipelines.
- Some environments are more demonstration-focused than realistic.

---

## 📈 Use Cases

- Studying multi-agent cooperation, competition, and social dilemmas.
- Testing RL algorithms in settings with agent interdependence.
- Curriculum learning and coordination in control tasks.

---

## 🔗 Related Notes

- [[Reinforcement Learning]]
- [[OpenAI Gym]]
- [[Stable Baselines3]]
- [[Supersuit]]
- [[Multi-Agent Systems]]
- [[RLlib]]

---

## 🌐 External Resources

- [PettingZoo Docs](https://www.pettingzoo.farama.org/)
- [GitHub Repository](https://github.com/Farama-Foundation/PettingZoo)
- [Farama Foundation](https://www.farama.org/)
- [List of Supported Environments](https://www.pettingzoo.farama.org/environments/)

---
