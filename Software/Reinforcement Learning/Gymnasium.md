# 🟣 Gymnasium

**Gymnasium** is the official successor to [[OpenAI Gym]], maintained by the Farama Foundation. It provides a standardized interface for developing and comparing reinforcement learning (RL) algorithms in single-agent environments. Gymnasium enhances and modernizes the Gym API while maintaining backward compatibility in most cases.

---

## 🧠 Summary

- Drop-in replacement for OpenAI Gym with long-term maintenance.
- Provides a unified interface for RL environments.
- Supports both classic control tasks and complex physics simulators.
- Actively maintained and versioned environments.

---

## 🔁 Core Concepts

| Concept           | Description                                                                 |
|-------------------|-----------------------------------------------------------------------------|
| Environment        | A simulation where an agent interacts with the world.                      |
| Action Space       | Defines the set of valid agent actions (e.g., Discrete, Box).              |
| Observation Space  | Describes the format of inputs to the agent.                               |
| Step               | One action taken by the agent, yielding observation, reward, done, info.   |
| Reset              | Reinitializes the environment to a starting state.                         |

---

## ✅ Key Features

- Type-annotated, modern Pythonic API
- Built-in environment versioning and testing
- Enhanced consistency across environments
- Compatible with vectorized and asynchronous agents
- Extensive documentation and community support

---

## 🆚 Comparison to OpenAI Gym

| Feature                     | OpenAI Gym        | Gymnasium             |
|-----------------------------|-------------------|------------------------|
| Maintained By               | OpenAI (archived) | Farama Foundation      |
| API Type Annotations        | ❌ None            | ✅ Full                |
| Environment Versioning      | ❌ Manual          | ✅ Built-in            |
| Vectorized Environments     | Limited            | ✅ Supported           |
| Community Activity          | Low                | High                   |

---

## 🚀 Use Cases

- Training and benchmarking RL agents
- Evaluating new control strategies
- Academic RL research
- Curriculum learning via environment wrappers

---

## 🏆 Strengths

- Open-source and community-supported
- Works seamlessly with RL libraries (e.g., [[Stable Baselines3]], [[Tianshou]], [[RLlib]])
- Easily extensible for custom tasks
- Consistent interface reduces integration issues

---

## ⚠️ Weaknesses

- Primarily designed for single-agent RL (see [[PettingZoo]] for multi-agent support)
- Some external environments require updates for full compatibility

---

## 🔗 Related Notes

- [[OpenAI Gym]]
- [[Stable Baselines3]]
- [[Tianshou]]
- [[PettingZoo]]
- [[Supersuit]]
- [[Reinforcement Learning]]

---

## 🌐 External Resources

- https://gymnasium.farama.org/
- https://github.com/Farama-Foundation/Gymnasium

---
