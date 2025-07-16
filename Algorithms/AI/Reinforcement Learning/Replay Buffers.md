# Replay Buffers

**Replay Buffers** (also known as experience replay) are data structures used in **reinforcement learning** (RL) to store and reuse past experiences during training. They are essential components of many **off-policy** algorithms, allowing the agent to learn from a distribution of previously encountered transitions rather than only from the most recent interaction with the environment.

Replay buffers improve sample efficiency, reduce correlation in training data, and enable stabilization of learning—especially in deep RL settings.

---

## 🧠 Overview

A typical replay buffer stores **transitions** of the form `(state, action, reward, next_state, done)` and allows for random sampling during training. This breaks the temporal correlations between consecutive steps and enables better generalization.

Key variants include:
- **Uniform Replay Buffer**: Samples transitions with equal probability.
- **Prioritized Replay Buffer**: Samples important transitions more frequently based on **TD error**.

---

## 🧪 Use Cases

- Required for **off-policy** algorithms like [[DQN]], [[DDPG]], [[SAC]]  
- Helpful in continuous and discrete action spaces  
- Enables stabilization in environments with sparse rewards  
- Allows **batch updates** from diverse experiences  
- Useful for **multi-agent replay sharing** (e.g., [[PettingZoo]], [[Neural MMO]])

---

## 📊 Comparison Table

| Type                      | Sampling Strategy       | Use Case                           | Notes                                    |
|---------------------------|--------------------------|-------------------------------------|------------------------------------------|
| Uniform                   | Random uniform sampling  | General off-policy RL               | Simple, fast                             |
| Prioritized               | Based on TD-error        | High-variance environments          | Improves learning from valuable samples  |
| Episodic Buffer           | Stores full episodes     | Tasks with long-term dependencies   | Enables trajectory-based learning        |
| Reservoir Sampling Buffer | Constant memory sampling | Streaming data scenarios            | Maintains representative set             |

---

## ⚙️ Key Features

- `add(transition)` — Add new experience to the buffer  
- `sample(batch_size)` — Sample a batch of experiences for training  
- `clear()` — Optionally reset the buffer  
- `max_size` — Fixed capacity with FIFO replacement  
- Optional features:
  - Priority weights
  - Importance sampling corrections
  - Temporal sequences for RNN policies

---

## ✅ Pros

- Improves sample efficiency  
- Breaks temporal correlation between experiences  
- Enables learning from diverse and past data  
- Compatible with parallel and distributed training

---

## ❌ Cons

- Not suitable for on-policy methods (e.g., [[PPO]])  
- Introduces stale data unless managed carefully  
- Prioritization adds complexity and overhead  
- Large buffers can consume significant memory

---

## 🔗 Related Concepts

- [[TD Learning]]  
- [[DQN]]  
- [[SAC]]  
- [[DDPG]]  
- [[Off-Policy]]  
- [[RL Transition]]  
- [[RL Step]]  
- [[RL Episode]]  
- [[RL Trajectory]]  
- [[Experience Replay]]  
- [[RL Agent]]

---

## 📚 Further Reading

- [Prioritized Experience Replay Paper (Schaul et al.)](https://arxiv.org/abs/1511.05952)  
- [Stable-Baselines3 Replay Buffer Docs](https://stable-baselines3.readthedocs.io/en/master/guide/custom_policy.html#custom-replay-buffer)  
- [CleanRL GitHub Examples](https://github.com/vwxyzjn/cleanrl)  
- [rl-agents replay buffer source](https://github.com/eleurent/rl-agents)

---
