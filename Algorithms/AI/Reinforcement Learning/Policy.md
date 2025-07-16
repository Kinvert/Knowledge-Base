# Policy

In reinforcement learning (RL), a **Policy** is a function or strategy that defines the agent’s behavior by mapping observed states of the environment to actions. It is a core component in all RL systems, whether learned or predefined, and determines how the agent interacts with the environment to maximize long-term reward.

Policies can be deterministic or stochastic and are often represented as neural networks in deep RL.

---

## 🧠 Overview

- **Deterministic Policy**: Always returns the same action for a given state, e.g., `a = π(s)`  
- **Stochastic Policy**: Returns a probability distribution over actions, e.g., `a ~ π(a|s)`  
- Policies can be:
  - **Parameterized** and learned through gradient descent (common in [[Policy Gradient Methods]])  
  - **Tabular**, used in simpler environments  
  - **Hand-crafted**, e.g., in rule-based or traditional control systems

---

## 🧪 Use Cases

- Core logic for decision-making in reinforcement learning agents  
- Policy optimization via algorithms like [[PPO]], [[TRPO]], [[DQN]]  
- Robotics controllers that adapt to uncertain environments  
- Game-playing agents in simulators or real-time strategy scenarios  
- Simulation-based training (e.g., [[Isaac Gym]], [[PettingZoo]])  
- Multi-agent learning and competition setups (e.g., [[Neural MMO]])

---

## ⚙️ Capabilities

- Defines agent behavior over the entire state space  
- Can be learned or manually designed  
- Determines the quality and generalization of the RL agent  
- Optimized with reward signals via [[Policy Gradient]], [[Actor Critic]], etc.

---

## 📊 Comparison Table

| Policy Type         | Description                                  | Common In             | Learnable | Notes                               |
|---------------------|----------------------------------------------|------------------------|-----------|-------------------------------------|
| Deterministic        | Maps each state to one action               | [[DQN]], Classical RL | ✅        | Fast and stable, but lacks exploration |
| Stochastic           | Outputs action probabilities                | [[PPO]], [[TRPO]]     | ✅        | Encourages exploration              |
| Tabular              | Table of state-action mappings              | Gridworld, low-dim    | ✅        | Simple, not scalable                |
| Neural Network       | Deep function approximator (πθ(s))          | Modern DRL            | ✅        | Powerful but data-intensive         |
| Hand-crafted         | Rule-based, not learned                     | Classical robotics    | ❌        | Human-defined logic only            |

---

## ✅ Pros

- Central to learning in RL  
- Flexible across a wide variety of problems  
- Supports both model-free and model-based RL  
- Encourages efficient exploration when designed well

---

## ❌ Cons

- May require large amounts of training data  
- Poorly initialized policies can lead to slow learning  
- Balancing exploration and exploitation is non-trivial  
- Sensitive to reward shaping

---

## 🔗 Related Concepts

- [[RL Agent]]  
- [[RL Policy]]  
- [[Policy Gradient Methods]]  
- [[Value Function]]  
- [[Actor Critic]]  
- [[PPO]]  
- [[TRPO]]  
- [[DQN]]  
- [[State]]  
- [[Action Space]]  
- [[Observation Space]]

---

## 📚 Further Reading

- [Spinning Up RL – Policies](https://spinningup.openai.com/en/latest/spinningup/key-concepts.html#policies)  
- [RL Book: Sutton & Barto – Chapter 13](http://incompleteideas.net/book/the-book-2nd.html)  
- [CS285 Deep RL Lectures](http://rail.eecs.berkeley.edu/deeprlcourse/)

---
