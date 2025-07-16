# A2C (Advantage Actor-Critic)

**A2C**, or **Advantage Actor-Critic**, is a synchronous variant of [[A3C]] where multiple agents interact with the environment **in parallel**, but their experiences are collected and **used in a synchronized fashion** to update the shared policy and value networks. It retains the core architecture of A3C but simplifies training by avoiding the complexities of asynchronous updates.

A2C is widely used in reinforcement learning (RL) due to its simplicity, stability, and support in many modern frameworks.

---

## 🧠 Overview

A2C performs the following steps:
- Runs multiple environment instances in parallel (using separate processes or threads)
- Collects experience for a fixed number of steps
- Computes **advantage estimates** using the value function
- Updates the **policy (actor)** and **value function (critic)** using gradient descent
- Applies optional **entropy regularization** to encourage exploration

---

## 🧪 Use Cases

- Tasks with either discrete or continuous action spaces  
- Benchmarking against more complex RL algorithms  
- Environments that benefit from parallel simulation (e.g., [[Isaac Gym]], [[PettingZoo]])  
- Simpler alternatives to [[PPO]] or [[SAC]] when computational simplicity is desired

---

## 📊 Comparison Table

| Algorithm   | Type          | On/Off Policy | Parallelism Style | Replay Buffer | Notes                                 |
|-------------|---------------|----------------|-------------------|---------------|----------------------------------------|
| A2C         | Actor-Critic   | On-Policy       | Synchronous       | ❌             | Stable, easier to implement than A3C   |
| [[A3C]]     | Actor-Critic   | On-Policy       | Asynchronous      | ❌             | Parallelism through threads            |
| [[PPO]]     | Actor-Critic   | On-Policy       | Optional          | ❌             | More robust updates with clipping      |
| [[SAC]]     | Actor-Critic   | Off-Policy      | Optional          | ✅             | Works better in continuous domains     |
| [[DQN]]     | Value-Based    | Off-Policy      | Serial            | ✅             | Uses experience replay                 |

---

## ⚙️ Core Components

- **Advantage Estimation**: Reduces variance in policy gradients  
- **Synchronous Workers**: Collect rollouts in lockstep  
- **Actor**: Outputs action probabilities or values  
- **Critic**: Predicts expected return (value function)  
- **Entropy Term**: Maintains exploration pressure

---

## ✅ Pros

- Easier to implement and debug than [[A3C]]  
- Parallel rollouts stabilize learning  
- On-policy algorithm—no replay buffer needed  
- Performs well in many classical control environments

---

## ❌ Cons

- Less sample efficient than off-policy methods  
- Synchronization can be a bottleneck at large scale  
- Sensitive to hyperparameters (like learning rate, entropy coefficient)  
- Can still suffer from local optima and slow convergence in sparse-reward tasks

---

## 🔗 Related Concepts

- [[A3C]]  
- [[Actor Critic]]  
- [[Advantage Function]]  
- [[TD Learning]]  
- [[Policy Gradient Methods]]  
- [[Entropy Regularization]]  
- [[RL Policy]]  
- [[Value Function]]  
- [[RL Agent]]  
- [[RL Episode]]  
- [[RL Trajectory]]

---

## 📚 Further Reading

- [OpenAI Baselines – A2C](https://github.com/openai/baselines/tree/master/baselines/a2c)  
- [CleanRL A2C Implementation](https://github.com/vwxyzjn/cleanrl)  
- [Deep Reinforcement Learning Hands-On (2nd Edition) – Chapter on A2C]  
- [RLlib Docs – A2C Trainer](https://docs.ray.io/en/latest/rllib-algorithms.html#a2c)

---
