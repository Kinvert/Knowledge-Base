# RL Trajectory

In **Reinforcement Learning (RL)**, a *trajectory* is a sequence of states, actions, and rewards experienced by an agent during interaction with the environment. It represents the path taken by the agent over time, typically collected during one episode or partial episode.

Trajectories are essential for policy evaluation, training, and experience replay.

---

## 📚 Overview

A trajectory is often denoted as:

`τ = (s₀, a₀, r₁, s₁, a₁, r₂, ..., s_T)`

where:
- `s_t` is the state at time `t`
- `a_t` is the action taken at time `t`
- `r_{t+1}` is the reward received after action `a_t`
- `T` is the terminal timestep (episode length)

Trajectories can be full (complete episodes) or partial (fixed length segments).

---

## 🧠 Core Concepts

- `Trajectory`: Ordered sequence of states, actions, and rewards  
- `Episode`: Often synonymous with a full trajectory  
- `Return`: Computed from rewards along the trajectory  
- `On-policy`: Trajectories collected using current policy  
- `Off-policy`: Trajectories collected using a different policy  
- `Replay Buffer`: Stores trajectories for reuse during training  

---

## 🧰 Use Cases

- Training value functions and policies via Monte Carlo or policy gradients  
- Experience replay in off-policy methods (e.g., DQN, DDPG)  
- Imitation learning from expert trajectories  
- Behavioral cloning or inverse RL  
- Evaluation and debugging of agent behavior  

---

## ✅ Pros

- Captures rich sequential information for learning  
- Enables sample-efficient reuse through replay buffers  
- Supports batch training and parallelization  
- Provides data for offline RL and transfer learning  

---

## ❌ Cons

- Storing long trajectories can be memory-intensive  
- May contain redundant or uninformative data  
- Partial trajectories can bias learning if not handled correctly  
- Data distribution mismatch in off-policy learning  

---

## 📊 Comparison Table: Trajectory vs Episode vs Transition

| Term         | Definition                                  | Typical Length        | Usage                         |
|--------------|----------------------------------------------|-----------------------|-------------------------------|
| Transition   | Single (s, a, r, s’) tuple                   | 1 timestep            | Step-wise updates in RL        |
| Trajectory   | Sequence of transitions                      | Partial or full episode | Batch training and replay     |
| Episode     | Complete trajectory from start to terminal   | Full episode length   | Return calculation and evaluation |

---

## 🤖 In Robotics Context

| Scenario                | Trajectory Use                                      |
|-------------------------|-----------------------------------------------------|
| Robot arm manipulation  | Sequence of joint states and actions during a task  |
| Autonomous driving      | Path and control commands over a driving session    |
| Drone flight            | Flight path with sensor data and control inputs     |
| Multi-agent systems     | Individual agent trajectories for coordination      |
| Simulation training     | Recorded trajectories for offline policy learning   |

---

## 🔧 Compatible Items

- [[Reinforcement Learning]] – Trajectories are core data for training  
- [[Replay Buffer]] – Stores and samples trajectories  
- [[RL Episode]] – Trajectories often correspond to episodes  
- [[RL Agent]] – Collects and learns from trajectories  
- [[RL Return]] – Computed over trajectories  
- [[Policy Gradient Methods]] – Use trajectories to estimate gradients  

---

## 🔗 Related Concepts

- [[Transition]] (Basic building block of trajectories)  
- [[Monte Carlo Methods]] (Use full trajectories for updates)  
- [[Experience Replay]] (Sampling from stored trajectories)  
- [[On-policy vs Off-policy]] (Trajectory source differences)  
- [[Imitation Learning]] (Learning from expert trajectories)  

---

## 📚 Further Reading

- [Sutton & Barto – Trajectories and RL](http://incompleteideas.net/book/the-book.html)  
- [OpenAI Spinning Up: Trajectories](https://spinningup.openai.com/en/latest/spinningup/rl_intro.html#trajectories)  
- [Experience Replay in RL](https://arxiv.org/abs/1312.5602)  
- [PettingZoo – Multi-agent trajectories](https://pettingzoo.farama.org/)  

---
