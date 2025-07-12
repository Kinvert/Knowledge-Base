# RL Transition

In **Reinforcement Learning (RL)**, a *transition* refers to a single step of interaction between the agent and the environment. It captures how the agent's action causes a change in the environment and what the agent receives in response.

Formally, a transition is typically represented as a tuple:
`(s_t, a_t, r_{t+1}, s_{t+1})`

This is the atomic unit of experience in RL and is used to build higher-level constructs like trajectories and episodes.

---

## 📚 Overview

A transition captures:
- The **current state** `s_t`
- The **action** `a_t` taken by the agent
- The **reward** `r_{t+1}` given by the environment
- The **next state** `s_{t+1}` resulting from the action

This unit is crucial in value function estimation, policy learning, and experience replay. In tabular methods, transitions are stored directly; in deep RL, they are often batched and replayed during training.

---

## 🧠 Core Concepts

- `Transition`: The outcome of a single step: state → action → next state + reward  
- `Experience Tuple`: `(s_t, a_t, r_{t+1}, s_{t+1})`  
- `Reward Signal`: Captured in the transition  
- `Step`: A function like `env.step(a)` that returns a transition  
- `Experience Replay`: Stores and reuses transitions  
- `Temporal-Difference (TD) Learning`: Uses transitions to update value estimates  

---

## 🧰 Use Cases

- Building training batches for Q-learning and DDPG  
- Updating policies or value functions based on sampled transitions  
- Logging interaction data for analysis  
- Constructing episodes and trajectories  
- Debugging agent behavior at the lowest resolution  

---

## ✅ Pros

- Simple and lightweight data unit  
- Enables modular data collection and training  
- Compatible with all major RL frameworks  
- Efficient for sampling and storage in off-policy methods  

---

## ❌ Cons

- Individual transitions lack global context  
- Learning from isolated transitions may ignore long-term dependencies  
- Reward sparsity can reduce information in transitions  
- Credit assignment is harder with disconnected transitions  

---

## 📊 Comparison Table: Transitions vs Trajectories vs Episodes

| Unit         | Description                                      | Size          | Use Cases                          |
|--------------|--------------------------------------------------|---------------|------------------------------------|
| Transition   | One step: `(s, a, r, s')`                        | 1             | Online updates, replay buffers     |
| Trajectory   | Sequence of transitions (partial or full)        | Variable      | Policy gradients, batch training   |
| Episode      | Full trajectory from `reset` to `done = True`    | Task-dependent| Evaluation, return computation     |

---

## 🤖 In Robotics Context

| Application            | Example Transition                             |
|------------------------|-------------------------------------------------|
| Robot arm              | `s`: joint angles, `a`: torque, `r`: accuracy  |
| Drone navigation       | `s`: GPS + IMU, `a`: motor command             |
| Warehouse robot        | `s`: shelf state, `a`: pick/drop, `r`: success |
| Quadruped locomotion   | `s`: sensor input, `a`: joint positions        |
| Assembly robot         | `s`: part config, `a`: move, `r`: match result |

---

## 🔧 Compatible Items

- [[Reinforcement Learning]] – Transitions are key data points  
- [[RL Step]] – Each step yields a transition  
- [[Replay Buffer]] – Stores transitions for training  
- [[RL Return]] – Accumulated over transitions  
- [[RL Episode]] – Sequence of transitions  
- [[Temporal Difference Learning]] – Learns from transitions  

---

## 🔗 Related Concepts

- [[Transition Function]] (Probability model of transitions in MDPs)  
- [[Replay Buffer]] (Mechanism for storing transitions)  
- [[Trajectory]] (Built from multiple transitions)  
- [[Experience Replay]] (Batching transitions for training)  
- [[Q-Learning]] (Uses transitions to estimate value updates)  

---

## 📚 Further Reading

- [Sutton & Barto – RL Fundamentals](http://incompleteideas.net/book/the-book.html)  
- [Spinning Up: Transition Data](https://spinningup.openai.com/en/latest/spinningup/rl_intro.html#interacting-with-the-environment)  
- [Deep Q-Learning Paper](https://www.cs.toronto.edu/~vmnih/docs/dqn.pdf)  
- [Stable Baselines3: Replay Buffer](https://stable-baselines3.readthedocs.io/en/master/guide/replay_buffer.html)  

---
