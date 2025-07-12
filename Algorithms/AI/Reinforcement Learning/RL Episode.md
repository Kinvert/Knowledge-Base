# RL Episode

In **Reinforcement Learning (RL)**, an *episode* is a complete sequence of interactions between an agent and its environment, beginning with an initial state and ending in a terminal state. It is the unit of experience in episodic tasks and typically corresponds to a single trial or attempt at solving a task.

Episodes are used to collect data, compute returns, and evaluate agent performance.

---

## 📚 Overview

An episode starts when the environment is reset (usually via `env.reset()`) and proceeds step by step (`env.step(action)`) until the `done` flag is `True`. Each step produces:
- `observation` (or state)
- `action`
- `reward`
- `next observation`
- `done`
- `info`

An episode ends when a terminal condition is met, such as reaching a goal, failing a task, or running out of time steps.

---

## 🧠 Core Concepts

- `Episode`: Sequence from reset to terminal state  
- `Trajectory`: Often used as a synonym for episode in RL  
- `Return`: Cumulative reward over the episode  
- `Terminal State`: A state where the environment sets `done = True`  
- `Episode Length`: Number of steps before termination  
- `Truncation`: When an episode ends due to time or step limit, not natural termination  

---

## 🧰 Use Cases

- Training RL agents from collected episodes  
- Logging for debugging or performance tracking  
- Evaluation of policy changes over consistent tasks  
- Curriculum learning based on episode success/failure  
- Collecting trajectories for imitation or offline learning  

---

## ✅ Pros

- Natural unit for experience in many RL tasks  
- Allows return calculation for learning  
- Enables episodic resets and curriculum training  
- Simplifies evaluation and benchmarking  

---

## ❌ Cons

- May result in uneven training data (especially in sparse rewards)  
- Long or inconsistent episode lengths can slow training  
- Requires terminal condition definition in custom environments  
- Some environments (e.g., robotics control) may not have clear episode boundaries  

---

## 📊 Comparison Table: Episodic vs Continuing Tasks

| Property              | Episodic                          | Continuing                        |
|-----------------------|-----------------------------------|-----------------------------------|
| End Condition         | Terminal state or max steps       | No explicit end                   |
| Learning Target       | Return over episode               | Discounted infinite return        |
| Examples              | Games, maze-solving, pick & place | Factory control, trading bots     |
| Reset Required        | After each episode                | Not needed, though may be used    |
| Suitable Algorithms   | Most RL algorithms                | Special discounting or bootstraps |

---

## 🤖 In Robotics Context

| Scenario               | Episode Definition                        |
|------------------------|-------------------------------------------|
| Pick-and-place task    | One full object placement                 |
| Drone flight           | From takeoff to landing or crash          |
| Navigation             | Start to goal or collision                |
| Human interaction      | One interaction session or time window    |
| Industrial arm control | One sequence of motions to complete task  |

---

## 🔧 Compatible Items

- [[Reinforcement Learning]] – Episodes are core to most training paradigms  
- [[RL Step]] – Steps build up an episode  
- [[RL Return]] – Computed over the episode  
- [[RL Agent]] – Learns from episodic experiences  
- [[Trajectory]] – Often synonymous with episode  
- [[Replay Buffer]] – Stores past episodes or partial episodes  

---

## 🔗 Related Concepts

- [[Trajectory]] (Episodes are sequences of transitions)  
- [[Reset Function]] (Starts a new episode)  
- [[POMDP]] (Episodes can vary based on observation)  
- [[RL Reward]] (Episodes accumulate reward for learning)  
- [[Episode Length Limit]] (Can truncate episodes prematurely)  

---

## 📚 Further Reading

- [Sutton & Barto – Episodic Tasks](http://incompleteideas.net/book/the-book.html)  
- [OpenAI Gym – Episodic Envs](https://gymnasium.farama.org/)  
- [Spinning Up: RL Loop](https://spinningup.openai.com/en/latest/spinningup/rl_intro.html#the-reinforcement-learning-loop)  
- [PettingZoo API – Episode Tracking](https://pettingzoo.farama.org/)  

---
