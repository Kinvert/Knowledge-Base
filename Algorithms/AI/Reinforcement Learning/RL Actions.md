# RL Actions

In **Reinforcement Learning (RL)**, *actions* are the decisions an agent makes to interact with its environment. They are the core output of an RL policy, and directly influence the agent’s next state and received reward. The space of possible actions (called the *action space*) plays a crucial role in defining how learning is performed.

Understanding how actions are structured, represented, and sampled is critical for implementing effective RL agents in both simulation and real-world robotics.

---

## 📚 Overview

Actions in RL fall into two major categories:

- `Discrete actions`: A finite set of possible choices (e.g., "move left", "move right")
- `Continuous actions`: Real-valued vectors representing magnitudes or directions (e.g., joint torques, velocities)

An RL agent’s objective is to learn a *policy* that maps observed states to these actions to maximize cumulative rewards.

---

## 🧠 Core Concepts

- `Action Space`: The set of all possible actions an agent can take  
- `Policy`: A function (deterministic or stochastic) that maps states to actions  
- `Exploration vs Exploitation`: Choosing between trying new actions or repeating known good ones  
- `Action Distribution`: Used in stochastic policies (e.g., Gaussian over continuous actions)  
- `Sampled Actions`: Chosen either from a learned distribution or deterministically from the policy  
- `Bounded Actions`: Most continuous action spaces are clipped within min/max limits  
- `Action Delay`: Some environments apply actions with a delay, important in real robotics  

---

## 🧰 Use Cases

- Selecting joint angles or torques in robot arms  
- Choosing movement commands for mobile robots  
- Managing throttle/brake/steering in autonomous vehicles  
- Triggering communication or sensory behaviors in swarm robotics  
- Switching between discrete modes (e.g., grasp, release)  

---

## ✅ Pros

- Flexible abstraction for control problems  
- Allows both high-level and low-level decision making  
- Can be extended to multi-agent settings  
- Compatible with simulation and real-world systems  

---

## ❌ Cons

- Large or continuous action spaces are harder to explore  
- Poorly bounded actions can lead to instability  
- Delayed or noisy action feedback complicates learning  
- Requires careful normalization and tuning for convergence  

---

## 📊 Comparison Table: Action Types

| Action Type       | Description                              | Example                      | Algorithms Often Used       |
|-------------------|------------------------------------------|------------------------------|-----------------------------|
| Discrete          | Finite set of choices                    | `left`, `right`, `idle`      | DQN, A2C, PPO (discrete)    |
| Continuous        | Real-valued vectors                      | `[0.2, -0.5, 1.0]`           | PPO, SAC, DDPG              |
| Multi-Discrete    | Tuple of multiple discrete actions        | `[0, 2, 1]`                  | A2C, PPO                    |
| Multi-Binary      | Vector of 0/1 choices                    | `[1, 0, 1]`                  | Custom use cases            |
| Parameterized     | Discrete action with continuous params   | `kick(power=0.8)`            | Complex RL environments     |

---

## 🤖 In Robotics Context

| Scenario                       | Example Action                      |
|--------------------------------|-------------------------------------|
| Mobile robot navigation        | `linear_velocity`, `angular_velocity`  
| Robotic arm control            | Joint torques or joint angle deltas  
| Drone control                  | Roll, pitch, yaw, throttle inputs  
| Gripper control                | `open` / `close` or force-based grip  
| Multi-agent coordination       | Agent-specific movement or messages  

---

## 🔧 Compatible Items

- [[Reinforcement Learning]] – Action is the primary agent output  
- [[Gymnasium]] – Defines standardized action spaces  
- [[PufferLib]] – Manages action sampling and vectorization  
- [[PettingZoo]] – Handles per-agent action interfaces  
- [[Isaac Gym]] – Accepts continuous actions from policies  
- [[Simulation Environments]] – Use action feedback to evolve state  

---

## 🔗 Related Concepts

- [[Observation Space]] (Defines what the agent sees)  
- [[Reinforcement Learning]] (Parent topic)  
- [[Policy Gradient Methods]] (Optimize action selection directly)  
- [[Gymnasium]] (Standardized RL environment API)  
- [[Action Space Normalization]] (Technique for improving learning stability)  

---

## 📚 Further Reading

- [OpenAI Spinning Up: Policy Gradient](https://spinningup.openai.com/en/latest/spinningup/rl_intro.html#policy-optimization)  
- [Gymnasium Spaces API](https://gymnasium.farama.org/api/spaces/)  
- [PPO with Continuous Actions Example](https://github.com/openai/baselines)  
- [SAC Paper](https://arxiv.org/abs/1812.05905) – Soft Actor Critic  

---
