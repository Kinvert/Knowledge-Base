# RL Agent

In **Reinforcement Learning (RL)**, an *agent* is the decision-making entity that learns to act within an environment to maximize cumulative reward. The agent observes the environment’s state, selects actions according to a policy, and updates its behavior based on the outcomes (rewards) received.

The agent is the central component in the RL loop and typically encapsulates the learning algorithm, policy, value function (if applicable), and sometimes internal memory or models of the environment.

---

## 📚 Overview

An RL agent’s objective is to learn a *policy* that maps observations to actions. It interacts with an *environment* through repeated episodes, gradually improving its performance through experience. The agent may use different learning strategies such as value iteration, policy gradients, or model-based learning.

The agent can be stateless (pure policy) or stateful (recurrent or memory-based), and may support continuous, discrete, or hybrid action spaces.

---

## 🧠 Core Concepts

- `Policy`: Function that maps observations to actions  
- `Value Function`: Estimates expected cumulative reward from a state or state-action pair  
- `Exploration Strategy`: How the agent tries new actions (e.g. ε-greedy, entropy maximization)  
- `Replay Buffer`: Stores past transitions for training  
- `Learning Algorithm`: Method used to update the agent’s parameters (e.g. DQN, PPO, A3C)  
- `Actor-Critic`: Architecture where actor selects actions and critic estimates value  

---

## 🧰 Use Cases

- Autonomous navigation and path planning  
- Manipulator control in robotics  
- Multi-agent swarm coordination  
- Game playing (Atari, Go, StarCraft)  
- Trading bots and decision-making systems  
- Simulated training before real-world deployment  

---

## ✅ Pros

- Can learn complex behaviors from interaction  
- Adaptable to various environments and tasks  
- Requires minimal supervision (compared to supervised learning)  
- Works in continuous or discrete domains  
- Easily extensible to multi-agent systems  

---

## ❌ Cons

- Sample inefficiency, especially in sparse reward settings  
- Can overfit or learn unsafe behaviors without constraints  
- Hyperparameter tuning is often difficult and task-specific  
- Reward hacking if the incentive structure is poorly defined  
- Reproducibility challenges in stochastic environments  

---

## 📊 Comparison Table: Agent Architectures

| Agent Type       | Description                              | Common Algorithms         | Use Cases                            |
|------------------|------------------------------------------|---------------------------|--------------------------------------|
| Value-based      | Learns Q-values or V-values              | DQN, Double DQN           | Discrete action problems             |
| Policy-based     | Directly learns the policy                | REINFORCE, PPO            | Continuous actions, stable policies |
| Actor-Critic     | Combines value + policy networks          | A2C, A3C, SAC             | Stability with large state spaces   |
| Model-based      | Learns a model of the environment         | MBPO, MuZero              | Sample-efficient but complex         |
| Multi-agent      | Manages multiple learning agents          | MADDPG, QMIX              | Swarm and team coordination          |

---

## 🤖 In Robotics Context

| Scenario                   | Agent Behavior                                    |
|----------------------------|---------------------------------------------------|
| Pick-and-place             | Learns object grasping and motion control         |
| Drone flight               | Learns stable hovering and obstacle avoidance     |
| Quadruped locomotion       | Learns gaits and balance                          |
| Human-robot interaction    | Learns appropriate social responses               |
| Factory task automation    | Learns efficiency and safety in sequence control  |

---

## 🔧 Compatible Items

- [[Reinforcement Learning]] – The agent is the learner in the RL framework  
- [[RL Actions]] – Selected by the agent at each timestep  
- [[RL Observations]] – Received by the agent from the environment  
- [[RL Reward]] – Guides the learning of the agent  
- [[PufferLib]] – Simplifies managing agents across vectorized environments  
- [[PettingZoo]] – Multi-agent wrapper supporting multiple concurrent agents  

---

## 🔗 Related Concepts

- [[Policy Gradient Methods]] (Subset of agents that use gradients to learn policies)  
- [[Replay Buffer]] (Stores agent interactions for experience replay)  
- [[Actor Critic]] (Agent architecture with separate action and value models)  
- [[RL Environment]] (The world the agent interacts with)  
- [[RL Step]] (Each decision cycle of the agent)  

---

## 📚 Further Reading

- [OpenAI Spinning Up: Agents](https://spinningup.openai.com/en/latest/)  
- [DeepMind's Agent57](https://deepmind.com/blog/article/Agent57-Outperforming-the-human-Atari-benchmark)  
- [Stable-Baselines3 Agents](https://github.com/DLR-RM/stable-baselines3)  
- [Sutton & Barto - RL Book](http://incompleteideas.net/book/the-book.html)  
- [RLlib Agents](https://docs.ray.io/en/latest/rllib/index.html)  

---
