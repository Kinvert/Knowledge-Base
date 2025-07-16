# RL Policy

In **Reinforcement Learning (RL)**, the *policy* is a strategy used by the agent to determine its actions based on the current state or observation. It defines the agent’s behavior and is at the heart of the learning process.

Formally, a policy can be deterministic (`a = π(s)`) or stochastic (`π(a|s)` gives a probability distribution over actions given a state). The goal of RL is often to find an *optimal policy* that maximizes the expected return.

---

## 📚 Overview

The policy maps input (state or observation) to output (action or action distribution). It can be:
- Explicit: a function or neural network trained via optimization
- Implicit: derived from a value function (e.g., greedy w.r.t Q-values)

Policies are used in almost all RL methods—value-based, policy-based, and hybrid actor-critic approaches.

---

## 🧠 Core Concepts

- `Deterministic Policy`: Maps state to a single action  
- `Stochastic Policy`: Outputs a probability distribution over actions  
- `Policy Gradient`: Optimization method for stochastic policies  
- `Greedy Policy`: Always picks the highest-valued action  
- `Exploration`: Choosing actions to gather information  
- `Exploitation`: Choosing actions to maximize return  

---

## 🧰 Use Cases

- Controlling a robot arm based on joint positions  
- Navigating an autonomous vehicle in an unknown map  
- Game AI choosing strategic moves  
- Industrial control systems tuning motors or processes  
- Multi-agent systems where each agent has a unique policy  

---

## ✅ Pros

- Flexible: can learn any behavior from data  
- Suited to continuous or discrete actions  
- Stochasticity improves exploration  
- Natural for on-policy learning (e.g. PPO, A2C)  
- Encodable as neural networks for function approximation  

---

## ❌ Cons

- Can be unstable to train  
- High variance in gradient estimation  
- Poor exploration leads to suboptimal policies  
- Difficult to interpret in deep RL contexts  

---

## 📊 Comparison Table: Policy Types

| Type              | Description                          | Common Algorithms | Suitable For                  |
|-------------------|--------------------------------------|-------------------|-------------------------------|
| Deterministic     | Maps state to one action             | DDPG, TD3         | Continuous control            |
| Stochastic        | Outputs action probabilities         | PPO, A3C, REINFORCE | Discrete + exploration-heavy |
| Value-derived     | Chooses action from Q-values         | DQN               | Simple discrete tasks         |
| Learned directly  | Trained via gradient-based methods   | Policy gradient   | Complex policy modeling       |

---

## 🤖 In Robotics Context

| Scenario                 | Policy Role                                      |
|--------------------------|--------------------------------------------------|
| Manipulator control      | Determines joint torques from camera input       |
| Navigation               | Maps lidar scan to velocity command              |
| Biped locomotion         | Produces stable gait using learned motor commands|
| Multi-agent cooperation  | Agent-specific policy for collaboration          |
| Drone obstacle avoidance | Maps sensor data to acceleration changes         |

---

## 🔧 Compatible Items

- [[Reinforcement Learning]] – The policy is what the agent learns  
- [[RL Agent]] – Encapsulates and updates the policy  
- [[RL Actions]] – Selected according to the policy  
- [[Actor Critic]] – Uses a policy (actor) and a value function (critic)  
- [[Policy Gradient Methods]] – Algorithms to learn the policy  
- [[RL Reward]] – Guides optimization of the policy  

---

## 🔗 Related Concepts

- [[Exploration vs Exploitation]] – Balancing policy behavior  
- [[Entropy Regularization]] – Promotes diverse policy actions  
- [[Value Function]] – Can be used to derive a policy  
- [[RL Environment]] – Where the policy acts and learns  
- [[SAC]] – (Soft Actor-Critic) Stochastic policy optimization  

---

## 📚 Further Reading

- [Sutton & Barto – Policies](http://incompleteideas.net/book/the-book.html)  
- [Spinning Up: Policy Gradient](https://spinningup.openai.com/en/latest/spinningup/rl_intro.html#policies)  
- [Lil’log: Policy Gradient Algorithms](https://lilianweng.github.io/lil-log/2018/04/08/policy-gradient-algorithms.html)  
- [Stable-Baselines3 Policy Guide](https://stable-baselines3.readthedocs.io/en/master/guide/policies.html)  

---
