# Action Space

In reinforcement learning (RL) and robotics, the **Action Space** defines the set of all possible actions an agent can take at any given [[RL State]]. It is a fundamental concept that shapes how an agent interacts with its environment and directly affects the design and complexity of learning algorithms.

---

## 🔍 Overview

- The action space can be **discrete** (a finite set of distinct actions) or **continuous** (actions take values in continuous ranges).  
- The choice of action space influences the type of RL algorithms suitable for the problem.  
- Defining an appropriate action space is crucial for effective policy learning and efficient exploration.  
- Action spaces can be multi-dimensional, combining several control variables simultaneously.  

---

## 🧠 Core Concepts

- **Discrete Action Space**: The agent selects from a finite set of predefined actions (e.g., move left, move right, jump). Typical in classic RL problems and many games.  
- **Continuous Action Space**: Actions are real-valued vectors representing quantities like torque, velocity, or steering angles. Common in robotics and control tasks.  
- **Hybrid Action Space**: Combines discrete and continuous actions, sometimes handled by specialized algorithms.  
- **Action Constraints**: Physical or safety limits on actions (e.g., maximum torque). These must be incorporated into the environment or policy.  
- **Action Dimensionality**: Number of independent control variables in the action vector.  

---

## 🧰 Use Cases

- Discrete action spaces are common in grid-worlds, board games, and simplified robotic controllers.  
- Continuous action spaces are essential for robot joint control, autonomous driving, and drone flight.  
- Designing the action space properly helps reduce the learning difficulty and improves sample efficiency.  
- Action spaces influence the choice of policy representation and exploration strategy.  

---

## ✅ Pros and Cons

| Action Space Type | Pros                                           | Cons                                         |
|-------------------|------------------------------------------------|----------------------------------------------|
| Discrete          | Easier to model and learn; simpler algorithms | Limited expressiveness for real-world control |
| Continuous        | More realistic and flexible for control tasks | More complex learning; requires function approximation and advanced exploration |
| Hybrid            | Combines flexibility and discreteness          | Increased complexity in algorithm design     |

---

## 📊 Comparison Table: Algorithms vs Action Space Compatibility

| Algorithm                 | Action Space Type         | Notes                                         |
|---------------------------|--------------------------|-----------------------------------------------|
| DQN                       | Discrete                 | Works only with discrete actions               |
| PPO                       | Discrete & Continuous    | Flexible policy gradient method                |
| SAC                       | Continuous               | Off-policy algorithm suited for continuous control |
| TRPO                      | Discrete & Continuous    | Trust region method for stable policy updates  |
| Deep Deterministic Policy Gradient (DDPG) | Continuous   | Actor-critic method for continuous actions     |

---

## 🔗 Related Concepts

- [[Policy Gradient]] (Directly optimize policies over continuous or discrete action spaces)  
- [[Exploration vs Exploitation]] (Action selection strategies)  
- [[Action Masking]] (Restricting available actions in certain states)  
- [[State Space]] (Defines the environment observations related to actions)  
- [[Reward Function]] (Defines feedback for chosen actions)  

---

## 📚 Further Reading

- [Sutton & Barto – Reinforcement Learning: An Introduction (Chapter 3)](http://incompleteideas.net/book/the-book.html)  
- [OpenAI Spinning Up – Continuous vs Discrete Action Spaces](https://spinningup.openai.com/en/latest/spinningup/rl_intro3.html)  
- [Deep RL for Continuous Control](https://arxiv.org/abs/1509.02971)  
- [Hybrid Action Spaces in RL](https://arxiv.org/abs/1910.09127)  

---
