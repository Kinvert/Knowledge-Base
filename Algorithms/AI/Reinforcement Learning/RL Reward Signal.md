# RL Reward Signal

In **Reinforcement Learning (RL)**, the *reward signal* is the feedback received from the environment in response to the agent's actions. It serves as the fundamental learning signal, guiding the agent to develop a policy that maximizes cumulative reward over time.

Rewards can be sparse or dense, positive or negative, and immediate or delayed—each form shaping the agent’s behavior differently.

---

## 📚 Overview

At each timestep `t`, after the agent takes an action `a_t` in state `s_t`, it receives a **reward** `r_{t+1}` from the environment:

- Scalar value
- Indicates the desirability of the action’s outcome
- Used to calculate the **return**, which influences learning

The design and shaping of the reward signal play a crucial role in learning efficiency, convergence, and policy quality.

---

## 🧠 Core Concepts

- `Reward (r_t)`: Scalar signal for agent’s immediate action outcome  
- `Return (G_t)`: Discounted cumulative sum of rewards  
- `Sparse Reward`: Few or infrequent rewards (e.g. +1 only at success)  
- `Dense Reward`: Frequent feedback (e.g. every timestep)  
- `Reward Shaping`: Adjusting reward to guide learning more efficiently  
- `Delayed Reward`: Consequences felt several steps later  

---

## 🧰 Use Cases

- Training agents in goal-based tasks (e.g. pick-and-place)  
- Measuring task success in navigation and control systems  
- Shaping behavior in simulated environments or games  
- Multi-agent competition/cooperation using shared or individual rewards  
- Real-world reward feedback (e.g., sensor triggers, success flags)  

---

## ✅ Pros

- Intuitive mechanism for guiding learning  
- Simple and efficient signal structure  
- Supports a wide range of learning algorithms  
- Encourages reward-maximizing behavior  

---

## ❌ Cons

- Poorly designed rewards can lead to unintended behaviors  
- Sparse rewards slow learning and exploration  
- Reward hacking can occur if agents exploit loopholes  
- Delayed rewards complicate credit assignment  

---

## 📊 Comparison Table: Reward Types

| Reward Type       | Description                                      | Example                               | Pros                        | Cons                         |
|-------------------|--------------------------------------------------|----------------------------------------|-----------------------------|------------------------------|
| Sparse            | Given only at terminal or critical events        | +1 if robot reaches goal               | Simple design               | Hard to learn                |
| Dense             | Frequent, continuous feedback                    | -0.1 for each timestep not at goal     | Faster convergence          | May bias toward trivial policies |
| Shaped            | Manually engineered for smoother learning        | +0.1 for getting closer to goal        | Better guidance             | Risk of overfitting behavior |
| Delayed           | Comes many steps after action                    | Reward after task completion           | Fits real-world scenarios   | Hard credit assignment       |

---

## 🤖 In Robotics Context

| Scenario                 | Reward Signal Example                        |
|--------------------------|----------------------------------------------|
| Pick-and-place task      | +1 if object placed correctly                |
| Navigation               | -1 per collision, +1 for reaching target     |
| Drone flight             | +reward for maintaining altitude             |
| Human-robot interaction  | +1 when user gives positive feedback         |
| Warehouse automation     | -0.1 delay penalty, +1 for correct binning   |

---

## 🔧 Compatible Items

- [[Reinforcement Learning]] – Reward is the key learning signal  
- [[RL Return]] – Built from rewards  
- [[RL Agent]] – Optimizes behavior based on reward  
- [[Reward Shaping]] – Engineering reward for better learning  
- [[RL Episode]] – Contains sequence of rewards  
- [[RL Environment]] – Provides the reward to the agent  

---

## 🔗 Related Concepts

- [[Reward Shaping]] (Engineering intermediate rewards)  
- [[Inverse Reinforcement Learning]] (Inferring reward from behavior)  
- [[Sparse Rewards]] (Delayed or infrequent feedback)  
- [[RL Return]] (Cumulative form of rewards)  
- [[Credit Assignment Problem]] (Attributing outcomes to actions)  

---

## 📚 Further Reading

- [Sutton & Barto – Rewards and Returns](http://incompleteideas.net/book/the-book.html)  
- [Spinning Up – Reward Design](https://spinningup.openai.com/en/latest/spinningup/rl_intro.html#rewards)  
- [Reward Shaping Survey](https://arxiv.org/abs/1906.11468)  
- [OpenAI Blog: Reward Hacking](https://openai.com/blog/faulty-reward-functions/)  
- [DeepMind: Sparse Reward Techniques](https://deepmind.com/blog/article/solving-rubiks-cube)  

---
