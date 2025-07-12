# TD Learning

**Temporal Difference (TD) Learning** is a core concept in **Reinforcement Learning (RL)** that blends ideas from **Monte Carlo methods** and **Dynamic Programming**. It enables agents to learn directly from raw experience without needing a model of the environment and without waiting for episodes to terminate.

TD Learning updates value estimates based on the difference between predicted and actually observed rewards — the **TD Error**.

---

## 📚 Overview

TD Learning updates the value of a state `s_t` using the reward received after taking an action and the estimated value of the next state `s_{t+1}`:

`V(s_t) ← V(s_t) + α [r_{t+1} + γ V(s_{t+1}) - V(s_t)]`

Where:
- `α` is the learning rate
- `γ` is the discount factor
- The term inside brackets is the **TD Error**

Unlike Monte Carlo methods, TD can update values after a single step, making it efficient and suitable for **online learning**.

---

## 🧠 Core Concepts

- `TD Error`: Difference between predicted and updated value  
- `Bootstrapping`: Update based on estimated future values  
- `TD(0)`: One-step lookahead  
- `TD(λ)`: Blends TD and Monte Carlo with eligibility traces  
- `Online Learning`: Updates occur after every transition  
- `Off-policy`: Variants like Q-learning use TD off-policy  

---

## 🧰 Use Cases

- Real-time learning in robotics and simulation  
- Learning value functions for actor-critic models  
- Improving sample efficiency over Monte Carlo methods  
- Updating target networks in deep RL algorithms  
- Adaptive control systems with delayed outcomes  

---

## ✅ Pros

- Updates without needing full episodes  
- Supports online and incremental learning  
- More sample-efficient than Monte Carlo  
- Basis for many practical RL algorithms  

---

## ❌ Cons

- Introduces bias through bootstrapping  
- May require careful tuning of learning rate and discount  
- Can propagate inaccurate estimates if not handled properly  
- Less stable with non-linear function approximation  

---

## 📊 Comparison Table: TD vs Monte Carlo vs DP

| Method         | Bootstraps | Needs Episodes | Model-Based | Sample Efficiency | Bias        | Variance    |
|----------------|------------|----------------|--------------|-------------------|-------------|-------------|
| TD Learning    | Yes        | No             | No           | High              | Moderate    | Low         |
| Monte Carlo    | No         | Yes            | No           | Low               | None        | High        |
| Dynamic Programming | Yes  | No             | Yes          | N/A (planning)    | None        | Low         |

---

## 🤖 In Robotics Context

| Task                  | TD Learning Role                                       |
|------------------------|--------------------------------------------------------|
| Arm control            | Online value updates after each joint movement         |
| Navigation             | Learn better paths from short episodes                 |
| Grasp learning         | Use TD error to refine success prediction              |
| Sim-to-real transfer   | Improve learning from transitions mid-trajectory       |
| Continuous control     | Works well with actor-critic or DDPG methods           |

---

## 🔧 Compatible Items

- [[Reinforcement Learning]] – TD is a foundational learning strategy  
- [[RL Value Function]] – TD updates value predictions  
- [[Actor Critic]] – Critic often uses TD updates  
- [[Q-Learning]] – A form of off-policy TD learning  
- [[Advantage Function]] – TD often helps estimate advantages  
- [[TD(λ)]] – Extends TD with eligibility traces  

---

## 🔗 Related Concepts

- [[Monte Carlo Methods]] (Alternative to TD for value learning)  
- [[Eligibility Traces]] (Used in TD(λ))  
- [[Q-Learning]] (TD-based action value learning)  
- [[SARSA]] (On-policy TD algorithm)  
- [[Bootstrapping]] (Core mechanism in TD)  

---

## 📚 Further Reading

- [Sutton & Barto – Chapter 6: TD Learning](http://incompleteideas.net/book/the-book.html)  
- [Spinning Up – TD vs MC](https://spinningup.openai.com/en/latest/spinningup/rl_intro.html#td-learning)  
- [TD(λ) Paper](https://www.cs.toronto.edu/~vmnih/docs/tdlambda.pdf)  
- [DeepMind’s Use of TD in DQN](https://www.nature.com/articles/nature14236)  

---
