# Q-Learning

**Q-Learning** is a foundational algorithm in reinforcement learning that enables agents to learn the optimal action-selection policy in a Markov Decision Process (MDP) by learning action-value pairs, or Q-values. It is an off-policy, model-free method that converges to the optimal policy under certain conditions.

---

## 🔍 Overview

- Q-Learning learns the optimal **Q-function**, `Q*(s, a)`, which represents the maximum expected future rewards obtainable from state `s` after taking action `a`.  
- The Q-values are updated iteratively using the **Bellman Equation**.  
- It is an **off-policy** algorithm, meaning it learns the optimal policy independently of the agent's current behavior.  
- Can be extended with function approximators such as neural networks (e.g., [[Deep Q Learning]]).  

---

## 🧠 Core Concepts

- **Q-Value (Action-Value Function)**: Expected cumulative reward of taking action `a` in state `s` and following the optimal policy.  
- **Bellman Update Rule**:  
  `Q(s, a) ← Q(s, a) + α * (r + γ * max(Q(s′, a′)) − Q(s, a))`  
- **Learning Rate (α)**: Controls how much new information overrides old.  
- **Discount Factor (γ)**: How much future rewards are worth relative to immediate ones.  
- **Exploration vs Exploitation**: Often handled with `ε-greedy` policies.  

---

## 🧰 Use Cases

- Simple environments with discrete state and action spaces.  
- Grid worlds, navigation tasks, turn-based games, and educational demos.  
- Foundation for more advanced methods like [[DQN]], [[Double DQN]], and [[SARSA]].  
- Algorithmic baseline for comparing performance of RL methods.  

---

## ✅ Pros

- Guaranteed to converge to the optimal policy under appropriate learning conditions.  
- Does not require a model of the environment (model-free).  
- Conceptually simple and easy to implement.  
- Efficient for small, discrete environments.  

---

## ❌ Cons

- Poor scalability to high-dimensional or continuous state/action spaces.  
- Requires a table of size |S| × |A| — infeasible for large problems.  
- Slower learning compared to some policy-based or model-based methods.  
- Sensitive to hyperparameters like α, γ, and ε.  

---

## 📊 Comparison Table: Q-Learning vs Related Algorithms

| Algorithm      | Type         | On/Off-Policy | Value or Policy-Based | Suitable for           |
|----------------|--------------|---------------|------------------------|-------------------------|
| Q-Learning     | Tabular      | Off-policy    | Value-based            | Discrete action/state   |
| SARSA          | Tabular      | On-policy     | Value-based            | Discrete action/state   |
| DQN            | Neural Net   | Off-policy    | Value-based            | High-dimensional input  |
| PPO            | Neural Net   | On-policy     | Policy-based           | Continuous control      |
| Actor Critic   | Neural Net   | Mixed         | Policy + Value         | Continuous/Discrete     |

---

## 🔧 Compatible Items

- [[Temporal Difference Learning]] – Q-learning is a TD method  
- [[ε-greedy Policy]] – Common exploration strategy  
- [[Q-Value Function]] – The learned value being updated  
- [[DQN]] – Deep learning extension of Q-learning  
- [[Replay Buffer]] – Needed for stability in deep variants  
- [[Bellman Equation]] – Fundamental to the Q-value update  

---

## 🔗 Related Concepts

- [[Policy Gradient]] (Contrasts with value-based methods like Q-Learning)  
- [[SARSA]] (Similar but on-policy)  
- [[Value Function]] (Estimated through Q(s, a))  
- [[State Space]] and [[Action Space]] (Fundamental RL definitions)  
- [[Markov Decision Process]] (Problem formulation for Q-Learning)  

---

## 📚 Further Reading

- [Reinforcement Learning: An Introduction – Sutton & Barto (Chapter 6)](http://incompleteideas.net/book/the-book.html)  
- [OpenAI Spinning Up: Q-Learning](https://spinningup.openai.com/en/latest/algorithms/q_learning.html)  
- [CS50 AI: Q-Learning Lecture](https://cs50.harvard.edu/ai/2020/weeks/7/)  
- [David Silver's RL Course (Lecture 4)](https://www.davidsilver.uk/teaching/)  

---
