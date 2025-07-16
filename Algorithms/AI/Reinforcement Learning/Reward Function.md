# Reward Function

A **Reward Function** is a core component of a reinforcement learning (RL) system that provides **feedback** to an agent based on its actions in the environment. It guides the learning process by assigning scalar values (rewards) to transitions, influencing how the agent updates its **policy** and makes future decisions.

Designing an effective reward function is often one of the most challenging and critical parts of developing a successful RL system.

---

## 🧠 Overview

The reward function defines `r(s, a, s')`, where:
- `s` = current state
- `a` = action taken
- `s'` = resulting state

In episodic tasks, the **return** is the cumulative reward over time. The agent's objective is to learn a policy that maximizes expected return.

Reward signals can be:
- **Sparse**: Only given in rare conditions (e.g., task completion)
- **Dense**: Frequent, often heuristic (e.g., small penalty each step)
- **Shaped**: Augmented with auxiliary rewards to guide exploration

---

## 🧪 Use Cases

- Game AI: Positive reward for scoring, negative for losing  
- Robotics: Reward for reaching a goal position or minimizing energy  
- Multi-agent RL: Shared vs. individual rewards (cooperation vs. competition)  
- Autonomous driving: Reward for safety, speed, and smoothness  
- Industrial systems: Optimization of throughput or cost efficiency

---

## 📊 Comparison Table

| Reward Type     | Description                        | Example                     | Tradeoff                           |
|------------------|------------------------------------|------------------------------|-------------------------------------|
| Sparse           | Only given on success/failure      | Maze exit gives +1          | Hard to learn, but clean signal     |
| Dense            | Frequent incremental feedback      | +0.1 every meter moved      | Faster learning, risk of bias       |
| Shaped           | Engineered reward components       | Distance-to-goal penalty    | Helps exploration, may mislead      |
| Extrinsic        | Provided by environment            | Winning a game              | Task-specific                       |
| Intrinsic        | Generated internally (e.g., curiosity) | Novelty bonus              | Aids exploration, complex to tune   |

---

## ⚙️ Key Concepts

- **Reward Signal**: Actual value given to agent  
- **Return**: Sum of discounted future rewards  
- **Discount Factor (γ)**: Controls tradeoff between short and long-term rewards  
- **Reward Shaping**: Adding auxiliary rewards for guidance  
- **Sparse vs Dense Rewards**: Tradeoff between clarity and trainability

---

## ✅ Pros of Reward-Based Learning

- General-purpose: Applicable across domains  
- Scalable with deep learning models  
- Enables end-to-end behavior learning  
- Works with minimal supervision

---

## ❌ Challenges

- Poorly designed rewards can result in unintended behavior  
- Sparse rewards slow down learning  
- Overly shaped rewards may limit generalization  
- Difficult to balance competing objectives (e.g., speed vs. safety)

---

## 🔗 Related Concepts

- [[RL Reward Signal]]  
- [[RL Return]]  
- [[RL Policy]]  
- [[Value Function]]  
- [[TD Learning]]  
- [[Advantage Function]]  
- [[Exploration vs Exploitation]]  
- [[Reward Shaping]]  
- [[MDP]] (Markov Decision Process)  
- [[RL Environment]]

---

## 📚 Further Reading

- [Reward Shaping in Reinforcement Learning (Ng et al.)](https://papers.nips.cc/paper_files/paper/1999/file/2e2675f5b46c98a364e3f1c4c4e7535f-Paper.pdf)  
- [DeepMind's Reward Design Principles](https://deepmind.com/blog/article/specification-gaming-the-flip-side-of-ai-ingenuity)  
- RL frameworks: [CleanRL], [Stable-Baselines3], [Ray RLlib] – all allow custom reward functions

---
