# Temporal Difference (TD) Learning

**Temporal Difference (TD) Learning** is a foundational method in reinforcement learning that blends ideas from **Monte Carlo methods** and **dynamic programming**. It allows agents to learn value functions directly from experience, without a full model of the environment, and without waiting for the final outcome of an episode.

---

## 🔍 Overview

- TD learning estimates the value of a state by **bootstrapping**—updating predictions based on other learned predictions.  
- Unlike Monte Carlo methods, TD learning can update estimates after every step instead of waiting for an episode to finish.  
- Core to many popular RL algorithms like [[Q-Learning]], [[SARSA]], and [[Actor Critic]].  
- Enables **online learning** in real-time environments, making it suitable for robotics and games.  

---

## 🧠 Core Concepts

- **Bootstrapping**: Updating an estimate based partly on other learned estimates.  
- **TD Target**:  
  `TD Target = r + γ * V(s′)`  
  where `r` is the reward, `γ` the discount factor, and `V(s′)` the estimated value of the next state.  
- **TD Error (δ)**:  
  `δ = TD Target − V(s)`  
  This error drives the update to the value function.  
- **Update Rule** (for state value):  
  `V(s) ← V(s) + α * δ`  
- **n-step TD**: Uses rewards from multiple steps to form the target.  
- **TD(λ)**: A generalization combining n-step methods using eligibility traces.  

---

## 🧰 Use Cases

- Learning value functions online from real-time sensor data.  
- Environments where episodes are long or continuous.  
- Robotics, video games, and any domain where step-wise feedback is available.  
- Forming the basis of value-based methods like [[Q-Learning]], [[SARSA]], and [[Actor Critic]].  

---

## ✅ Pros

- Learns directly from raw experience without requiring a model.  
- More sample-efficient than Monte Carlo methods.  
- Works well in online and incremental settings.  
- Can be combined with function approximation (e.g., neural nets).  

---

## ❌ Cons

- Bootstrapping can introduce bias.  
- Sensitive to hyperparameters like learning rate (α) and discount factor (γ).  
- Less stable than Monte Carlo in some scenarios, especially early in training.  

---

## 📊 Comparison Table: TD vs Monte Carlo vs Dynamic Programming

| Property                   | Temporal Difference (TD) | Monte Carlo           | Dynamic Programming     |
|----------------------------|---------------------------|------------------------|--------------------------|
| Updates During Episode     | ✅ Yes                    | ❌ No                  | ❌ No                    |
| Needs Environment Model    | ❌ No                     | ❌ No                  | ✅ Yes                   |
| Variance                   | Low                       | High                   | Low                      |
| Bias                       | High                      | None                   | None (with exact model)  |
| Applicability to RL        | ✅ Broad                  | ✅ Broad               | ❌ Limited               |

---

## 🔧 Compatible Items

- [[TD Learning]] – They are the same
- [[Q-Learning]] – A TD method that learns action-value functions off-policy  
- [[SARSA]] – A TD method that learns on-policy  
- [[TD Error]] – The core update signal in TD learning  
- [[Value Function]] – Learned and updated using TD methods  
- [[Actor Critic]] – Uses TD error to update both actor and critic  
- [[Eligibility Traces]] – Enhance TD learning via TD(λ)  

---

## 🔗 Related Concepts

- [[Reinforcement Learning]] (Core category including TD methods)  
- [[Monte Carlo Methods]] (Complementary approach to TD)  
- [[Policy Evaluation]] (TD is often used for this purpose)  
- [[Function Approximation]] (TD can work with neural nets, linear functions, etc.)  
- [[Bellman Equation]] (TD target is derived from this)  

---

## 📚 Further Reading

- [Sutton & Barto – Chapter 6: Temporal Difference Learning](http://incompleteideas.net/book/the-book.html)  
- [Spinning Up – Value Learning](https://spinningup.openai.com/en/latest/spinningup/rl_intro2.html#value-learning)  
- [CS50 RL – Temporal Difference Learning Explained](https://cs50.harvard.edu/ai/2020/weeks/7/)  
- [TD(λ) and Eligibility Traces (David Silver)](https://www.davidsilver.uk/teaching/)  

---

## 🗂 Suggested Folder Location

Software > Reinforcement Learning > Core Concepts > Temporal Difference Learning  
