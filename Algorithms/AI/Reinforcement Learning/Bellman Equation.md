# Bellman Equation

The **Bellman Equation** is a fundamental recursive relationship in reinforcement learning and dynamic programming. It describes how the value of a decision-making policy can be decomposed into immediate rewards and the value of future states. It serves as the theoretical backbone of many RL algorithms, including [[Q-Learning]], [[Value Iteration]], and [[Policy Iteration]].

---

## 🔍 Overview

- Named after **Richard Bellman**, a pioneer of dynamic programming.  
- Provides a way to compute the **value function** of a policy by breaking it into subproblems.  
- Used to derive both **state-value functions (V)** and **action-value functions (Q)**.  
- Central to many RL algorithms for updating estimates during training.  

---

## 🧠 Core Concepts

- **State-Value Function (V)**:  
  `V(s) = E[ r + γ * V(s′) ]` under a given policy π  
- **Action-Value Function (Q)**:  
  `Q(s, a) = E[ r + γ * max(Q(s′, a′)) ]` for the optimal policy  
- **Recursive Decomposition**: Each value is expressed in terms of future values.  
- **Expectation**: The expected value is taken over all possible next states and rewards.  
- **Bellman Optimality Equation**: Provides a way to compute the optimal value function using dynamic programming.

---

## 🧰 Use Cases

- Solving finite Markov Decision Processes (MDPs).  
- Deriving value-based reinforcement learning methods like [[Q-Learning]] and [[DQN]].  
- Theoretical foundation for understanding the behavior of RL algorithms.  
- Used in value iteration, policy iteration, and other planning algorithms.  

---

## ✅ Pros

- Enables recursive computation of complex value functions.  
- Forms the foundation for convergence guarantees in many RL algorithms.  
- Applies to both model-based and model-free reinforcement learning.  

---

## ❌ Cons

- Requires a model of the environment to compute expectations exactly.  
- In large or continuous spaces, exact computation is infeasible.  
- Approximations (e.g., with neural networks) introduce complexity and instability.  

---

## 📊 Comparison Table: Bellman Equation Use in Algorithms

| Algorithm        | Uses Bellman Equation | Type              | Requires Environment Model |
|------------------|------------------------|-------------------|-----------------------------|
| Q-Learning       | ✅ Yes                 | Model-free        | ❌ No                       |
| SARSA            | ✅ Yes                 | Model-free        | ❌ No                       |
| Value Iteration  | ✅ Yes                 | Model-based       | ✅ Yes                      |
| Policy Iteration | ✅ Yes                 | Model-based       | ✅ Yes                      |
| DQN              | ✅ Yes                 | Model-free (deep) | ❌ No                       |

---

## 🔧 Compatible Items

- [[Q-Learning]] – Uses Bellman equation for Q-value updates  
- [[TD Learning]] – Bootstraps using Bellman-derived targets  
- [[MDP]] (Markov Decision Process) – Bellman equations are defined over MDPs  
- [[Value Function]] – Computed recursively using the Bellman equation  
- [[Dynamic Programming]] – The Bellman equation is its core technique  

---

## 🔗 Related Concepts

- [[Temporal Difference Learning]] (Uses Bellman targets for updates)  
- [[Monte Carlo Methods]] (Contrasted with Bellman-based bootstrapping)  
- [[Policy Gradient]] (Does not rely on Bellman equation directly)  
- [[State Space]], [[Action Space]] – The domains where values are computed  

---

## 📚 Further Reading

- [Sutton & Barto – Chapter 3: The Bellman Equation](http://incompleteideas.net/book/the-book.html)  
- [David Silver’s RL Course – Lecture 3](https://www.davidsilver.uk/teaching/)  
- [CS50 AI – Bellman Equation Intro](https://cs50.harvard.edu/ai/2020/weeks/7/)  
- [Wikipedia: Bellman Equation](https://en.wikipedia.org/wiki/Bellman_equation)  

---
