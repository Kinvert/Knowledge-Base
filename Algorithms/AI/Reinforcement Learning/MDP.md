# MDP

A **Markov Decision Process (MDP)** is the mathematical framework that formalizes the environment in reinforcement learning. It models decision-making situations where outcomes are partly random and partly under the control of an agent. Most RL algorithms assume the environment behaves as an MDP.

---

## 🔍 Overview

- An MDP provides a structured way to describe the dynamics of environments using a tuple:  
  `(S, A, P, R, γ)`  
- The **Markov property** means the future state depends only on the current state and action, not on the history.  
- RL problems are typically modeled as MDPs to enable theoretical guarantees and algorithm design.

---

## 🧠 Core Concepts

- **States (S)**: All possible configurations of the environment.  
- **Actions (A)**: All possible decisions the agent can take.  
- **Transition Probability (P)**: `P(s′ | s, a)` is the probability of moving to state `s′` from state `s` by taking action `a`.  
- **Reward Function (R)**: `R(s, a, s′)` gives the immediate reward received when transitioning.  
- **Discount Factor (γ)**: Determines how much future rewards are worth compared to immediate ones.  
- **Policy (π)**: A strategy that defines the agent’s behavior, mapping states to actions.

---

## 🧰 Use Cases

- Underlies all classical and modern RL algorithms.  
- Applicable in domains such as robotics, game-playing, recommendation systems, and finance.  
- Forms the basis for policy evaluation, value iteration, and dynamic programming.  

---

## ✅ Pros

- Provides a clean mathematical structure for modeling decision-making.  
- Supports convergence and optimality guarantees.  
- Modular — can be extended to [[POMDP]] (Partially Observable MDPs), continuous spaces, and stochastic policies.  

---

## ❌ Cons

- Real-world problems may violate the Markov assumption.  
- Requires full knowledge of transition and reward functions for model-based methods.  
- State and action spaces can grow exponentially, making exact solutions intractable.  

---

## 📊 Comparison Table: MDP vs Other Formulations

| Framework      | Observability | Transition Model | Common Use Case         |
|----------------|----------------|------------------|--------------------------|
| [[MDP]]        | Full           | Known or unknown | Standard RL setup        |
| POMDP          | Partial        | Known/unknown    | Noisy sensors, partial info |
| Bandit         | N/A            | No state         | Single-step decisions    |
| Markov Chain   | Full           | Known            | No agent actions         |

---

## 🔧 Compatible Items

- [[Bellman Equation]] – Defined over MDPs  
- [[Q-Learning]] – Learns optimal policy from MDP interaction  
- [[Policy Iteration]] – Iteratively solves MDPs  
- [[Value Function]] – Describes expected return in an MDP  
- [[State Space]], [[Action Space]] – Core components of MDP definition  

---

## 🔗 Related Concepts

- [[RL Policy]] – The agent’s behavior in an MDP  
- [[Reward Signal]] – Defined in the MDP as part of `R(s, a, s′)`  
- [[RL Transition]] – The probabilistic change of state  
- [[TD Learning]] – Learns value functions under MDP assumptions  
- [[Exploration vs Exploitation]] – Key behavior in interacting with MDPs  

---

## 📚 Further Reading

- [Sutton & Barto – Chapter 3: The MDP](http://incompleteideas.net/book/the-book.html)  
- [David Silver’s RL Course – Lecture 2](https://www.davidsilver.uk/teaching/)  
- [Spinning Up – RL Foundations](https://spinningup.openai.com/en/latest/spinningup/rl_intro.html#mdps)  
- [Wikipedia: Markov Decision Process](https://en.wikipedia.org/wiki/Markov_decision_process)  

---
