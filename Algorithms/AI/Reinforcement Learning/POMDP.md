# POMDP

A **POMDP (Partially Observable Markov Decision Process)** extends the classic [[MDP]] (Markov Decision Process) framework by accounting for situations where the agent cannot directly observe the true state of the environment. Instead, it must make decisions based on indirect, noisy, or incomplete observations.

---

## 🔍 Overview

- POMDPs model uncertainty in both **state transitions** and **state observability**.  
- Used when sensors are unreliable or when some parts of the system state are hidden.  
- Common in robotics, dialogue systems, and planning under uncertainty.  
- Adds a **belief state**—a probability distribution over possible states.  

---

## 🧠 Core Concepts

- **States (S)**: The true underlying states of the environment.  
- **Actions (A)**: Choices the agent can make.  
- **Observations (O)**: Noisy or partial views of the state.  
- **Observation Function (Ω)**: `P(o | s′, a)` gives the probability of observing `o` after taking action `a` and landing in state `s′`.  
- **Transition Function (T)**: `P(s′ | s, a)` like in regular MDPs.  
- **Reward Function (R)**: `R(s, a, s′)` assigns rewards as usual.  
- **Belief State (b)**: A probability distribution over possible states, updated via Bayes’ rule after each action-observation cycle.  
- **Policy (π)**: Maps belief states to actions instead of true states.

---

## 🧰 Use Cases

- Robotics with noisy sensors (e.g. vision, sonar).  
- Automated assistants and dialogue systems.  
- Medical diagnosis systems where symptoms (observations) don't fully reveal the condition (state).  
- Exploration problems with occlusions or latent variables.

---

## ✅ Pros

- Models **realistic conditions** with uncertainty.  
- Enables **robust planning** in partially known environments.  
- Can exploit uncertainty to **gather better information** (active sensing).  

---

## ❌ Cons

- Significantly more computationally expensive than [[MDP]]s.  
- Solving exactly is intractable for large problems.  
- Requires good models for observation and transition probabilities.  
- Complex to implement and understand.  

---

## 📊 Comparison Table: MDP vs POMDP

| Feature               | MDP                          | POMDP                             |
|------------------------|-------------------------------|------------------------------------|
| State Observability   | Fully observable             | Partially observable               |
| Decision Basis        | Current state                | Belief over states                 |
| Complexity            | Moderate                     | High                               |
| Used in RL?           | ✅ Yes                        | ✅ Yes                              |
| Common Use Case       | Structured environments       | Uncertain, sensor-limited settings |

---

## 🔧 Compatible Items

- [[Belief State]] – Internal probabilistic state representation  
- [[Bayes Filter]] – Used to update the belief state  
- [[Sensor Fusion]] – May help reduce observability gaps  
- [[RL Agent]] – Needs modification to operate in a POMDP  
- [[Policy]] – Becomes belief-state dependent  

---

## 🔗 Related Concepts

- [[MDP]] (Markov Decision Process) – The base model for POMDPs  
- [[Reinforcement Learning]] – Many RL algorithms adapt to partial observability  
- [[State Estimation]] – Core technique in handling POMDPs  
- [[Hidden Markov Model]] – Similar idea, but no actions  
- [[Kalman Filter]] – A belief state estimator in linear systems  

---

## 📚 Further Reading

- [Sutton & Barto – Chapter 17: POMDPs](http://incompleteideas.net/book/the-book.html)  
- [Wikipedia: Partially Observable Markov Decision Process](https://en.wikipedia.org/wiki/Partially_observable_Markov_decision_process)  
- [POMDP.org](http://www.pomdp.org/)  
- [Kaelbling et al. "Planning and Acting in Partially Observable Stochastic Domains"](https://www.cs.rutgers.edu/~mlittman/papers/pomdp-survey.pdf)  

---
