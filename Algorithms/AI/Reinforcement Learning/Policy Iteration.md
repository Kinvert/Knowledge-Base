# Policy Iteration

**Policy Iteration** is a classical algorithm in reinforcement learning and dynamic programming used to compute the optimal policy for a Markov Decision Process (MDP). It alternates between **policy evaluation** (estimating how good a policy is) and **policy improvement** (updating the policy based on the evaluation), until convergence.

---

## 🔍 Overview

- Introduced by Richard Bellman as part of his work on dynamic programming.  
- Guarantees convergence to the optimal policy in a finite number of steps for MDPs with a finite number of states and actions.  
- Each iteration improves the policy until the best possible one is found.  
- Forms the theoretical basis of many modern RL algorithms.  

---

## 🧠 Core Concepts

- **Policy Evaluation**:  
  Calculate the value function `V(s)` for the current policy `π` using the **Bellman Expectation Equation**.  
  This often requires iterative updates until convergence:
  `V(s) = E[ r + γ * V(s′) ]`  
- **Policy Improvement**:  
  Update the policy by acting greedily with respect to the current value function:  
  `π′(s) = argmax_a [ r + γ * V(s′) ]`  
- The new policy replaces the old one, and the cycle repeats.  
- Stops when the policy no longer changes—i.e., it's optimal.  

---

## 🧰 Use Cases

- Solving small to medium-sized MDPs when the environment's model (transition probabilities and rewards) is known.  
- Benchmarking and theoretical study of reinforcement learning algorithms.  
- Teaching and understanding the concept of policy evaluation and improvement.  

---

## ✅ Pros

- Converges to the optimal policy in finite steps.  
- More sample-efficient than some value-only methods like [[Value Iteration]] in small environments.  
- Clear and interpretable learning process.  

---

## ❌ Cons

- Requires access to a **complete model** of the environment (transition and reward functions).  
- Computationally expensive in large state/action spaces due to repeated evaluations.  
- Not suitable for model-free or online learning without modification.  

---

## 📊 Comparison Table: Policy Iteration vs Related Methods

| Algorithm         | Requires Model | Value Estimation | Policy Update       | Best Use Case           |
|-------------------|----------------|------------------|----------------------|--------------------------|
| Policy Iteration  | ✅ Yes         | Full evaluation  | Greedy improvement   | Small MDPs               |
| Value Iteration   | ✅ Yes         | Bootstrapped     | Simultaneous         | Faster convergence       |
| Q-Learning        | ❌ No          | Bootstrapped     | Off-policy           | Model-free RL            |
| SARSA             | ❌ No          | Bootstrapped     | On-policy            | Exploration-focused tasks|
| PPO               | ❌ No          | Approximate      | Gradient ascent      | Large/continuous spaces  |

---

## 🔧 Compatible Items

- [[Bellman Equation]] – Core to policy evaluation  
- [[Value Function]] – Evaluated and used to improve policy  
- [[MDP]] (Markov Decision Process) – The formal setting for Policy Iteration  
- [[Policy]] – The primary object being updated  
- [[Dynamic Programming]] – Category where Policy Iteration belongs  
- [[Value Iteration]] – Related but performs simultaneous updates  

---

## 🔗 Related Concepts

- [[Q-Learning]] (Model-free alternative using TD updates)  
- [[Policy Gradient]] (Optimization-based approach to policy learning)  
- [[Temporal Difference Learning]] (Basis for approximated evaluation)  
- [[Actor Critic]] (Combines policy improvement and value estimation)  
- [[Policy Evaluation]] – First step of Policy Iteration  

---

## 📚 Further Reading

- [Sutton & Barto – Chapter 4: Dynamic Programming](http://incompleteideas.net/book/the-book.html)  
- [David Silver's RL Course – Lecture 2](https://www.davidsilver.uk/teaching/)  
- [Wikipedia – Policy Iteration](https://en.wikipedia.org/wiki/Policy_iteration)  
- [CS50 AI: Policy Iteration](https://cs50.harvard.edu/ai/2020/weeks/7/)  

---
