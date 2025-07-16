# Value Function

A **Value Function** in reinforcement learning represents the expected return (cumulative reward) an agent can achieve from a given state or state-action pair under a certain policy. It is a core concept in RL, helping agents decide which actions to take by estimating long-term outcomes.

---

## 🔍 Overview

- The value function quantifies **how good** it is to be in a state or to take a certain action in a state.  
- Two main types:
  - **State-value function `V(s)`**: Expected return from state `s` following policy π.  
  - **Action-value function `Q(s, a)`**: Expected return from taking action `a` in state `s`, then following π.  
- Central to **value-based methods** like [[Q-Learning]], [[SARSA]], and [[Dynamic Programming]].

---

## 🧠 Core Concepts

- **State-Value Function**:  
  `Vπ(s) = Eπ[ ∑ γ^t * r_t | s_0 = s ]`  
- **Action-Value Function**:  
  `Qπ(s, a) = Eπ[ ∑ γ^t * r_t | s_0 = s, a_0 = a ]`  
- **Discount Factor (γ)**: A number in [0,1) that reduces the weight of future rewards.  
- Value functions can be:
  - **Tabular**: Stored explicitly for small environments.  
  - **Approximated**: With linear models or neural networks for larger environments.

---

## 🧰 Use Cases

- Used in policy evaluation and improvement steps in classical RL algorithms like [[Policy Iteration]] and [[Value Iteration]].  
- Guides exploration and exploitation in model-free methods.  
- Forms the critic in [[Actor Critic]] architectures.  
- Basis for bootstrapping in [[TD Learning]] and other learning strategies.

---

## ✅ Pros

- Allows the agent to make **long-term decisions** instead of greedy, short-sighted choices.  
- Enables bootstrapping (predicting from predictions) to speed up learning.  
- Efficient with proper generalization in large or continuous domains.

---

## ❌ Cons

- Requires accurate estimation, which is challenging with function approximation.  
- Incorrect value estimates can misguide policies.  
- Can lead to instability when combined with non-linear approximators like deep networks.

---

## 📊 Comparison Table: V(s) vs Q(s, a)

| Property                  | V(s)                       | Q(s, a)                     |
|---------------------------|----------------------------|-----------------------------|
| Describes                 | Value of a state           | Value of a state-action pair|
| Policy Evaluation         | ✅ Yes                     | ✅ Yes                      |
| Used in Policy Improvement| ✅ Indirectly              | ✅ Directly                 |
| Needed by Actor-Critic    | ✅ Critic                  | ✅ Actor/Critic             |
| Table Size (tabular)      | |S|                        | |S| × |A|                   |

---

## 🔧 Compatible Items

- [[Bellman Equation]] – Defines the recursive relationship for value functions  
- [[Q-Learning]] – Learns the Q-value function  
- [[Policy Iteration]] – Uses V(s) and policy improvement  
- [[Actor Critic]] – Separates policy (actor) and value (critic)  
- [[Temporal Difference Learning]] – Updates value estimates online  

---

## 🔗 Related Concepts

- [[Q-Value Function]] – A specific type of value function  
- [[TD Learning]] – Core method for value function updates  
- [[RL Policy]] – Value function is defined under a policy  
- [[MDP]] (Markov Decision Process) – The environment where value functions apply  
- [[Function Approximation]] – Common for estimating V(s) or Q(s, a) in large domains  

---

## 📚 Further Reading

- [Sutton & Barto – Chapter 3: The Value Function](http://incompleteideas.net/book/the-book.html)  
- [David Silver’s RL Course – Lecture 2](https://www.davidsilver.uk/teaching/)  
- [Spinning Up – Introduction to RL](https://spinningup.openai.com/en/latest/spinningup/rl_intro.html)  
- [CS50 AI – Value Functions](https://cs50.harvard.edu/ai/2020/weeks/7/)

---
