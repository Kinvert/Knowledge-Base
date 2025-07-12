# Actor Critic

**Actor-Critic** is a class of algorithms in **Reinforcement Learning (RL)** that combines the strengths of both value-based and policy-based methods. It simultaneously maintains two components:
- **Actor**: The policy function that selects actions
- **Critic**: The value function that evaluates the actor’s actions

By using the critic’s feedback to improve the actor, the system can learn more efficiently and with reduced variance compared to pure policy gradient methods.

---

## 📚 Overview

The actor updates the policy parameters in the direction suggested by the critic, which estimates the *value* or *advantage* of the current policy's actions.

- The **actor** is updated using gradients: `∇ log π(a|s) * A(s, a)`
- The **critic** is updated by minimizing the value or TD error

This architecture forms the basis of many modern RL algorithms like A2C, A3C, PPO, and DDPG.

---

## 🧠 Core Concepts

- `Actor`: Learns the policy π(a|s) that decides actions  
- `Critic`: Estimates value function V(s) or Q(s, a) to evaluate the actor  
- `Advantage Function`: Guides how much better an action is than average  
- `Policy Gradient`: The learning rule for the actor  
- `Temporal-Difference (TD) Learning`: Used by the critic to update value estimates  
- `Bootstrapping`: Critic estimates based on future value predictions  
- `On-policy vs Off-policy`: Many actor-critic methods can support both

---

## 🧰 Use Cases

- Robotics (continuous control, e.g. joint angle regulation)  
- Sim2Real transfer (actor learns in simulation, then transfers)  
- Game playing (actor optimizes action selection, critic stabilizes learning)  
- Autonomous vehicles (smooth navigation policy learning)  
- Continuous action space tasks where discrete Q-learning fails  

---

## ✅ Pros

- Reduces variance compared to vanilla policy gradient  
- Supports continuous action spaces  
- Can leverage bootstrapping (TD learning) for sample efficiency  
- Flexible for on-policy or off-policy learning  
- Modular (easily replace actor or critic independently)  

---

## ❌ Cons

- More complex to implement and tune  
- Sensitive to learning rate and critic accuracy  
- Critic approximation error can degrade performance  
- Requires additional computational resources  

---

## 📊 Comparison Table: Actor-Critic vs Other RL Methods

| Method             | Action Space       | Uses Value Function | Policy Directly Optimized | Sample Efficiency | Common Use Case               |
|--------------------|--------------------|----------------------|---------------------------|--------------------|-------------------------------|
| Q-Learning         | Discrete           | Yes (Q)              | No                        | High               | Gridworld, simple games       |
| Policy Gradient    | Discrete/Continuous| No                   | Yes                       | Low                | Basic control tasks           |
| Actor-Critic       | Discrete/Continuous| Yes (V or Q)         | Yes                       | Medium             | Robotics, high-dimensional RL |
| DDPG / TD3         | Continuous         | Yes (Q)              | Yes (deterministic policy)| High               | Complex control environments  |
| PPO / A2C          | Both               | Yes (Advantage)      | Yes                       | Medium             | Stable, general-purpose RL    |

---

## 🤖 In Robotics Context

| Task                     | Role of Actor-Critic                                  |
|--------------------------|--------------------------------------------------------|
| Arm manipulation         | Actor decides torque, critic evaluates grasp outcome   |
| Bipedal walking          | Actor chooses joint movements, critic tracks stability |
| Path planning            | Actor generates waypoints, critic assesses cost        |
| Multi-agent swarm        | Each agent has local actor-critic pair                 |
| Sim2Real movement        | Trained in Isaac Gym or PyBullet, deployed to hardware |

---

## 🔧 Compatible Items

- [[Reinforcement Learning]] – Actor-critic is a hybrid approach  
- [[RL Value Function]] – Used by critic to evaluate actions  
- [[Advantage Function]] – Often computed to reduce variance  
- [[Policy Gradient Methods]] – Actor learns using policy gradients  
- [[GAE]] – Computes smooth advantage estimates  
- [[RL Agent]] – Structured as actor and critic components  

---

## 🔗 Related Concepts

- [[DDPG]] (Off-policy actor-critic for continuous actions)  
- [[PPO]] (Stable actor-critic using clipped policy objectives)  
- [[A2C]] / [[A3C]] (Synchronous/asynchronous actor-critic)  
- [[TD Learning]] (Used by critic)  
- [[Entropy Regularization]] (Helps exploration in actor-critic)  

---

## 📚 Further Reading

- [Sutton & Barto – Actor-Critic Methods](http://incompleteideas.net/book/the-book.html)  
- [Spinning Up by OpenAI – Actor Critic](https://spinningup.openai.com/en/latest/algorithms/a2c.html)  
- [DDPG Paper](https://arxiv.org/abs/1509.02971)  
- [PPO Paper](https://arxiv.org/abs/1707.06347)  
- [Stable Baselines3 – Actor Critic Examples](https://stable-baselines3.readthedocs.io/)  

---
