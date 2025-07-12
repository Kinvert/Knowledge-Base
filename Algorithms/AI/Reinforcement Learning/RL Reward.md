# RL Reward

In **Reinforcement Learning (RL)**, the *reward* is the scalar feedback signal given to the agent after taking an action in a particular state. It defines the learning objective and drives the agent to develop optimal behavior by maximizing cumulative reward over time. Carefully designing the reward function is crucial to guide learning and avoid unintended behavior.

Reward signals can be dense (frequent, detailed) or sparse (infrequent, only for major accomplishments), and are central to the formulation of the Markov Decision Process (MDP).

---

## 📚 Overview

At each timestep, the RL environment returns a `reward` value along with the next state and done flag. The agent uses this signal to update its policy. The cumulative reward (often discounted using a factor `gamma`) defines the agent’s success in completing a task.

Designing good reward functions requires balancing task goals, learning speed, and behavioral safety.

---

## 🧠 Core Concepts

- `Reward Function`: A mapping from state and action (or transitions) to a scalar feedback signal  
- `Cumulative Reward`: The total reward an agent accumulates over an episode or long-term horizon  
- `Discount Factor (γ)`: Reduces the weight of future rewards, balancing short vs long-term gains  
- `Sparse Reward`: Rewards only at key milestones (e.g. success/failure)  
- `Shaped Reward`: Dense feedback to guide the agent toward good behavior  
- `Reward Hacking`: When an agent finds unintended ways to maximize reward  

---

## 🧰 Use Cases

- Scoring points in a game or competition  
- Minimizing time or energy consumption in robotics  
- Encouraging successful grasping or placement  
- Penalizing collisions or unsafe behavior  
- Optimizing trajectory smoothness or efficiency  
- Encouraging exploration or coverage  

---

## ✅ Pros

- Simple scalar feedback is flexible and lightweight  
- Can encode complex goals and tradeoffs via shaping  
- Can be handcrafted, learned, or derived from demonstrations  
- Central to reward-based training approaches like policy gradients  

---

## ❌ Cons

- Poor reward design leads to suboptimal or unsafe policies  
- Sparse rewards make learning slow and unstable  
- Shaped rewards can introduce bias or shortcuts  
- Rewards need tuning and domain-specific understanding  

---

## 📊 Comparison Table: Reward Design Strategies

| Reward Type      | Description                            | Pros                         | Cons                           |
|------------------|----------------------------------------|------------------------------|--------------------------------|
| Sparse           | Only for success/failure               | Simple, clear objective      | Hard to learn from             |
| Dense/Shaped     | Frequent feedback                      | Faster learning              | May introduce bias             |
| Learned Reward   | Trained via IRL or preference learning | Human-like, flexible         | Complex, requires data         |
| Multi-Objective  | Vector of goals                        | Encodes tradeoffs            | Needs weighting or priorities  |
| Binary Reward    | 0 or 1 based on outcome                | Simple                       | Ignores nuances of performance |

---

## 🤖 In Robotics Context

| Task                      | Example Reward Design                            |
|---------------------------|--------------------------------------------------|
| Object manipulation       | +1 for successful grasp, small penalties for misses  
| Mobile navigation         | -1 for collisions, +0.1 per meter toward goal  
| Drone control             | Reward altitude stability, penalize drift  
| Multi-agent cooperation   | Shared reward for completing group task  
| Energy efficiency         | Reward based on minimal actuation effort  

---

## 🔧 Compatible Items

- [[Reinforcement Learning]] – Reward is the learning signal  
- [[Policy Gradient Methods]] – Directly optimize reward expectation  
- [[RL Observations]] – Observations influence which actions receive reward  
- [[PufferLib]] – Handles reward shaping and batching  
- [[Simulation Environments]] – Provide customizable reward functions  
- [[Gymnasium]] – Uses `.step()` to return reward at each step  

---

## 🔗 Related Concepts

- [[Reinforcement Learning]] (Parent concept)  
- [[Reward Shaping]] (Refines reward signals to improve learning)  
- [[Imitation Learning]] (Sometimes avoids needing an explicit reward function)  
- [[Policy]] (Learns to maximize expected reward)  
- [[RL Actions]] (Determine how rewards are earned)  

---

## 📚 Further Reading

- [OpenAI Reward Design Examples](https://spinningup.openai.com/en/latest/spinningup/rl_intro.html#reward-hacking)  
- [Ng et al., 1999 – Reward Shaping Paper](https://papers.nips.cc/paper_files/paper/1999/file/9e3cfc48c2c8613c2ed8a7497e33bdf7-Paper.pdf)  
- [Reward Engineering in Robotics](https://arxiv.org/abs/1806.07365)  
- [Inverse Reinforcement Learning](https://arxiv.org/abs/1811.08782)  
- [DeepMind Reward Hacking Examples](https://deepmind.com/blog/article/specification-gaming-the-flip-side-of-ai-ingenuity)  

---
