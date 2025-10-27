# Annealing (in AI / Reinforcement Learning)

Annealing in the context of Artificial Intelligence (AI) and Reinforcement Learning (RL) refers to the gradual adjustment of parameters—such as learning rate or exploration rate—over time, inspired by the physical process of *annealing* in metallurgy. The goal is to allow an algorithm to explore broadly at first and then progressively refine its search toward optimal or stable solutions as it "cools."

---

## 🧠 Overview

Annealing is a meta-optimization technique that helps balance *exploration* and *exploitation*. It’s commonly used in reinforcement learning for:
- Decreasing the learning rate over time (learning rate annealing)
- Reducing randomness in policy exploration (epsilon annealing in [[Epsilon-Greedy]])
- Controlling temperature in softmax-based policies (temperature annealing)

The underlying principle comes from *Simulated Annealing*, a stochastic optimization inspired by the physical cooling process where high energy states are slowly reduced to reach a low-energy equilibrium.

---

## ⚙️ Core Concepts

- **Temperature (T):** A parameter controlling randomness. High T encourages exploration, low T favors exploitation.
- **Cooling Schedule:** Defines how T (or analogous variables) decreases over time. Can be linear, exponential, or adaptive.
- **Exploration vs. Exploitation:** Annealing allows more random actions early in training and more deterministic ones later.
- **Learning Rate Annealing:** Gradual reduction in learning rate to stabilize convergence.
- **Entropy Regularization:** Sometimes used with annealing to maintain sufficient exploration during policy optimization.

Example schedules:
- Linear decay: `T = T0 * (1 - t / Tmax)`
- Exponential decay: `T = T0 * exp(-k * t)`

---

## 🧩 Comparison Chart

| Technique | Primary Use | Key Parameter | Exploration Behavior | Inspired By | Typical Application |
|------------|--------------|----------------|----------------------|--------------|---------------------|
| **Annealing** | Gradual reduction of randomness or rate | Temperature / ε / α | Decreases over time | Thermodynamics | [[Reinforcement Learning]] |
| **Simulated Annealing** | Global optimization | Temperature | Decreases with schedule | Physical annealing | [[Optimization Algorithms]] |
| **Epsilon-Greedy** | Policy exploration | ε (epsilon) | Randomness decreases | Heuristic | [[Q-Learning]] |
| **Boltzmann Exploration** | Probabilistic policy | Temperature | Softmax-based randomness | Statistical mechanics | [[Policy Gradient Methods]] |
| **Adaptive LR** | Gradient-based learning | α (learning rate) | Adjusts via gradients | Optimization | [[Deep Reinforcement Learning]] |

---

## 🧰 Use Cases

- **Epsilon decay** in [[DQN]] (Deep Q Network) training  
- **Learning rate annealing** in policy gradient or actor-critic methods  
- **Temperature decay** in [[Soft Actor-Critic]] or softmax exploration  
- **Reward shaping** in dynamic environments where exploration needs to shift over time  
- **Simulated annealing** for solving combinatorial optimization tasks such as TSP or robot path planning

---

## ✅ Strengths

- Encourages early exploration and eventual convergence  
- Reduces risk of getting stuck in local minima  
- Easy to implement across many algorithms  
- Compatible with both value-based and policy-based RL methods  
- Proven to improve convergence stability and training efficiency  

---

## ❌ Weaknesses

- Requires careful tuning of decay rates and schedules  
- Overly aggressive cooling can cause premature convergence  
- Too slow cooling wastes computational resources  
- May behave unpredictably in non-stationary environments  

---

## 🔧 Variants

- **Exponential Annealing:** Fast initial change, slower final cooling  
- **Linear Annealing:** Simple and predictable decay  
- **Cosine Annealing:** Smooth, periodic decay often used in deep learning optimizers  
- **Adaptive Annealing:** Adjusts based on training feedback (e.g., performance plateaus)  

---

## 📚 Related Concepts / Notes

- [[Reinforcement Learning]]  
- [[Simulated Annealing]]  
- [[Epsilon-Greedy]]  
- [[DQN]] (Deep Q Network)  
- [[Soft Actor-Critic]]  
- [[Exploration vs Exploitation]]  
- [[Policy Gradient Methods]]  
- [[Learning Rate Scheduling]]  

---

## 🌐 External Resources

- Sutton & Barto, *Reinforcement Learning: An Introduction* (2nd Edition)  
- Silver et al., *Deterministic Policy Gradient Algorithms*, ICML 2014  
- DeepMind’s DQN paper: *Human-level control through deep reinforcement learning* (Nature, 2015)  
- OpenAI Spinning Up: *Exploration Strategies* section  
- François Chollet, *Deep Learning with Python* (Keras temperature annealing examples)

---
