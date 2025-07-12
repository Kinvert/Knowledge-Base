# RL State

In **Reinforcement Learning (RL)**, the *state* refers to a complete representation of the environment at a given time. It encodes all necessary information that influences the outcome of future actions and rewards. The state is central to the Markov Decision Process (MDP) framework, which assumes that the current state contains all information needed to determine the next outcome.

Unlike *observations*, which are what the agent directly perceives, the *true state* may be partially or fully hidden in real-world or simulated environments.

---

## 📚 Overview

The RL state is the abstract or physical configuration that determines the dynamics of the environment. In fully observable settings, agents have access to this full state, but in many practical cases (e.g., robotics, games, real-world control systems), agents must act based only on partial observations or inferred estimates.

State is a critical concept in defining value functions, policy mappings, and transition dynamics.

---

## 🧠 Core Concepts

- `State (s)`: A full description of the environment at a specific timestep  
- `Observation (o)`: What the agent sees, which may or may not equal the full state  
- `Markov Property`: Future states depend only on the current state and action  
- `State Space`: The set of all possible states  
- `Transition Function`: `P(s' | s, a)` defines how states evolve given actions  
- `Initial State Distribution`: Describes how each episode starts  

---

## 🧰 Use Cases

- Designing RL environments that expose full or partial state  
- Creating state estimators in partially observable systems  
- Defining reward functions that depend on state variables  
- Sim-to-real tasks where state differs from noisy observation  
- Using internal state in memory-based policies (e.g., RNNs or filters)  

---

## ✅ Pros

- Provides the most accurate, complete information for learning  
- Essential for defining theoretical guarantees (e.g., Bellman equations)  
- Useful in debugging and environment introspection  
- Allows deterministic replay and evaluation  

---

## ❌ Cons

- Not always available in real-world scenarios  
- Can be high-dimensional and difficult to represent compactly  
- Assumes perfect knowledge, which limits realism  
- May require sensor fusion or estimation to approximate in practice  

---

## 📊 Comparison Table: State vs Observation

| Property             | State                        | Observation                   |
|----------------------|------------------------------|-------------------------------|
| Completeness         | Fully describes environment   | May be partial/noisy          |
| Availability         | Depends on env implementation| Always available to agent      |
| Use in theory        | Core to MDP, value functions  | Used in POMDPs or approximations |
| Robotics example     | True joint positions + object poses | Lidar scan + camera image      |
| Learning difficulty  | Easier if known               | Requires inference/estimation |

---

## 🤖 In Robotics Context

| System              | True State Example                              |
|---------------------|--------------------------------------------------|
| Mobile robot        | Full pose (x, y, θ) + map of environment         |
| Manipulator arm     | Joint angles, velocities, object positions       |
| Autonomous car      | Precise location, velocity, and all surroundings |
| Drone flight        | Position, orientation, velocity, wind conditions |
| Multi-agent swarm   | All agent positions and internal states          |

---

## 🔧 Compatible Items

- [[Reinforcement Learning]] – State is central to MDP formulation  
- [[RL Observations]] – Often a subset or transformation of the full state  
- [[RL Reward]] – Can be based on state features  
- [[RL Step]] – Transitions the state given an action  
- [[Sim-to-Real Transfer]] – State is typically used during simulation training  
- [[Trajectory]] – Sequence of states, actions, and rewards  

---

## 🔗 Related Concepts

- [[Observation Space]] (Subset of state exposed to agent)  
- [[Markov Decision Process]] (Formalism where decisions depend on state)  
- [[POMDP]] (When states are partially observable)  
- [[RL Policy]] (Maps from state or observation to action)  
- [[RL Agent]] (Acts based on perceived state or observation)  

---

## 📚 Further Reading

- [Sutton & Barto – RL Book](http://incompleteideas.net/book/the-book.html)  
- [OpenAI Gym vs True State](https://gymnasium.farama.org/)  
- [Partially Observable MDPs (POMDPs)](https://web.stanford.edu/class/psych209/Readings/SuttonBartoIPRLBook2ndEd.pdf)  
- [POMDPs.jl for Julia](https://github.com/JuliaPOMDP/POMDPs.jl)  
- [DeepMind Planning with Latent States](https://arxiv.org/abs/1806.01822)  

---
