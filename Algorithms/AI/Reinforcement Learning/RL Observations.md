# RL Observations

In **Reinforcement Learning (RL)**, *observations* represent the information an agent receives from the environment at each timestep. These observations are the basis for decision-making and are passed to the policy to compute actions. The structure and richness of the observation space directly influence the complexity of learning and the success of trained policies.

Observations differ from the full *state* of the environment, especially in partially observable settings. Effective observation design is critical for performance and generalization.

---

## 📚 Overview

An RL observation is a vector or structured data that describes the environment’s current status from the agent’s perspective. Observations can range from raw sensor inputs (e.g., RGB images, Lidar) to structured representations (e.g., joint angles, object positions). In multi-agent RL, each agent may have a unique observation.

Environments define their own `observation_space`, which should be compatible with the RL algorithm and network architecture.

---

## 🧠 Core Concepts

- `Observation Space`: The range and structure of values the environment returns  
- `Partial Observability`: Observations don't reveal the full environment state  
- `History Dependence`: Sequences of observations may be needed for good decisions  
- `Feature Engineering`: Optionally processes or augments raw inputs  
- `Sensors and Perception`: Real-world observations come from noisy, delayed sensors  
- `Normalization`: Often used to standardize observation values to aid training  

---

## 🧰 Use Cases

- Observing robot pose, velocities, joint angles, and contact forces  
- Feeding RGB, depth, or point cloud data into vision-based RL agents  
- Extracting relevant object positions for manipulation tasks  
- Using Lidar or occupancy grids in navigation and SLAM tasks  
- Creating ego-centric views for autonomous vehicles or drones  

---

## ✅ Pros

- Highly flexible input formats: structured data, images, sequences  
- Allows agent-specific design in multi-agent environments  
- Enables abstraction of complex real-world sensor input  
- Can include proprioception, exteroception, or external context  

---

## ❌ Cons

- High-dimensional observations increase sample complexity  
- Incomplete or noisy observations may require memory or filtering  
- Raw sensor data (e.g., images) require more compute and training data  
- Partial observability limits policy performance in some cases  

---

## 📊 Comparison Table: Observation Modalities

| Type              | Dimensionality | Example Input                  | Common Uses                       |
|-------------------|----------------|--------------------------------|-----------------------------------|
| Proprioceptive    | Low            | Joint angles, velocities       | Manipulators, locomotion          |
| Vision            | High           | RGB or depth images            | Grasping, navigation, drones      |
| LiDAR/Range       | Medium         | Distance vectors, point clouds | SLAM, obstacle avoidance          |
| Positional        | Medium         | Object/world coordinates       | Pick-and-place, pursuit-evasion   |
| Task Parameters   | Low            | Goal pose, object class        | Goal-conditioned policies         |
| History Buffer    | High           | Past N observations            | RNN-based policies, POMDPs        |

---

## 🤖 In Robotics Context

| System/Environment      | Typical Observations Used                           |
|-------------------------|-----------------------------------------------------|
| Mobile robot navigation | Lidar scans, velocity, heading                      |
| Robot arm manipulation  | Joint angles, object pose, force/torque sensors     |
| Drone control           | IMU data, altitude, velocity, camera feed           |
| Multi-agent systems     | Agent position, neighbor locations, communication   |
| Simulated environments  | Engineered state vectors (object positions, etc.)   |

---

## 🔧 Compatible Items

- [[Reinforcement Learning]] – Observations are inputs to the agent  
- [[Gymnasium]] – Provides `observation_space` and `reset()`/`step()` methods  
- [[PufferLib]] – Manages observation normalization and encoding  
- [[PettingZoo]] – Defines per-agent observations in multi-agent settings  
- [[Isaac Gym]] – Exposes tensors representing environment observations  
- [[Sim-to-Real Transfer]] – Observations need to generalize across domains  

---

## 🔗 Related Concepts

- [[RL Actions]] (Observations are used to choose actions)  
- [[Observation Space Normalization]] (Standardizing values improves training)  
- [[Partial Observability]] (Observation ≠ full environment state)  
- [[RL Policy]] (Consumes observations to output actions)  
- [[Replay Buffer]] (Stores past observations for training)  

---

## 📚 Further Reading

- [Gymnasium Observation Spaces](https://gymnasium.farama.org/api/spaces/)  
- [POMDPs Explained](https://web.stanford.edu/class/psych209/Readings/SuttonBartoIPRLBook2ndEd.pdf)  
- [RoboSuite Observation Configs](https://github.com/ARISE-Initiative/robosuite)  
- [OpenAI Procgen Observations](https://openai.com/research/procgen-benchmark)  
- [Sim-to-Real Transfer Survey](https://arxiv.org/abs/2009.13303)  

---
