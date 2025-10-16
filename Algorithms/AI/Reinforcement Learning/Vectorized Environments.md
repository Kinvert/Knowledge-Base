# Vectorized Environments

**Vectorized Environments** are reinforcement learning environments that run multiple independent environment instances in parallel, allowing agents to collect experience more efficiently. They are a key technique for speeding up training, particularly in environments with fast simulations and minimal rendering.

---

## 🔍 Overview

- Enable parallel interaction with multiple copies of the same or similar environments.  
- Often used in deep RL to increase throughput and reduce wall-clock training time.  
- Each environment runs independently, allowing batch data collection.  
- Commonly supported in libraries like [[Gym]], [[PufferLib]], [[EnvPool]], and [[Stable-Baselines3]].

---

## 🧠 Core Concepts

- **Environment Batch**: A group of environments treated as a single object that returns batched observations, rewards, etc.  
- **Synchronization**: Envs can be run synchronously (wait for all to finish) or asynchronously (act as soon as one is ready).  
- **Data Efficiency**: Improves sample collection rate, especially important for on-policy algorithms like [[PPO]].  
- **Exploration Diversity**: Each env can explore different trajectories, reducing correlation in experience data.

---

## 🧰 Use Cases

- Scaling RL training across CPUs or threads.  
- On-policy methods requiring large batches of fresh data.  
- Distributed or multi-process RL agents.  
- Evaluating policy generalization across multiple seeds/scenarios.

---

## ✅ Pros

- Greatly reduces training time by parallelizing experience collection.  
- Improves learning stability with diverse trajectories.  
- Seamlessly integrates with batch-based neural network training.  
- Can be scaled across CPU cores or GPU threads (e.g. in [[Isaac Gym]]).

---

## ❌ Cons

- Higher memory usage due to multiple environment instances.  
- Debugging can be more complex compared to single-env setups.  
- Some environments (especially with heavy rendering) may not benefit much.  
- Async vectorization introduces potential for desynchronization bugs.

---

## 📊 Comparison Table: Vectorized vs Single-Environment Setup

| Feature               | Vectorized Environments          | Single Environment             |
|-----------------------|----------------------------------|-------------------------------|
| Sample Collection     | Parallel                         | Sequential                    |
| Training Speed        | Faster                           | Slower                        |
| Experience Diversity  | High (multiple envs)             | Low (single trajectory)       |
| Debuggability         | Lower                            | Easier                        |
| Use Cases             | Scalable training, PPO, SAC      | Debugging, small models       |

---

## 🔧 Compatible Items

- [[PufferLib]] – Provides high-performance vectorized envs  
- [[EnvPool]] – Extreme-speed vectorized simulation  
- [[Stable-Baselines3]] – Uses vector envs in training loops  
- [[Replay Buffer]] – Works well with parallel data streams  
- [[On-Policy]] – Algorithms that benefit greatly from fresh, parallel data

---

## 🔗 Related Concepts

- [[Simulation Environments]] – Source of state-action-reward data  
- [[RL Environment]] – General abstraction of a task for an agent  
- [[Batch Processing]] – Neural networks trained on collected batches  
- [[Parallel Computing]] – Technical foundation for env vectorization  
- [[Off-Policy]] – Can also benefit from multiple simultaneous rollouts

---

## 📚 Further Reading

- [OpenAI Baselines - VecEnv API](https://github.com/openai/baselines/blob/master/baselines/common/vec_env/vec_env.py)  
- [SB3 Docs - Vectorized Environments](https://stable-baselines3.readthedocs.io/en/master/guide/vec_envs.html)  
- [EnvPool GitHub](https://github.com/sail-sg/envpool)  
- [Parallelization in RL - Spinning Up](https://spinningup.openai.com/en/latest/techniques/parallelization.html)  

---
