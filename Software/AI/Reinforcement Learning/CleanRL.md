# CleanRL

**CleanRL** is a collection of high-quality, minimalistic reinforcement learning implementations written in **pure PyTorch**. The goal of CleanRL is to provide **clear and single-file** implementations of RL algorithms that are easy to read, understand, and modify — making it ideal for education, debugging, and research prototyping.

---

## 🔍 Overview

- Written in Python using PyTorch, with minimal abstractions and no heavy framework layers.  
- Each algorithm is implemented in a single `.py` file with less than 300 lines.  
- Includes logging via `wandb`, support for vectorized environments, seeding, and evaluation.  
- Prioritizes readability and reproducibility over performance or extensibility.

---

## 🧠 Core Concepts

- **Reproducibility**: Consistent results with fixed seeds, deterministic behavior where possible.  
- **Minimalism**: Every algorithm is implemented with minimal boilerplate, making it readable line-by-line.  
- **No Framework Lock-In**: No reliance on custom classes or abstract layers — just PyTorch and Gym.  
- **Algorithm-by-File Design**: Each script is self-contained and independently runnable.

---

## 🧰 Use Cases

- Learning how RL algorithms work under the hood.  
- Debugging performance or stability issues in custom implementations.  
- Educational demonstrations and tutorials.  
- Quick prototyping for research by modifying reference implementations.

---

## ✅ Pros

- Extremely easy to read and modify.  
- Great for education and understanding core RL algorithms.  
- Reproducible training and evaluation out-of-the-box.  
- Clean integration with logging tools like `wandb`.

---

## ❌ Cons

- Not built for large-scale production use.  
- No support for distributed training or complex multi-agent settings.  
- Limited hyperparameter tuning tools compared to libraries like [[Ray RLlib]] or [[TorchRL]].  
- Lacks modular abstractions for components like replay buffers or loss modules.

---

## 📊 Comparison Table: CleanRL vs Other RL Libraries

| Feature               | CleanRL            | [[Stable-Baselines3]]  | [[TorchRL]]            | [[RLlib]]                |
|------------------------|--------------------|---------------------|--------------------|----------------------|
| Code Style             | Minimal, flat      | Object-oriented     | Modular, typed     | Scalable framework   |
| Best For               | Learning, hacking  | Prototyping         | Research           | Production           |
| Lines per Algorithm    | <300               | ~1,000+             | ~500+              | 1,000s (with config) |
| Logging                | ✅ wandb           | ✅ Tensorboard       | ✅ Tensorboard      | ✅ All major tools    |
| Extensibility          | ❌ Low             | 🟡 Moderate         | ✅ High             | ✅ High              |

---

## 🔧 Compatible Items

- [[PyTorch]] – Foundation of all CleanRL code  
- [[Gym]] – Standard environment interface used  
- [[wandb]] – Optional logging integration  
- [[RL Algorithms]] – Like [[PPO]], [[DQN]], [[A2C]], [[DDPG]] all implemented here  
- [[Replay Buffer]] – Used directly in off-policy algorithms like [[SAC]] and [[DQN]]  
- [[Vectorized Environments]] – Supported via Gym's `AsyncVectorEnv` or `SubprocVectorEnv`

---

## 🔗 Related Concepts

- [[Reinforcement Learning]] – CleanRL serves as a reference implementation library  
- [[PPO]], [[A2C]], [[DQN]], [[DDPG]], [[SAC]] – All implemented in CleanRL  
- [[TorchRL]] – More advanced, modular, and production-capable alternative  
- [[RL Training Pipelines]] – CleanRL provides the most minimal pipeline possible  
- [[CARBS]] – Can be used on top of CleanRL scripts for HPO

---

## 📚 Further Reading

- [CleanRL GitHub Repository](https://github.com/vwxyzjn/cleanrl)  
- [CleanRL Documentation](https://docs.cleanrl.dev)  
- [CleanRL Blog and Tutorials](https://docs.cleanrl.dev/blog)  
- [OpenRL Benchmarking Leaderboard](https://benchmark.cleanrl.dev)  

---
