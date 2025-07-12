# RL Step

In **Reinforcement Learning (RL)**, the `step()` function is the core interaction point between an agent and its environment. It executes one timestep of the simulation or real-world task, processing the agent’s chosen action and returning the consequences.

The `step()` function is typically called in a training loop and is fundamental to any RL framework. It is responsible for transitioning the environment’s state, returning the new observation, reward, done signal, and additional info.

---

## 🧠 Core Concepts

- `Action Input`: The decision made by the agent for the current timestep  
- `State Transition`: The environment updates its internal state based on the action  
- `Observation`: What the agent sees after the action is applied  
- `Reward`: Feedback from the environment indicating how good the action was  
- `Done`: Boolean flag indicating if the episode has ended  
- `Info`: Optional dictionary with debugging or auxiliary data  

Typical signature:
`observation, reward, done, info = env.step(action)`

---

## 📚 Example Use

- `action = policy(observation)`  
- `obs, reward, done, info = env.step(action)`  
- `if done: env.reset()`  

This pattern is consistent across most environments, including `Gymnasium`, `Isaac Gym`, `PyBullet`, and `PettingZoo`.

---

## 🧰 Use Cases

- Iterative interaction with the environment  
- Collecting data for training or replay buffers  
- Logging rewards and episode statistics  
- Driving animations or hardware actuators  
- Multi-agent coordination by sequential stepping  

---

## ✅ Pros

- Standard interface across libraries  
- Enables episodic learning and trajectory generation  
- Flexible enough to wrap simulations, games, or real robots  
- Works with both discrete and continuous actions  

---

## ❌ Cons

- Assumes synchronous, step-by-step control (less real-time)  
- Misuse (e.g. forgetting to reset) leads to broken episodes  
- Inconsistent `info` usage across environments  
- Real-time constraints in robotics require careful timing  

---

## 📊 Comparison Table: `step()` in Different Frameworks

| Framework     | API Call                  | Notes                              |
|---------------|---------------------------|-------------------------------------|
| Gymnasium     | `obs, reward, done, info` | Most common single-agent interface |
| PettingZoo    | `env.step(action_dict)`   | Multi-agent: pass dict of actions  |
| Isaac Gym     | GPU-accelerated stepping  | Batched envs, requires tensor ops  |
| PyBullet      | `env.step(action)`        | Includes physics simulation        |
| PufferLib     | Wraps `step()` internally | Compatible with distributed agents |

---

## 🤖 In Robotics Context

| Task                      | `step()` Role                                               |
|---------------------------|-------------------------------------------------------------|
| Arm control               | Moves joints based on torques or positions                  |
| Navigation                | Moves robot base and updates lidar/odometry readings        |
| Simulation environments   | Handles dynamics and contact updates per action             |
| Real-world deployment     | May wrap actuation/sensor code within step abstraction      |
| Multi-agent systems       | Each agent's step affects shared state and observation      |

---

## 🔧 Compatible Items

- [[RL Environment]] – `step()` is a method defined on environments  
- [[RL Actions]] – Input to the `step()` function  
- [[RL Observations]] – Output from `step()`  
- [[RL Reward]] – Part of the returned tuple  
- [[PufferLib]] – Abstracts `step()` calls for multi-agent and batch use  
- [[Gymnasium]] – Origin of the standard `step()` API  

---

## 🔗 Related Concepts

- [[Reinforcement Learning]] (Parent topic)  
- [[Episode]] (Sequence of steps ending with `done = True`)  
- [[Trajectory]] (Full list of step interactions for training)  
- [[Reset Function]] (Used to restart after an episode ends)  
- [[Replay Buffer]] (Stores step results for training)  

---

## 📚 Further Reading

- [Gymnasium API Overview](https://gymnasium.farama.org/api/env/#gymnasium.Env.step)  
- [PettingZoo Parallel Envs](https://pettingzoo.farama.org/api/parallel/)  
- [Isaac Gym Step Behavior](https://developer.nvidia.com/isaac-gym)  
- [Multi-agent RL Loop (MADDPG)](https://arxiv.org/abs/1706.02275)  

---
