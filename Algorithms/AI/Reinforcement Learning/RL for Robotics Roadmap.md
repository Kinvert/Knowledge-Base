# 🤖 RL for Robotics Roadmap

**RL for Robotics** is the path from reinforcement learning concepts to policies that can control simulated and physical robots. In this vault, the useful thread is [[PufferLib]] for fast RL iteration, [[C99]] for environment authoring, [[MuJoCo]] and [[Isaac Lab]] for robot simulation, [[Sim2Real]] for transfer, and [[ROS2]] for deployment.

---

## 🧭 Overview

This roadmap connects the existing notes into a practical learning route. The purpose is not to replace textbooks or papers. It is to make the Obsidian graph show how RL algorithms, robot simulators, control theory, system identification, datasets, and deployment tools relate to each other.

The core path:

1. Learn the vocabulary with [[MDP]], [[RL Environment]], [[RL Agent]], [[RL Policy]], [[Reward Function]], and [[RL Trajectory]].
2. Train stable baselines with [[PPO]] and [[SAC]].
3. Build fast custom environments with [[PufferLib C99 Environment Authoring]].
4. Move robot tasks into [[MuJoCo MJCF]] or [[Isaac Lab Task Authoring]].
5. Close the [[Sim2Real]] gap with [[Actuator Modeling for Sim2Real]] and [[System Identification for Sim2Real]].
6. Deploy through [[ROS2]], [[ros2_control]], and [[Robot Policy Deployment]].

---

## 🧠 Learning Stages

| Stage | Main Notes | What To Build |
|---|---|---|
| RL basics | [[Reinforcement Learning]], [[MDP]], [[Bellman Equation]], [[Policy Gradient]] | A tiny tabular or Gymnasium task |
| Deep RL | [[PPO]], [[SAC]], [[Actor Critic]], [[GAE]], [[Replay Buffer]] | A continuous-control baseline |
| Fast environments | [[PufferLib]], [[PufferLib C99 Environment Authoring]], [[Vectorized Environments]] | A C99 toy robot env |
| Robot simulation | [[MuJoCo]], [[MuJoCo MJCF]], [[Isaac Lab]], [[Isaac Lab Task Authoring]] | A reacher, walker, or manipulation task |
| Control bridge | [[PID Controller]], [[Inverse Kinematics]], [[RL and Classical Control]] | RL policy wrapped by a low-level controller |
| Sim2Real | [[Sim2Real]], [[Domain Randomization]], [[System Identification for Sim2Real]] | A sim log compared against real robot data |
| Deployment | [[ROS2]], [[ros2_control]], [[ONNX]], [[Robot Policy Deployment]] | A policy node with safety limits |
| Robot data | [[Imitation Learning]], [[LeRobot]], [[robomimic]], [[Diffusion Policy]] | Teleop demos and behavior cloning |

---

## 📊 Roadmap Comparison Chart

| Track | Best First Tool | Main Skill | Strength | Weakness |
|---|---|---|---|---|
| Fast RL systems | [[PufferLib]] | Environment throughput | Teaches rollout performance | Not a full robot simulator |
| Robot physics | [[MuJoCo]] | Rigid-body modeling | Fast, clean dynamics | Less asset-rich than Isaac |
| GPU robot learning | [[Isaac Lab]] | Scalable robotics RL | Strong Sim2Real workflow | Heavy setup |
| Classical robotics | [[ROS2]] + [[MoveIt]] | Integration and planning | Real robot ecosystem | Not an RL trainer |
| Imitation learning | [[LeRobot]] | Dataset and policy training | Direct real robot path | Dataset quality dominates |
| Offline benchmarks | [[robomimic]] | Reproducible IL/offline evaluation | Good manipulation baselines | Mostly benchmark-oriented |

---

## 🧰 Practical Projects

- Write a minimal [[Gymnasium Environment Authoring for Robotics]] task, then port the fast part to [[PufferLib C99 Environment Authoring]].
- Build the same simple robot task in [[MuJoCo MJCF]] and [[Isaac Lab Task Authoring]] to compare simulator workflow.
- Add actuator delay, observation noise, friction randomization, and action limits to test [[Sim2Real]] assumptions.
- Collect a small teleoperation dataset with [[LeRobot]] and train [[ACT Action Chunking Transformer]] or [[Diffusion Policy]].
- Export a small policy and run it as a [[ROS2]] node with a watchdog and fallback controller.

---

## ✅ Good Learning Signals

- You can explain why [[PPO]] is common in locomotion and why [[SAC]] is common in sample-efficient continuous control.
- You can design observations and actions without leaking privileged simulation state.
- You can tell when a reward is shaping behavior versus hiding a modeling bug.
- You can identify whether a failure is policy learning, actuator modeling, state estimation, or deployment latency.
- You can choose between [[PufferLib]], [[MuJoCo]], [[Isaac Lab]], [[RSL-RL]], [[RL-GAMES]], and [[LeRobot]] for a specific task.

---

## ⚠️ Common Traps

- Jumping into [[Isaac Lab]] before understanding the Gymnasium-style RL interface.
- Treating [[Domain Randomization]] as a substitute for basic system identification.
- Training with privileged simulator observations that cannot exist on hardware.
- Tuning reward weights before checking action scale, control frequency, and reset conditions.
- Assuming a policy export is enough for deployment without safety limits.

---

## 🔗 Related Notes

- [[PufferLib]]
- [[PufferLib Robotics Fit and Limits]]
- [[Sim2Real]]
- [[Isaac Lab]]
- [[MuJoCo]]
- [[ROS2]]
- [[C99]]
- [[CUDA]]

---

## 📝 Summary

The fastest path into RL for robotics is to learn RL environment mechanics first, robot simulation second, and hardware deployment third. [[PufferLib]] and [[C99]] are excellent for understanding fast environment loops. [[MuJoCo]] and [[Isaac Lab]] are the bridge to real robot physics. [[LeRobot]] and [[robomimic]] cover the modern data-driven manipulation path.
