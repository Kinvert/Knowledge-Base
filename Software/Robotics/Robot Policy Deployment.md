# 🚀 Robot Policy Deployment

**Robot Policy Deployment** is the process of taking a trained policy from an RL or imitation-learning stack and running it inside a real robot control system. It connects [[Reinforcement Learning]], [[Sim2Real]], [[ROS2]], [[ONNX]], low-level control, and runtime safety.

---

## 📚 Overview

A policy that works in simulation is not deployed until it runs at the correct frequency, consumes real observations, respects action limits, handles stale data, and fails safely. Deployment is where model export, middleware, latency, state estimation, and actuator constraints meet.

---

## 🧠 Core Concepts

- **Policy Export**: Saving a trained network as PyTorch, TorchScript, [[ONNX]], or TensorRT.
- **Observation Pipeline**: Converts real robot state into the exact tensor layout used during training.
- **Action Pipeline**: Converts policy output into joint targets, torques, velocities, or task-space commands.
- **Control Frequency**: Rate at which policy actions are produced.
- **Watchdog**: Stops or falls back if observations, inference, or commands become stale.
- **Fallback Controller**: Safer controller used when the learned policy is invalid.
- **Telemetry Logging**: Records observations, actions, latency, interventions, and safety events.

---

## 📊 Comparison Chart

| Deployment Format | Best For | Strength | Weakness | Robotics Fit |
|---|---|---|---|---|
| PyTorch eager | Lab debugging | Easy to inspect | Slower, Python runtime | Medium |
| TorchScript | PyTorch production | Simple export path | Some model limits | High |
| [[ONNX]] | Cross-runtime export | Portable | Conversion quirks | High |
| TensorRT | NVIDIA inference | Low latency | NVIDIA-specific | Very high on Jetson |
| C++ inference | Embedded control | Tight integration | More engineering | High |
| Microcontroller policy | Tiny robots | Low power | Small models only | Niche |

---

## ✅ Pros

- Makes trained policies usable outside notebooks and simulators.
- Forces observation and action contracts to be explicit.
- Enables latency and safety testing.
- Works with [[ROS2]], [[ros2_control]], and edge computers like [[Jetson Family]].
- Creates logs for Sim2Real debugging.

---

## ❌ Cons

- Exported models can differ numerically from training.
- Real observations may not match simulator distributions.
- Latency and jitter can destabilize policies.
- Action scaling mistakes can damage hardware.
- Requires safety checks outside the policy.

---

## 🧰 Deployment Checklist

1. Freeze the exact observation order and normalization constants.
2. Export the model and compare outputs against the training runtime.
3. Add action clipping, action-rate limits, and safety bounds.
4. Run policy inference against recorded logs before live hardware.
5. Add watchdogs for stale observations and slow inference.
6. Start with reduced speed, torque, and workspace limits.
7. Log observations, actions, inference time, and interventions.

---

## 🔗 Related Notes

- [[Sim2Real]]
- [[Safe RL for Robotics]]
- [[ROS2]]
- [[ros2_control]]
- [[ONNX]]
- [[TensorRT]]
- [[Actuator Modeling for Sim2Real]]

---

## 🌐 External Resources

- ROS 2 Docs: https://docs.ros.org/
- ONNX Runtime: https://onnxruntime.ai/
- NVIDIA TensorRT: https://developer.nvidia.com/tensorrt

---

## 📝 Summary

Robot policy deployment is where RL becomes robotics software. The important work is making the observation/action contract exact, bounding the policy's authority, and ensuring the robot has safe behavior when inference or state estimation fails.
