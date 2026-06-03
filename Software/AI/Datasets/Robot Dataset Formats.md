# 🗂️ Robot Dataset Formats

**Robot Dataset Formats** describe how robot observations, actions, videos, rewards, episode metadata, and calibration data are stored for imitation learning, offline RL, and policy evaluation.

---

## 📚 Overview

Robot learning datasets are more complicated than ordinary image datasets because actions and timestamps matter. A good dataset records what the robot saw, what state it was in, what command was sent, when everything happened, and how the episode ended.

---

## 🧠 Core Concepts

- **Episode / Trajectory**: Sequence of observations, actions, rewards, and done flags.
- **Observation Modalities**: Images, depth, proprioception, force, audio, or language.
- **Action Space**: Joint targets, end-effector deltas, gripper commands, or torques.
- **Metadata**: Robot model, camera calibration, task name, operator, success label.
- **Chunking**: Splitting long trajectories for sequence models.
- **Compression**: Video or array compression for large datasets.

---

## 📊 Comparison Chart

| Format | Strength | Weakness | Common Use | Robotics Fit |
|---|---|---|---|---|
| [[HDF5]] | Mature hierarchical storage | Can be awkward at scale | [[robomimic]] datasets | High |
| Zarr | Chunked cloud-friendly arrays | More ecosystem choices | Large array datasets | High |
| RLDS | Standard RL episodes | TensorFlow ecosystem weight | Robot/agent datasets | High |
| LeRobotDataset | Robot-learning workflow | Young format | [[LeRobot]] | Very high |
| ROS bags | Native ROS recording | Not training-ready directly | Sensor and robot logs | High |
| Parquet | Tabular analytics | Not ideal for video tensors | Metadata and metrics | Medium |

---

## ✅ Pros

- Makes robot data reusable across algorithms.
- Supports imitation learning and offline RL.
- Preserves experiment context for debugging.
- Enables dataset validation before training.
- Helps compare sim and real rollouts.

---

## ❌ Cons

- Video and multi-camera data become large quickly.
- Timestamp mismatch can silently break learning.
- Converting between formats can lose metadata.
- Different tools expect different action conventions.
- Dataset schemas evolve quickly in robot learning.

---

## 🧰 Minimum Fields

- Episode ID and step index.
- Observation timestamp.
- Camera frames or image paths.
- Joint positions, velocities, and gripper state.
- Raw action command.
- Reward, terminal flag, and success label if available.
- Robot, camera, and task metadata.

---

## 🔗 Related Notes

- [[LeRobot]]
- [[robomimic]]
- [[DROID]]
- [[Open X-Embodiment]]
- [[HDF5]]
- [[Parquet]]
- [[Robot Data Collection and Teleoperation]]

---

## 📝 Summary

Robot dataset formats should preserve timing, action semantics, observations, and metadata. For imitation and offline RL, dataset correctness is part of the learning algorithm.
