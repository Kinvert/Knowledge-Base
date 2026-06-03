# 🎮 Robot Data Collection and Teleoperation

**Robot Data Collection and Teleoperation** covers the workflows for gathering demonstrations and robot experience for imitation learning, offline RL, debugging, and evaluation. It is the practical foundation for [[LeRobot]], [[robomimic]], [[Diffusion Policy]], and [[ACT Action Chunking Transformer]].

---

## 📚 Overview

Modern robot learning often depends more on data quality than model novelty. Demonstrations must have synchronized cameras, robot state, actions, task metadata, reset labels, and success labels. Teleoperation quality also affects whether a policy learns smooth, recoverable behavior.

---

## 🧠 Core Concepts

- **Teleoperation Device**: Keyboard, gamepad, SpaceMouse, VR controller, or leader arm.
- **Episode**: One task attempt from reset to success, failure, or timeout.
- **Timestamp Alignment**: Synchronizing images, states, and actions.
- **Action Label**: Command sent to the robot, not just measured joint motion.
- **Success Label**: Whether the episode completed the task.
- **Reset Protocol**: How the robot and scene return to initial conditions.
- **Dataset Schema**: File layout and metadata used for training.

---

## 📊 Comparison Chart

| Collection Method | Strength | Weakness | Best For | Example Use |
|---|---|---|---|---|
| Keyboard/gamepad | Cheap and easy | Low precision | Simple tasks | Mobile robots, grippers |
| SpaceMouse | Smooth 6D control | Needs mapping | Arm teleop | End-effector control |
| VR teleop | Natural motion | Setup complexity | Manipulation | Bimanual demos |
| Leader-follower arm | High-quality demos | Requires extra hardware | Fine manipulation | ALOHA-style systems |
| Scripted policy | Scalable labels | Less diverse behavior | Sim datasets | Benchmark generation |
| Kinesthetic teaching | Direct physical demos | Robot-dependent | Collaborative arms | Pick/place demos |

---

## ✅ Pros

- Enables real robot imitation learning.
- Produces logs for Sim2Real debugging.
- Supports offline RL and behavior cloning.
- Reveals hardware and timing issues early.
- Creates reusable datasets for future methods.

---

## ❌ Cons

- Data collection is slow and repetitive.
- Poor synchronization can ruin a dataset.
- Human demonstrations can be inconsistent.
- Resets may dominate experiment time.
- Dataset cleanup and metadata are easy to neglect.

---

## 🧰 Practical Checklist

1. Record images, robot state, actions, timestamps, and episode metadata.
2. Store raw commands separately from measured state.
3. Mark success, failure, timeout, and reset reason.
4. Keep camera calibration and robot configuration with the dataset.
5. Validate playback before training.
6. Start with simple behavior cloning before advanced policies.

---

## 🔗 Related Notes

- [[LeRobot]]
- [[robomimic]]
- [[Robot Dataset Formats]]
- [[Imitation Learning]]
- [[Diffusion Policy]]
- [[ACT Action Chunking Transformer]]
- [[HDF5]]

---

## 📝 Summary

Robot data collection is a systems problem: hardware, timing, metadata, storage, and operator skill all affect policy quality. Good data makes advanced models useful. Bad data makes them misleading.
