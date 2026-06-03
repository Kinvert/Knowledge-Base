# 🎛️ ros2_control

**ros2_control** is the ROS 2 framework for connecting robot hardware, simulators, and controllers through a common control interface. It is central to deploying learned policies that output joint commands or high-level setpoints in a [[ROS2]] system.

---

## 📚 Overview

ros2_control separates hardware access from controller logic. Hardware interfaces expose joint states and command interfaces. Controllers consume state and write commands. This separation lets the same controller code run against real hardware, Gazebo, Isaac bridges, or test components when the interfaces match.

---

## 🧠 Core Concepts

- **Controller Manager**: Loads, configures, activates, and deactivates controllers.
- **Hardware Interface**: Defines how robot state and commands are read and written.
- **State Interface**: Exposes values such as position, velocity, and effort.
- **Command Interface**: Accepts commands such as position, velocity, or effort targets.
- **Joint State Broadcaster**: Publishes joint states to ROS topics.
- **Controller Plugin**: A controller such as trajectory, velocity, effort, or custom policy controller.
- **Lifecycle State**: Managed states for safe startup and shutdown.

---

## 📊 Comparison Chart

| Tool | Role | Strength | Weakness | Robotics RL Use |
|---|---|---|---|---|
| **ros2_control** | Hardware/controller interface | Standard ROS2 control path | Requires interface setup | High |
| [[MoveIt]] | Motion planning | Mature planning stack | Not low-level control | High for arms |
| [[Gazebo]] control plugins | Simulation control | Good sim integration | Simulator-specific | Medium-high |
| Custom ROS2 node | Direct commands | Simple for prototypes | Easy to bypass safety | Medium |
| Embedded firmware loop | Low-level control | Deterministic | Less flexible | High |
| [[Isaac Lab]] deployment loop | Policy evaluation | Good training bridge | Not full ROS control | Medium |

---

## ✅ Pros

- Standardizes controller and hardware integration in ROS 2.
- Supports simulation and real hardware through similar interfaces.
- Works well with joint trajectory, position, velocity, and effort controllers.
- Encourages lifecycle-managed startup and shutdown.
- Useful boundary between learned policy and motor-level control.

---

## ❌ Cons

- Initial setup can be verbose.
- Real-time behavior depends on hardware, OS, executor, and controller design.
- Custom learned-policy controllers require careful timing and safety design.
- Debugging interface mismatches can be tedious.
- Does not solve policy training or Sim2Real by itself.

---

## 🧰 RL Deployment Pattern

1. Use ros2_control to expose joint states and command interfaces.
2. Run the learned policy in a separate node or custom controller.
3. Convert policy output into allowed position, velocity, or effort commands.
4. Use watchdogs and fallback controllers.
5. Log policy outputs and controller state for [[Sim2Real]] debugging.

---

## 🔗 Related Notes

- [[ROS2]]
- [[Robot Policy Deployment]]
- [[Safe RL for Robotics]]
- [[MoveIt]]
- [[URDF]]
- [[Joint State]]
- [[PID Controller]]

---

## 🌐 External Resources

- ros2_control Docs: https://control.ros.org/
- ros2_control Demos: https://github.com/ros-controls/ros2_control_demos

---

## 📝 Summary

ros2_control is the practical ROS 2 boundary between policies, controllers, and hardware. For robotics RL, it is where a learned action becomes a constrained robot command.
