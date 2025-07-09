# ROS2 Actions

ROS2 Actions are a communication mechanism in the Robot Operating System 2 (ROS2) designed for long-running, goal-oriented tasks that need feedback and the ability to be canceled. They combine elements of services (goal/result communication) and topics (streaming feedback), making them ideal for tasks like navigation, motion planning, or any operation that evolves over time.

---

## 📚 Overview

An action is defined using an `.action` file with three sections: **Goal**, **Result**, and **Feedback**. The client sends a goal to the server, which processes it asynchronously and sends periodic feedback. The client can also cancel the goal if needed. ROS2 Actions use a combination of topics and services under the hood to implement this pattern.

---

## 🧠 Core Concepts

- **Goal**: The desired outcome or task to be performed (e.g., move to a position).
- **Feedback**: Intermediate progress updates while executing the goal.
- **Result**: Final outcome once the action finishes (successfully or not).
- **Cancellation**: Clients can cancel active goals mid-execution.
- **Action Client & Server**: Nodes that send and execute goals, respectively.

---

## 🧰 Use Cases

- Robot navigation (e.g., move to a goal location)
- Arm manipulation (e.g., grasp or place an object)
- Autonomous docking
- Path following
- Object tracking or exploration tasks

---

## ✅ Pros

- Ideal for time-consuming or interruptible operations
- Supports continuous feedback for monitoring
- Built-in goal cancelation and result reporting

---

## ❌ Cons

- More complex than services or topics
- Not ideal for high-frequency, real-time tasks
- Requires more boilerplate setup and understanding

---

## 📊 Comparison Chart

| Mechanism  | Communication Type | Pattern                | Use Case                       | Notes                           |
|------------|--------------------|------------------------|--------------------------------|---------------------------------|
| **[[ROS2 Topics]]** | Asynchronous       | Pub-Sub                | Streaming sensor data          | No feedback or control flow     |
| **[[ROS2 Services]]** | Synchronous      | Request-Response       | Configuration or queries       | Blocking, simple tasks          |
| **[[ROS2 Actions]]** | Asynchronous       | Goal-Feedback-Result   | Long-running tasks with feedback | Cancellable, non-blocking     |
| **[[ROS2 Parameters]]** | Queryable       | Get/Set Config         | Persistent system settings     | Not for dynamic behavior        |

---

## 🔧 Compatible Items

- `rclcpp_action::Client` / `Server` (C++)
- `rclpy.action.ActionClient` / `ActionServer` (Python)
- `ros2 action send_goal`, `ros2 action list`, `ros2 action show`
- [[ROS2 Navigation Stack]] (Uses actions for movement)
- [[MoveIt2]] (Uses actions for robot arm goals)

---

## 🔗 Related Concepts

- [[ROS2 Services]] (Synchronous task communication)
- [[ROS2 Topics]] (For continuous data streams)
- [[ROS2 Node]] (Action clients and servers run in nodes)
- [[Behavior Trees]] (Often built atop action goals)
- [[Goal Pose]] (Common goal format for navigation or manipulation)

---

## 🛠 Developer Tools

- `ros2 action send_goal` for testing from CLI
- `ros2 action list`, `ros2 action show` for introspection
- RQt Action Client plugin for GUI testing
- `.action` files stored under the `action/` folder in packages
- Build integration via `CMakeLists.txt` and `setup.py`

---

## 📚 Further Reading

- [ROS2 Actions Design](https://design.ros2.org/articles/actions.html)
- [Official ROS2 Actions Tutorials](https://docs.ros.org/en/foxy/Tutorials/Actions.html)
- Navigation2 and MoveIt2 action interface examples

---
