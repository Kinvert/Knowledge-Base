# 🤖 ROS 2 (Robot Operating System 2)

ROS 2 is the next-generation version of the Robot Operating System (ROS), designed to support industrial, real-time, and production-grade robotic systems. It provides a modular and open-source software framework for robot development, enabling distributed computing, communication between components, and integration of various sensors and actuators.

---

## 🧭 Overview

| Feature | Details |
|--------|---------|
| Name | ROS 2 (Robot Operating System 2) |
| Initial Release | December 2017 (Ardent Apalone) |
| Latest LTS | [[Rolling]] or [[Humble Hawksbill]] (depending on timeline) |
| License | Apache 2.0 |
| Platform Support | Linux (primary), macOS, Windows |
| Language Support | C++, Python (via `rclpy`), Rust (community), Java (experimental) |
| Core Protocols | DDS (Data Distribution Service), RTPS |
| Main Use Cases | Robotics, autonomous vehicles, drones, industrial automation |

---

## 🧱 Core Concepts

- **Nodes**: Executable units of functionality that perform computation.
- **Topics**: Named buses over which nodes exchange messages (pub/sub).
- **Services**: RPC-style request/response mechanism.
- **Actions**: For long-running tasks with feedback and preemption.
- **Parameters**: Runtime configuration of nodes.
- **Launch System**: XML or Python-based launch files to run multiple nodes.
- **Middleware**: DDS handles real-time publish/subscribe transport layer.

---

## 🔁 ROS 2 vs ROS 1

| Feature | ROS 1 | ROS 2 |
|--------|--------|-------|
| Middleware | Custom TCP/UDP | DDS (Real-time capable) |
| Real-Time Support | Limited | Yes (RTOS, priority scheduling) |
| OS Support | Primarily Linux | Linux, Windows, macOS |
| Security | Minimal | SROS 2 (security layer via DDS-Security) |
| Modularity | Moderate | Improved modularity |
| Multi-Robot Support | Complex | Improved |
| Communication | ROS TCPROS | DDS / RTPS |

---

## 🧬 Middleware: DDS

ROS 2 uses DDS (Data Distribution Service) as its core communication middleware. DDS provides:

- Real-time guarantees
- Quality of Service (QoS) configurations
- Decentralized discovery
- Plug-and-play compatibility across implementations (Fast-DDS, Cyclone DDS, RTI Connext, etc.)

DDS allows ROS 2 to be used in mission-critical and safety-sensitive environments like autonomous vehicles and industrial robotics.

---

## ⚙️ Supported Languages

| Language | Notes |
|---------|-------|
| C++ | Core implementation (`rclcpp`) |
| Python | High-level scripting (`rclpy`) |
| Rust | Community crates (`ros2-rust`) |
| Java | Experimental bindings |
| Others | C, Lua (limited), Go (experimental) |

---

## 🔌 Tooling

- **`ros2` CLI**: Manage nodes, topics, services, and parameters.
- **RViz2**: 3D visualization tool.
- **rqt**: Qt-based GUI plugin system for monitoring and debugging.
- **rosbag2**: Data recording and playback.
- **Launch System**: Manage node startup configurations.
- **Colcon**: Build system (replaces `catkin` from ROS 1).
- **DDS Tools**: Some DDS vendors (e.g. RTI) provide visual network analyzers.

---

## 🔐 Security

ROS 2 introduces optional security features via SROS 2:

- Encrypted communication (TLS)
- Node authentication
- Access control
- Logging and audit

---

## 🛠️ Applications & Ecosystem

| Area | Examples |
|------|----------|
| Autonomous Vehicles | [[Autoware]], Apex.AI |
| Drones | PX4, Dronecode |
| Industrial | ABB, Bosch, Amazon Robotics |
| Simulation | Gazebo, Ignition Gazebo |
| Education | TurtleBot3, JetBot |

---

## 💡 Use Cases

- Multi-sensor fusion
- Motion planning and control
- SLAM (Simultaneous Localization and Mapping)
- Human-robot interaction
- Multi-agent coordination
- Perception and vision

---

## 📈 Strengths and Weaknesses

### ✅ Strengths

- Real-time ready via DDS
- Multi-platform support (Linux, Windows, macOS)
- Actively maintained and community-backed
- Scales to distributed and modular systems
- Strong tooling and introspection features

### ❌ Weaknesses

- Steep learning curve
- DDS configuration can be complex
- Windows support less mature than Linux
- Larger system footprint compared to microcontrollers

---

## 🔍 Related Concepts

- [[ROS1]]
- [[DDS]]
- [[eCAL]]
- [[Autoware]]
- [[MicroROS]]
- [[Gazebo]]
- [[RTOS]]

---

## 📚 References

- https://docs.ros.org/en/foxy/index.html
- https://design.ros2.org/
- https://github.com/ros2
  - https://github.com/ros2/ros2_documentation
- https://index.ros.org/packages/
- **Indexes**
  - https://github.com/fkromer/awesome-ros2
- **Examples**
  - https://github.com/ros-controls/ros2_control_demos
- **Drivers**
  - https://github.com/flynneva/bno055
