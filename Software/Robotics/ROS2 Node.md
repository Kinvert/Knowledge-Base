# 🟣 ROS 2 Node

A **ROS 2 Node** is the fundamental executable entity in the [[ROS2]] framework. Nodes represent independent modules of software that perform computation, communicate with other nodes, and interact with the world through topics, services, actions, and parameters.

---

## 🧠 Summary

- A node is a process that performs computation in ROS 2.
- Each node is uniquely named and can be launched individually or as part of a system.
- Nodes use the underlying DDS (Data Distribution Service) middleware for communication.
- Designed for modularity, enabling distributed and scalable robotics systems.

---

## ⚙️ Key Features

- **Topics:** Nodes publish or subscribe to topics for message passing (pub/sub model).
- **Services:** Nodes can provide or call services for request/response communication.
- **Actions:** Nodes can send goals and receive feedback/results over time.
- **Parameters:** Nodes can expose and modify parameters for configuration at runtime.
- **Lifecycle management:** Nodes can have managed states (configured, activated, deactivated, etc.).
- **Multilingual:** Nodes can be written in C++, Python, Rust, and other languages with ROS 2 client libraries.

---

## 🚀 Example Applications

- Sensor nodes (e.g., publishing camera or lidar data)
- Control nodes (e.g., motor controllers)
- Data processing nodes (e.g., point cloud filters)
- Planning nodes (e.g., path planners)

---

## 🏆 Strengths

- Highly modular: promotes distributed, loosely coupled architectures.
- DDS-based communication: scalable, real-time, and reliable.
- Rich ecosystem of reusable nodes for common robotics tasks.

---

## ⚠️ Weaknesses

- Requires careful management of namespaces and node names in complex systems.
- DDS configuration can add complexity for beginners.
- Performance depends on appropriate node granularity (too many nodes = overhead, too few = reduced modularity).

---

## 📊 Comparison to ROS 1 Nodes

| Feature               | ROS 1 Node | ROS 2 Node |
|-----------------------|------------|------------|
| Communication layer    | Custom TCP/UDP | DDS (Data Distribution Service) |
| Lifecycle management   | Basic | Advanced (lifecycle nodes, managed states) |
| Real-time support      | Limited | Improved, DDS helps |
| Language support       | C++, Python | C++, Python, Rust, others |
| Security               | Limited | DDS-based security options |

---

## 🔗 Related Notes

- [[ROS2]]
- [[DDS]] (Data Distribution Service)
- [[ROS2 Topics]]
- [[ROS2 Services]]
- [[ROS2 Actions]]
- [[ROS2 Parameters]]
- [[Node Lifecycle]]

---

## 🌐 External References

- [ROS 2 Documentation - Nodes](https://docs.ros.org/en/foxy/Concepts/Basic/About-Nodes.html)
- [ROS 2 Design - Node Lifecycle](https://design.ros2.org/articles/node_lifecycle.html)

---
