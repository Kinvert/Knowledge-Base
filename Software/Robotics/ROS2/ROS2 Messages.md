# ROS2 Messages

ROS2 Messages define the structure of the data exchanged between nodes over topics, services, and actions in the Robot Operating System 2 (ROS2). They serve as the fundamental unit of communication and are defined using the Interface Definition Language (IDL) in `.msg` files.

---

## 📚 Overview

Each ROS2 message specifies a set of typed fields—like `int32`, `float64`, `string`, or nested message types. These messages can be simple (e.g., an integer value) or complex (e.g., a full sensor message with timestamp, metadata, and data arrays). ROS2 messages are strongly typed and compiled into code for use in supported client libraries (e.g., C++, Python).

---

## 🧠 Core Concepts

- **.msg Files**: Define message structure; each line specifies a field name and type.
- **Primitive Types**: Basic types like `int32`, `float64`, `bool`, `string`.
- **Composite Types**: Messages can include other messages or arrays.
- **Header**: Often used to include timestamp and frame ID (e.g., `std_msgs/Header`).
- **IDL Compliance**: ROS2 messages are defined in an IDL-compliant format to support DDS.

---

## 🧰 Use Cases

- Sending sensor data (camera, LiDAR, IMU)
- Publishing robot state (pose, velocity, joint positions)
- Transmitting commands and control data
- Structuring service requests and action goals/results/feedback
- Logging diagnostics or system events

---

## ✅ Pros

- Strongly typed communication
- Supports introspection and tooling (e.g., ros2 topic echo)
- Easily extensible with custom message definitions
- Shared between ROS2 languages (C++, Python, etc.)

---

## ❌ Cons

- Requires compilation and regeneration when messages are changed
- Cannot easily send dynamically typed data
- Version mismatches can cause runtime issues if not managed

---

## 📊 Comparison Chart

| Message Type       | Common Use                          | Example Fields                 | Notes                                 |
|--------------------|--------------------------------------|---------------------------------|----------------------------------------|
| `std_msgs/String`  | Simple string communication           | `data: string`                 | Used in tutorials or logs              |
| `geometry_msgs/Pose` | Robot and object pose             | `position`, `orientation`      | 3D transform info                      |
| `sensor_msgs/Image`| Camera image streams                 | `encoding`, `data[]`           | Often used with camera drivers         |
| `nav_msgs/Odometry`| Robot pose and velocity tracking     | `pose`, `twist`                | Used in SLAM/VO                        |
| Custom Msg         | Any application-specific data        | User-defined fields            | Add via `.msg` files in ROS packages   |

---

## 🔧 Compatible Items

- `ros2 msg show <msg_type>` (e.g. `ros2 msg show sensor_msgs/Image`)
- `ros2 interface show`, `ros2 interface list`
- `rclcpp::msg` / `rclpy.msg` in C++/Python nodes
- [[ROS2 Publishers]] and [[ROS2 Subscribers]]
- [[ROS2 Services]] and [[ROS2 Actions]] use message types internally
- [[ROS2 Interface Definition]] (for creating `.msg`, `.srv`, `.action`)

---

## 🔗 Related Concepts

- [[ROS2 Topics]] (Messages are sent over topics)
- [[ROS2 Services]] (Use request and response messages)
- [[ROS2 Actions]] (Use goal, feedback, and result messages)
- [[ROS2 Interface Definition]] (IDL for `.msg`, `.srv`, `.action`)
- [[ROS2 QoS Policies]] (Affect delivery of messages)
- [[ROS2 Publishers]], [[ROS2 Subscribers]]

---

## 🛠 Developer Tools

- `ros2 msg list`, `ros2 msg show`, `ros2 msg package`
- `ros2 interface show` (alternative interface explorer)
- Custom messages: add `.msg` files in your package's `msg/` directory
- Build system integration (`CMakeLists.txt`, `package.xml`)

---

## 📚 Further Reading

- [ROS2 Interface Definition Overview](https://docs.ros.org/en/foxy/Concepts/About-ROS-Interfaces.html)
- [ROS2 Custom Message Tutorial](https://docs.ros.org/en/foxy/Tutorials/Custom-ROS2-Interfaces.html)
- DDS-XTypes specification for IDL compatibility

---
