# ROS2 Interface Definition

ROS2 Interface Definition refers to the standardized way of specifying messages, services, and actions in ROS2 using Interface Definition Language (IDL). These interface files—`.msg`, `.srv`, and `.action`—are the foundation for data communication in a ROS2 system, enabling type-safe, language-agnostic interoperability between nodes.

---

## 📚 Overview

ROS2 interface files define the structure of communication data and are stored in the `msg/`, `srv/`, and `action/` directories within a ROS2 package. Once defined, they are compiled into language-specific types for use in C++, Python, and other supported client libraries.

ROS2 uses IDL (Interface Definition Language) under the hood, aligning with DDS (Data Distribution Service) standards to enable reliable real-time communication.

---

## 🧠 Core Concepts

- **.msg Files**: Define data structures used in topics.
- **.srv Files**: Define request-response pairs for services.
- **.action Files**: Define goal, feedback, and result messages for actions.
- **IDL Mapping**: ROS2 types map to IDL and DDS types, enabling DDS middleware compatibility.
- **Interface Package**: A ROS2 package that contains only `.msg`, `.srv`, and `.action` files for reuse.

---

## 🧰 Use Cases

- Defining custom data formats for publishing/subscribing
- Creating reusable service interfaces
- Structuring long-running task definitions with feedback
- Sharing standard message contracts between teams and nodes
- Enabling cross-language compatibility and introspection

---

## ✅ Pros

- Strongly typed interfaces improve reliability
- Fully compatible with DDS transport
- Extensible and modular
- Easily reused across multiple nodes and packages

---

## ❌ Cons

- Requires package build steps to regenerate types
- Interface changes require careful version management
- Not dynamically typed—data structures are fixed at compile time

---

## 📊 Comparison Chart

| Interface Type | Extension | Purpose                      | Fields/Sections                  | Example                          |
|----------------|-----------|------------------------------|----------------------------------|----------------------------------|
| **Message**    | `.msg`    | Topic communication          | Fields                           | `std_msgs/String.msg`            |
| **Service**    | `.srv`    | Synchronous interaction      | Request / Response               | `std_srvs/Empty.srv`             |
| **Action**     | `.action` | Long-running tasks w/ feedback | Goal / Feedback / Result       | `nav2_msgs/NavigateToPose.action`|

---

## 🔧 Compatible Items

- `ros2 interface show`, `ros2 interface list`
- `ros2 msg`, `ros2 srv`, `ros2 action` CLI tools
- `rclcpp::msg::Type`, `rclpy.msg.Type`
- [[ROS2 Messages]], [[ROS2 Services]], [[ROS2 Actions]]
- [[ROS2 Publishers]], [[ROS2 Subscribers]]
- [[ROS2 QoS Policies]] (Affect message delivery for these interfaces)

---

## 🔗 Related Concepts

- [[ROS2 Messages]] (Defined with `.msg` files)
- [[ROS2 Services]] (Defined with `.srv` files)
- [[ROS2 Actions]] (Defined with `.action` files)
- [[ROS2 Node]] (Use interface types in publishers/subscribers)
- [[DDS]] (ROS2 interfaces compile to DDS-compatible IDL)

---

## 🛠 Developer Tools

- `ros2 interface show <pkg/type>` (e.g. `ros2 interface show std_msgs/msg/String`)
- `ros2 interface list` to browse available types
- Define `.msg`, `.srv`, `.action` files in `msg/`, `srv/`, `action/` folders
- Update `CMakeLists.txt` and `package.xml` for build integration

---

## 📚 Further Reading

- [ROS2 Interface Overview](https://docs.ros.org/en/foxy/Concepts/About-ROS-Interfaces.html)
- [Custom Interface Tutorial](https://docs.ros.org/en/foxy/Tutorials/Custom-ROS2-Interfaces.html)
- [IDL Syntax](https://design.ros2.org/articles/interface_definition.html)

---

## 🗂 Suggested Folder Location

Software > Robotics > ROS2  
or  
Protocols > Robotics and Industrial  
or  
Software > Data Formats
