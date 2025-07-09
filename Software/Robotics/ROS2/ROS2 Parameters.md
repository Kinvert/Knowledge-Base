# ROS2 Parameters

ROS2 Parameters allow nodes to store and retrieve named values at runtime, providing a powerful mechanism for configuration and behavior control without recompilation. Parameters can be set via launch files, configuration files (YAML), or dynamically through command-line tools or API calls.

---

## 📚 Overview

Parameters in ROS2 are key-value pairs associated with a specific node. Each node has its own parameter namespace, and parameters can be declared with types like `bool`, `int`, `float`, `string`, or arrays. They are commonly used for tuning algorithms, setting operational modes, or configuring hardware interfaces.

---

## 🧠 Core Concepts

- **Declaration**: Parameters must be explicitly declared before use.
- **Parameter Server**: In ROS1, this was global; in ROS2, parameters are local to nodes.
- **Dynamic Update**: Parameters can be modified at runtime if allowed by the node.
- **YAML Configs**: Parameters can be loaded from structured YAML files.
- **Lifecycle Nodes**: Parameters are often tied to node state transitions.

---

## 🧰 Use Cases

- Tuning control loop gains (e.g., PID values)
- Defining frame IDs, topic names, or file paths
- Setting flags for debugging or feature toggles
- Configuring sensor properties (e.g., update rate)
- Passing runtime arguments from launch files

---

## ✅ Pros

- Supports dynamic runtime configuration
- Type-safe with validation
- YAML-based launch file support
- Fully integrated into ROS2 node lifecycle

---

## ❌ Cons

- Requires upfront declaration before use
- Not shared globally across nodes
- Complex for deeply nested configs or large parameter sets

---

## 📊 Comparison Chart

| Feature            | ROS1 Parameters    | ROS2 Parameters    | Environment Variables | YAML/INI Config Files |
|--------------------|--------------------|---------------------|------------------------|------------------------|
| **Scope**          | Global              | Node-local          | Process-global         | File/global            |
| **Type Safety**    | ❌ No              | ✅ Yes              | ❌ No                  | ⚠️ Depends on parser  |
| **Dynamic Update** | Limited             | ✅ Yes              | ❌ No                  | ❌ No (static only)    |
| **Integration**    | ✅ Yes              | ✅ Yes              | ❌ No                  | ✅ External only       |

---

## 🔧 Compatible Items

- `rclcpp::Node::declare_parameter()` / `rclpy.node.declare_parameter()`
- `rclcpp::Node::get_parameter()` / `rclpy.node.get_parameter()`
- `ros2 param get`, `ros2 param set`, `ros2 param list`
- YAML configuration via `--params-file` in launch
- [[ROS2 Node]]
- [[ROS2 Launch Files]]
- [[Lifecycle Nodes]]

---

## 🔗 Related Concepts

- [[ROS2 Node]] (Parameters are scoped per node)
- [[ROS2 Launch Files]] (Use YAML or CLI to set parameters)
- [[Lifecycle Nodes]] (Use parameters during configuration state)
- [[ROS2 Topics]] (Parameters may affect publisher/subscriber behavior)
- [[Dynamic Reconfigure]] (Related concept in ROS1)

---

## 🛠 Developer Tools

- `ros2 param list`, `ros2 param get`, `ros2 param set`
- YAML parameter files: `my_node_params.yaml`
- `rqt_reconfigure` (may be available for dynamic parameter tuning)
- `ros2 launch` with `--params-file` or `ros2 run` with `--ros-args`

---

## 📚 Further Reading

- [ROS2 Parameter Documentation](https://docs.ros.org/en/foxy/How-To-Guides/Using-Parameters-In-A-Class-CPP.html)
- [ROS2 YAML Parameter Files](https://docs.ros.org/en/foxy/How-To-Guides/Using-Parameters-CPP.html)
- [Lifecycle Node Management](https://docs.ros.org/en/foxy/How-To-Guides/Node-Lifecycle-Manager.html)

---
