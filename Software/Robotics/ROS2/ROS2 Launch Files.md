# ROS2 Launch Files

ROS2 Launch Files are Python-based scripts used to start and configure multiple ROS2 nodes and subsystems simultaneously. They provide a structured way to initialize complex robotic systems by declaring parameters, remapping topics, and composing nodes dynamically at runtime.

---

## 📚 Overview

In contrast to ROS1's XML-based launch files, ROS2 uses Python for its launch system, enabling logic, conditions, loops, and substitutions. Launch files often load parameters from YAML, start lifecycle-managed nodes, and configure namespaces, node names, and topic remappings.

---

## 🧠 Core Concepts

- **Launch Description**: A Python function returning a `LaunchDescription` object with launch actions.
- **Nodes**: ROS2 nodes started by the launch file, possibly with parameters and remappings.
- **Parameters**: Passed inline or via YAML config files.
- **Substitutions**: Dynamic values like `LaunchConfiguration`, `PathJoinSubstitution`, etc.
- **Launch Arguments**: User-defined CLI arguments passed at runtime.

---

## 🧰 Use Cases

- Starting up robot software stacks (e.g., perception, control, navigation)
- Setting ROS2 parameters and topic remappings
- Launching simulation environments
- Managing multi-node distributed systems
- Conditional startup of components (e.g., camera only if available)

---

## ✅ Pros

- Python gives expressive power (conditions, loops)
- Supports composable nodes and lifecycle management
- Can load YAML parameters and include other launch files
- Reusable and modular

---

## ❌ Cons

- More complex than ROS1 XML-style
- Debugging may be harder for new users unfamiliar with Python
- Dependency on launch-specific APIs (e.g., `launch_ros`)

---

## 📊 Comparison Chart

| Feature                 | ROS1 Launch Files     | ROS2 Launch Files     | Systemd Scripts     |
|-------------------------|------------------------|------------------------|----------------------|
| **Language**            | XML                    | Python                 | Bash-like            |
| **Parameters**          | YAML/XML               | YAML or inline Python  | Limited              |
| **Node Composition**    | ❌ No                  | ✅ Yes                 | ❌ No                |
| **Conditionals/Logic**  | ❌ Limited             | ✅ Pythonic            | ✅ Bash/Python        |
| **Best For**            | Simple setups          | Complex robotics       | OS-level services     |

---

## 🔧 Compatible Items

- `ros2 launch` CLI tool
- `launch` and `launch_ros` Python modules
- YAML parameter files via `params_file`
- [[ROS2 Parameters]] (Typically loaded via launch)
- [[Lifecycle Nodes]] (Managed through launch)
- [[ROS2 Node]], [[ROS2 Topics]], [[ROS2 Actions]]

---

## 🔗 Related Concepts

- [[ROS2 Parameters]] (Pass into nodes during launch)
- [[ROS2 Node]] (Created and launched via launch files)
- [[ROS2 Topics]] (Remapped or namespaced via launch)
- [[Lifecycle Nodes]] (Often managed in launch scripts)
- [[Navigation2]] (Launched via complex structured launch files)
- [[Sensor Integration]] (Sensors launched and configured via launch files)

---

## 🛠 Developer Tools

- `ros2 launch <package> <file.py>`
- `LaunchConfiguration` for user-set CLI args
- `Node` action from `launch_ros.actions`
- `IncludeLaunchDescription` for modular composition
- Visual debugging via `rqt_graph` and log outputs

---

## 📚 Further Reading

- [ROS2 Launch System Overview](https://docs.ros.org/en/foxy/Tutorials/Launch-Files/Creating-Launch-Files.html)
- [Launch API Reference](https://docs.ros.org/en/foxy/How-To-Guides/Launch-system.html)
- [Examples of Launch Substitutions](https://github.com/ros2/launch_ros/tree/foxy/launch_ros/launch_ros/substitutions)

---
