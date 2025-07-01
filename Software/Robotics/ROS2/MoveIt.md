# 🤖 MoveIt

**MoveIt** is an open-source motion planning framework primarily used with the [[ROS2]] (and ROS 1) robotics ecosystem. It provides a comprehensive suite of tools for motion planning, kinematics, collision checking, trajectory execution, and more—enabling robots to interact with their environment in a safe and intelligent way.

---

## 🧠 Summary

- Developed originally by Willow Garage and now maintained by PickNik Robotics and the broader ROS community.
- Integrates multiple motion planning libraries, including [[OMPL]], [[CHOMP]], and [[STOMP]].
- Designed to simplify robot arm planning, grasping, navigation, and perception integration.
- Comes with GUI tools like **MoveIt Setup Assistant** and **MoveIt Motion Planning Plugin** for [[RViz]].

---

## ⚙️ Key Features

| Feature                     | Description                                                                 |
|-----------------------------|-----------------------------------------------------------------------------|
| **Motion Planning**         | Supports multiple algorithms via plugins like [[OMPL]], [[CHOMP]], etc.     |
| **Kinematics**              | Uses plugins (e.g. KDL, TracIK) for forward/inverse kinematics.             |
| **Collision Checking**      | Built-in collision detection using [[FCL]] or [[Bullet]].                   |
| **Path Planning**           | Includes planning pipelines and goal constraints.                          |
| **Perception Integration**  | Works with perception stacks (e.g., camera + depth sensors) for planning.  |
| **Grasping Support**        | Includes grasp generation, filtering, and execution.                       |
| **Real-time Control**       | Works with ROS controllers for executing plans.                            |
| **Simulation**              | Compatible with simulators like [[Gazebo]] and [[Ignition]].               |

---

## 🧩 Common Components

- `move_group`: The main node that handles planning and execution.
- `planning_interface`: APIs for C++ and Python integration.
- `moveit_commander`: Python API wrapper for MoveIt.
- `moveit_setup_assistant`: GUI for setting up MoveIt configurations.
- `planning_scene`: 3D scene representation including robot state and obstacles.

---

## 🧪 Typical Workflow

1. **Create URDF/XACRO**: Define your robot's structure.
2. **Use Setup Assistant**: Generate MoveIt configuration package.
3. **Define Planning Pipelines**: Choose your planners and kinematics solvers.
4. **Publish Robot State**: Use `robot_state_publisher` and `joint_state_publisher`.
5. **Execute Plans**: Plan and execute motions with `move_group` or Python APIs.

---

## 🔄 MoveIt vs OMPL

| Feature              | MoveIt                            | [[OMPL]]                           |
|----------------------|-----------------------------------|------------------------------------|
| Purpose              | Full motion planning framework     | Motion planning *library*          |
| Integration          | ROS-native                         | ROS-compatible (via MoveIt)        |
| GUI Tools            | Setup Assistant, RViz plugin       | None                               |
| Extensibility        | Plugins for kinematics, collision  | Algorithm-focused                  |
| Simulation Ready     | Yes                                | No direct simulation integration   |
| Higher-Level Features| Yes (grasping, perception, etc.)   | No                                 |

> 🧠 **Note**: OMPL is a planning backend *used within* MoveIt for path planning.

---

## 🏗️ Supported Planners

- [[OMPL]] (default path planning backend)
- [[CHOMP]]
- [[STOMP]]
- [[Pilz Industrial Motion Planner]]
- [[TrajOpt]]

---

## 🧰 Common Use Cases

- Robotic arm motion planning in industrial environments
- Mobile manipulation with navigation + arm coordination
- Integration with perception systems for obstacle-aware planning
- Simulating robot movements before deployment

---

## 🏆 Strengths

- Very mature and actively maintained
- Rich ROS2 integration
- Works with many robot models
- High-level abstraction of planning components
- Excellent RViz visualization support

---

## ⚠️ Weaknesses

- Initial configuration can be complex
- Computationally heavy for low-power systems
- Advanced use cases may require customization of planning pipelines

---

## 🔗 Related Topics

- [[OMPL]]
- [[CHOMP]]
- [[ROS2]]
- [[Gazebo]]
- [[Ignition]]
- [[URDF]]
- [[XACRO]]
- [[MoveIt Servo]]
- [[FCL]] (Flexible Collision Library)

---

## 🌐 External References

- [Official Website](https://moveit.picknik.ai/)
- [GitHub Repository](https://github.com/ros-planning/moveit2)
- [MoveIt Tutorials (ROS2)](https://moveit.picknik.ai/humble/index.html)
- [MoveIt Setup Assistant Docs](https://moveit.picknik.ai/humble/docs/setup_assistant/)

---
