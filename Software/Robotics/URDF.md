# 🤖 URDF (Unified Robot Description Format)

**URDF** stands for **Unified Robot Description Format**, a widely used XML specification for defining the physical and visual properties of a robot. URDF is most commonly associated with [[ROS]] and [[ROS2]], serving as the foundational structure for simulating, visualizing, and understanding robotic models.

URDF is designed to describe:

- A robot’s **links** (rigid bodies)
- **Joints** connecting links (and their constraints)
- **Inertial** properties
- **Visual** and **collision** geometry
- Optional hardware interfaces

---

## 🧠 What is URDF?

URDF is **not a full simulation format**, but rather a structural blueprint. It doesn't include dynamics, sensors, actuators, or environment info. It is meant to model robots in a format easily parsed by ROS tools like `rviz`, `robot_state_publisher`, and robot control stacks.

URDF files are often `.urdf` or `.xacro` (if preprocessed macros are used).

---

## 🛠️ Core Elements of URDF

| Element        | Description |
|----------------|-------------|
| `<robot>`      | The root element of the URDF |
| `<link>`       | Defines a single rigid body |
| `<joint>`      | Defines the relationship between two links |
| `<inertial>`   | Specifies mass, center of mass, inertia matrix |
| `<visual>`     | Appearance of the link in visualization |
| `<collision>`  | Collision geometry for physics |
| `<transmission>` | Maps joint to actuator, for use with controllers |
| `<material>`   | Visual color or texture reference |

---

## 🧩 Common Tools Using URDF

- `rviz` — visualizes the robot model
- `gazebo_ros` — URDF can be extended for simulation (though [[SDF]] is often better)
- `robot_state_publisher` — broadcasts transforms (TF2)
- `moveit` — motion planning integration
- `urdf_parser_py` and `urdfdom` — parsing libraries

---

## 🔀 Comparison with Similar Formats

| Format   | Description Scope     | Use Case                          | ROS Compatible | XML/Other |
|----------|------------------------|------------------------------------|----------------|------------|
| URDF     | Robot-only, static     | Structure and visualization        | ✅              | XML        |
| [[SDF]]  | Robot + env + physics  | Full simulation in [[Gazebo]]      | ✅              | XML        |
| [[XACRO]]| URDF + macros          | DRY URDF code                      | ✅              | XML+macro  |
| MJCF     | MuJoCo format          | Physics-focused simulation         | ⚠️ Partial      | XML        |
| UDRF     | Unified Description    | ROS3 concept (not official yet)    | 🚧              | XML        |

---

## ✅ Pros

- Well-integrated into ROS/ROS2
- Simple and human-readable
- Easy to visualize and debug
- Good ecosystem support
- XACRO support makes complex robots manageable

## ❌ Cons

- No native support for sensors or environment
- Can't model complex dynamics
- Requires XACRO or SDF for more advanced modeling
- Lacks plugin systems

---

## 🌐 URDF in Simulation

URDF can be loaded into simulation environments such as:

- [[Gazebo]] (via `gazebo_ros`)
- [[Ignition Gazebo]] (via conversion or bridge)
- [[Webots]]
- [[CoppeliaSim]] (with converters)

However, URDF often needs extensions (`<gazebo>` tags or conversion to [[SDF]]) for full simulation features like physics properties, sensors, or plugins.

---

## 🧩 Supported by

- ROS (1 and 2)
- MoveIt
- [[RViz]]
- Gazebo (via plugins or conversion)
- [[Unity]] with ROS bridge
- [[Foxglove]] Studio (for visualization)
- URDF-to-USD pipelines for modern rendering

---

## 📌 Best Practices

- Use [[XACRO]] for reusable definitions and macros
- Validate using `check_urdf` and RViz
- Keep geometry simple for visualization and collisions
- Separate visual and collision meshes
- Modularize robot definitions (e.g., one xacro per limb or system)

---

## 🔗 Internal Links

- [[ROS]]
- [[ROS2]]
- [[XACRO]]
- [[SDF]]
- [[Ignition]]
- [[Gazebo]]
- [[Simulation Tools]]
- [[Robot Description]]
- [[Kinematics]]
- [[Collision Detection]]

---

## 🌍 External Resources

- [ROS URDF Tutorial (ros.org)](http://wiki.ros.org/urdf/Tutorials)
- [URDF on Gazebo Documentation](https://gazebosim.org/tutorials?tut=ros_urdf)
- [URDF and XACRO Examples](https://github.com/ros/urdf_tutorial)
- [URDF in ROS2](https://docs.ros.org/en/foxy/Tutorials/URDF/Using-URDF-In-Robot-State-Publisher.html)
