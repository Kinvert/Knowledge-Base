# 🧩 XACRO (XML Macros for URDF)

**XACRO** stands for **XML Macros**, and it is a preprocessor for URDF (Unified Robot Description Format) files. XACRO allows for better modularity, reuse, and readability when defining complex robot models by extending standard URDF with programming-like features such as macros, variables, and conditional statements.

---

## 📖 Overview

URDF files can become very repetitive and verbose, especially for robots with many similar parts or configurations. XACRO solves this problem by allowing:

- **Macros** to encapsulate repetitive XML blocks.
- **Parameters** and **expressions** for dynamic values.
- **Conditional logic** for defining variants of a robot.
- **Includes** for breaking files into reusable components.

Once processed, XACRO outputs a standard URDF file.

---

## ⚙️ How It Works

- XACRO files use the `.xacro` extension.
- You write URDF-like XML but embed additional tags such as:
  - `<xacro:macro>`, `<xacro:property>`, `<xacro:if>`, etc.
- You use the XACRO command-line tool to convert it into a standard `.urdf` file.

---

## 🧰 Common Use Cases

- Modular design of multi-joint arms and mobile robots.
- Dynamically generating robot models for different configurations (e.g., 4-wheel vs 2-wheel).
- Keeping URDFs DRY (Don't Repeat Yourself).
- Separating mechanical components into logical files (e.g., sensors, base, arms).

---

## 🆚 Comparison: URDF vs XACRO

| Feature                  | URDF        | XACRO       |
|--------------------------|-------------|-------------|
| Macros                   | ❌          | ✅           |
| Conditionals             | ❌          | ✅           |
| Includes / reuse         | ❌          | ✅           |
| Runtime parameterization | ❌          | ✅           |
| File readability         | ⚠️ Verbose  | ✅ Cleaner   |
| Compatibility            | ✅           | ✅ (after preprocessing) |

---

## ✅ Advantages

- Easier to maintain large robot descriptions
- Encourages reuse and modular design
- Simplifies variant generation
- Supports math and parameter logic

## ❌ Disadvantages

- Requires preprocessing step before use in URDF consumers
- Slight learning curve due to new XML tags
- Errors in macros can be hard to trace without careful debugging

---

## 🔗 Internal Links

- [[URDF]]
- [[Robot Description Formats]]
- [[ROS2]]
- [[Simulation Tools]]
- [[Gazebo]]
- [[Ignition]]

---

## 🌍 External Resources

- [ROS Wiki: XACRO](http://wiki.ros.org/xacro)
- [GitHub: ros/xacro](https://github.com/ros/xacro)
- [Tutorial - Using XACRO with URDF](https://wiki.ros.org/xacro/Tutorials)

---
