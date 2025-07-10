# ROS2 Package

A **ROS2 Package** is the fundamental building block of any ROS2-based application. It is a self-contained unit that includes code, build files, dependencies, configuration, launch scripts, and more. ROS2 packages are modular, reusable, and built using tools like `colcon`, making them ideal for developing scalable robotics software.

---

## 📚 Overview

A ROS2 package is a directory with a standardized structure, including a `package.xml` manifest and a `CMakeLists.txt` or `setup.py`, depending on whether the package is written in C++ or Python. These packages can contain nodes, libraries, interfaces, tests, and launch files, and are typically built and managed using `colcon`.

---

## 🧠 Core Concepts

- **`package.xml`**: Defines metadata, dependencies, and version
- **Build System**: Uses `ament_cmake` (C++) or `ament_python` (Python)
- **`CMakeLists.txt`**: Defines build process and exported targets (for C++)
- **Launch Files**: Written in Python and used to start nodes
- **Interfaces**: Custom `msg`, `srv`, and `action` files can be defined within a package
- **Nodes**: Executable programs that perform robotic functions

---

## 🧰 Use Cases

- Implementing control systems, sensor drivers, and planners
- Organizing robot subsystems (e.g., navigation, localization)
- Defining robot-specific parameters and configurations
- Running simulation setups and visualizations
- Creating libraries for hardware abstraction or data processing

---

## ✅ Pros

- Modularity encourages reuse and scalability
- Well-integrated with the ROS2 ecosystem
- Compatible with both C++ and Python
- Easily packaged for distribution and deployment
- Supports custom interfaces and messages

---

## ❌ Cons

- Dependency management can become complex in large systems
- Build times can be long with many interdependent packages
- Requires adherence to ROS2 conventions and structure
- Debugging can be nontrivial, especially across multiple languages

---

## 📊 Comparison Chart

| Feature                  | ROS2 Package     | ROS1 Package     | CMake Project     | Python Module     | Docker Container    |
|--------------------------|------------------|------------------|-------------------|-------------------|---------------------|
| Language Support         | C++, Python       | C++, Python       | Any               | Python            | Any                 |
| ROS Integration          | ✅ Full           | ✅ Full           | ❌ None           | ❌ None           | ⚠️ External         |
| Inter-package Dependencies | ✅ ROS-specific  | ✅ ROS-specific   | ⚠️ Manual         | ⚠️ Manual         | ⚠️ Layered           |
| Deployment Flexibility   | ✅ High            | ⚠️ Manual         | ⚠️ Manual         | ✅ Pip            | ✅ Docker            |

---

## 🤖 In a Robotics Context

| Example Use Case                   | ROS2 Package Component                          |
|-----------------------------------|--------------------------------------------------|
| IMU Driver                        | C++ Node + Sensor Launch File                    |
| Visual Odometry                   | Python Node + Custom Msg                        |
| SLAM System                       | Nodes + Parameter Config + Launch Tree          |
| Mobile Robot Interface            | URDF + Control Interface + Joint State Broadcaster |
| ROS2 Web Bridge                   | WebSocket Bridge + Service Launch Config         |

---

## 🔧 Useful Commands (One-Liners)

- `ros2 pkg create my_package --build-type ament_cmake` – Create a C++ package  
- `ros2 pkg create my_package --build-type ament_python` – Create a Python package  
- `colcon build` – Build all packages in the workspace  
- `ros2 run my_package my_node` – Run a node from the package  
- `ros2 pkg list` – List all discovered ROS2 packages  
- `ros2 pkg prefix my_package` – Get install prefix of the package  
- `ros2 pkg executables my_package` – List executables in the package  

---

## 🔧 Compatible Items

- [[ROS2 Messages]] – Can be defined within a ROS2 package
- [[ROS2 Publishers]], [[ROS2 Subscribers]], [[ROS2 Services]], [[ROS2 Actions]] – Often live in packages
- [[ROS2 Parameters]] – Used to configure package behavior
- [[ROS2 Launch Files]] – Used to orchestrate multiple nodes
- [[Docker Container]] – Can encapsulate packages for deployment
- [[CI-CD Pipelines]] – Automate building and testing of packages
- [[colcon]] – The official ROS2 build tool

---

## 🔗 Related Concepts

- [[ROS2 Interface Definition]] (For defining custom messages/services)
- [[colcon]] (Build tool used with ROS2 packages)
- [[CMake]] (Used in C++ package builds)
- [[Dockerfile]] (To containerize packages)
- [[GitHub Actions]] / [[Jenkins]] (CI/CD for building/testing packages)
- [[Behavior Trees]] (Can be implemented and launched from packages)

---

## 📚 Further Reading

- [ROS2 Tutorials: Creating a Package](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Package.html)
- [Colcon Build System](https://colcon.readthedocs.io/en/released/)
- [ROS2 Build Type Guide](https://docs.ros.org/en/rolling/How-To-Guides/Ament-CMake-Documentation.html)
- [ament_cmake vs ament_python](https://docs.ros.org/en/rolling/How-To-Guides/Ament-CMake-Documentation.html#ament-python)

---
