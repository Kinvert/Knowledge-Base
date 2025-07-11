# ament_cmake

**ament_cmake** is a build system and package management tool specifically designed for ROS 2 (Robot Operating System 2) packages that use CMake. It extends CMake with ROS 2-specific macros and functions to simplify building, testing, and installing ROS 2 components.

---

## 📚 Overview

ament_cmake is part of the `ament` build system family in ROS 2, built to replace the legacy `catkin` system used in ROS 1. It integrates tightly with ROS 2’s architecture and conventions, providing enhanced support for building libraries, executables, message generation, and test infrastructure.

---

## 🧠 Core Concepts

- **CMake-based**: Uses standard CMake as its foundation, extended with ROS 2 utilities  
- **Package Manifest**: Relies on `package.xml` to define package metadata and dependencies  
- **ament_tools**: CLI and tooling for building and testing ament_cmake packages  
- **Dependency Management**: Supports find_package and ament-specific dependency macros  
- **Testing Support**: Integrates with CTest and GoogleTest frameworks  
- **Installation Targets**: Defines clear install rules for libraries, executables, and message files  

---

## 🧰 Use Cases

- Building ROS 2 packages written in C++ using CMake  
- Managing complex package dependencies in robotics projects  
- Generating and installing ROS 2 interface files (messages, services)  
- Automating testing of ROS 2 components  
- Integrating with ROS 2 workspace tools (`colcon`) for multi-package builds  

---

## ✅ Pros

- Native ROS 2 build system with official support  
- Leverages standard CMake, minimizing learning curve for C++ developers  
- Strong integration with ROS 2 package and dependency model  
- Supports automated testing and coverage reporting  
- Works seamlessly with ROS 2 workspace tools like `colcon`  

---

## ❌ Cons

- Limited to ROS 2 and CMake-based packages (not for pure Python or other build systems)  
- Can be complex for small or very simple packages  
- Requires understanding of both CMake and ROS 2 build conventions  

---

## 📊 Comparison Chart: ament_cmake vs catkin vs ament_python

| Feature                 | ament_cmake           | catkin                 | ament_python         |
|-------------------------|-----------------------|------------------------|----------------------|
| ROS Version             | ROS 2                 | ROS 1                  | ROS 2                |
| Build System           | CMake                 | CMake                  | Python setuptools    |
| Language Support       | C++, C                | C++, C                 | Python               |
| Dependency Management  | ament macros + CMake  | catkin macros + CMake  | setuptools + ament   |
| Testing Integration    | Yes                   | Yes                    | Yes                  |
| Workspace Tool         | colcon                | catkin_make/catkin_tools | colcon             |
| Message Generation     | Supports ROS 2 messages| Supports ROS 1 messages| Supports ROS 2 Python messages |

---

## 🤖 In a Robotics Context

| Scenario                        | ament_cmake Role                             |
|--------------------------------|----------------------------------------------|
| Building ROS 2 C++ packages     | Primary build system with ROS 2 extensions   |
| Generating message/service code | Automates generation of interfaces            |
| Running unit and integration tests | Supports CTest and GoogleTest frameworks   |
| Integrating multi-package workspaces | Works with `colcon` for scalable builds   |

---

## 🔧 Useful Commands (One-Liners)

- `colcon build --packages-select my_package` – Build specific ROS 2 package using ament_cmake  
- `colcon test --packages-select my_package` – Run tests for the package  
- `colcon build --symlink-install` – Build with symlinked installs for development  
- `ament_export_dependencies(<dependency_name>)` – Export dependencies in CMakeLists.txt  
- `ament_add_gtest(test_name test_file.cpp)` – Add a GoogleTest test target  

---

## 🔧 Compatible Items

- [[Colcon]] – ROS 2 build tool that orchestrates ament_cmake builds  
- [[ROS2 Messages]] – Interfaces generated and built by ament_cmake  
- [[ament_python]] – Complementary build system for Python-based ROS 2 packages  
- [[CMake]] – Underlying build system foundation  
- [[GoogleTest]] – Common C++ testing framework integrated with ament_cmake  

---

## 🔗 Related Concepts

- [[ROS2 Package]] (Built using ament_cmake or ament_python)  
- [[Colcon]] (Workspace build tool for ROS 2)  
- [[CMake]] (Cross-platform build system)  
- [[ament_python]] (Python build system for ROS 2)  
- [[ROS2 Messages]] (Interface files generated during build)  

---

## 📚 Further Reading

- [ament_cmake Documentation](https://index.ros.org/doc/ros2/Tutorials/Ament-CMake-Examples/)  
- [ROS 2 Build System Overview](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/)  
- [ROS 2 Developer Guide: Building Packages](https://docs.ros.org/en/humble/How-To-Guides/Colcon-Tutorial.html)  
- [CMake Official Site](https://cmake.org/)  
- [GoogleTest Documentation](https://github.com/google/googletest)  

---
