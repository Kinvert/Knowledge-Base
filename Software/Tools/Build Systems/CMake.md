# CMake

**CMake** is a cross-platform open-source build system generator. It is widely used in robotics, embedded systems, and C++ development, including in ROS and ROS2. CMake manages the build process in a compiler-agnostic way, generating native build files (Makefiles, Ninja, Visual Studio projects, etc.) from a platform-independent script.

---

## 📚 Overview

CMake allows developers to define the build logic in `CMakeLists.txt` files, which describe how to build binaries, link libraries, include headers, manage dependencies, and package outputs. In ROS2, `ament_cmake` builds on top of CMake to integrate with the ROS build ecosystem.

---

## 🧠 Core Concepts

- **`CMakeLists.txt`**: Script file that describes how the code should be built
- **Targets**: Abstracted build outputs (e.g., executables or libraries)
- **Variables**: Control the behavior of the build process (`CMAKE_INSTALL_PREFIX`, `CMAKE_CXX_STANDARD`)
- **Modules**: Built-in or custom scripts for finding libraries and packages
- **Find Packages**: Uses `find_package()` to locate external dependencies
- **Build Generators**: Creates build files for Ninja, Unix Makefiles, Visual Studio, etc.

---

## 🧰 Use Cases

- Building robotics projects with C++
- Managing large-scale C++ codebases in modular fashion
- Cross-compilation for embedded targets
- Integrating third-party libraries (e.g., Eigen, OpenCV, Boost)
- Creating toolchains for firmware or real-time systems

---

## ✅ Pros

- Powerful and flexible for large projects
- Cross-platform support
- Easily integrates with tools like `colcon`, `ninja`, and `make`
- Strong ecosystem of community modules
- Extensively used in open-source robotics (e.g., ROS, PX4)

---

## ❌ Cons

- Syntax and behavior can be unintuitive for newcomers
- Requires manual tuning for complex dependency graphs
- Error messages can be cryptic
- Performance depends on the underlying generator

---

## 📊 Comparison Chart

| Feature                | CMake               | [[Make]]               | [[Bazel]]              | [[Meson]]              | [[SCons]]              |
|------------------------|---------------------|--------------------|--------------------|--------------------|--------------------|
| Cross-platform         | ✅ Yes              | ⚠️ Limited         | ✅ Yes             | ✅ Yes             | ✅ Yes             |
| Declarative Syntax     | ✅ Moderate         | ❌ Imperative      | ✅ Declarative     | ✅ Declarative     | ⚠️ Python-based     |
| IDE Integration        | ✅ Excellent        | ❌ Poor            | ⚠️ Basic           | ⚠️ Basic           | ⚠️ Basic           |
| Robotics Support       | ✅ Excellent (ROS2) | ⚠️ Indirect        | ⚠️ Rare            | ❌ Rare            | ⚠️ Rare            |

---

## 🤖 In a Robotics Context

| Use Case                             | CMake Application                                     |
|--------------------------------------|--------------------------------------------------------|
| Building a ROS2 node                 | Define in `CMakeLists.txt` with `ament_cmake`         |
| Linking to sensor libraries          | Use `find_package(OpenCV REQUIRED)`                   |
| Adding compiler flags for hardware  | Set with `target_compile_options()`                   |
| Exporting headers for reuse         | Use `install(TARGETS ...)` and `target_include_directories()` |
| Creating simulation plugin           | Define shared library target                          |

---

## 🔧 Useful Commands (One-Liners)

- `cmake .` – Generate native build system in current directory  
- `cmake -B build -S .` – Separate build and source folders  
- `cmake --build build` – Build using generated system  
- `cmake -DCMAKE_BUILD_TYPE=Release ..` – Specify build type  
- `cmake --install build` – Install targets to system or prefix  
- `cmake --help` – Show command-line help and options  

---

## 🔧 Compatible Items

- [[ROS2 Package]] – Uses `ament_cmake` which builds on top of CMake
- [[colcon]] – ROS2's build tool that generates CMake commands
- [[C++]] – Most CMake projects are C++
- [[Eigen]] – Common dependency in CMake-based projects
- [[OpenCV]] – Integrated via `find_package(OpenCV)`
- [[Dockerfile]] – Often runs `cmake` commands in build steps

---

## 🔗 Related Concepts

- [[ament_cmake]] (ROS2-specific extension of CMake)
- [[ROS2 Interface Definition]] (Built into packages via CMake)
- [[GitHub Actions]] / [[Jenkins]] (Use CMake for build jobs)
- [[colcon]] (Builds CMake packages in ROS2)
- [[ROS2 Launch Files]] (Often launch CMake-built nodes)

---

## 📚 Further Reading

- [CMake Official Documentation](https://cmake.org/documentation/)
- [CMake Best Practices](https://cliutils.gitlab.io/modern-cmake/)
- [ament_cmake Docs](https://docs.ros.org/en/rolling/How-To-Guides/Ament-CMake-Documentation.html)
- [ROS2 CMake Template](https://github.com/ros2/examples)

---
