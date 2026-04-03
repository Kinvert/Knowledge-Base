# Conan

**Conan** is a decentralized, open-source C/C++ package manager that simplifies dependency management in complex C++ projects. It helps developers download, configure, and build libraries across different platforms and compilers—a critical capability in robotics and embedded systems, where toolchains and environments vary widely.

---

## 📚 Overview

Conan handles external libraries like Boost, OpenCV, Eigen, and many robotics-related dependencies. It integrates well with popular build systems like **CMake**, **Meson**, and **Make**, and supports binary caching and reproducible builds. With Conan, teams can version, share, and reuse dependencies in both embedded and cloud-native environments.

---

## 🧠 Core Concepts

- **Conanfile.py** or **conanfile.txt**: Declares dependencies and build requirements
- **Conan Center**: Central repository for thousands of C++ packages
- **Remotes**: Package sources (e.g., Artifactory, private repos)
- **Profiles**: Define compiler, architecture, and build options
- **Binary Packages**: Built artifacts stored in a remote or local cache

---

## 🧰 Use Cases

- Manage dependencies in C++ robotics middleware and tools
- Share libraries between teams across ROS or embedded projects
- Ensure consistent toolchains across machines or CI systems
- Avoid source-vs-binary mismatches in cross-compilation
- Simplify version pinning and upgrades of third-party libraries

---

## ✅ Pros

- Native support for C/C++ and build system integration
- Platform-agnostic: Linux, Windows, macOS, cross-compilation
- Fine-grained control over build flags, compiler versions, etc.
- Supports both public and private package hosting
- Works well with CI/CD systems and Docker

---

## ❌ Cons

- Learning curve for `conanfile.py` and profiles
- Less mature ecosystem compared to pip or npm
- Package availability can be hit or miss for bleeding-edge libs
- Ties you into the Conan build model unless carefully managed

---

## 📊 Comparison Chart

| Feature                   | Conan             | vcpkg             | CMake FetchContent | pip (for Python)    | Conda (C++)         |
|---------------------------|-------------------|-------------------|---------------------|----------------------|----------------------|
| Language Focus            | ✅ C/C++           | ✅ C/C++           | ✅ C/C++             | ❌ Python-only        | ✅ Multi-language     |
| Binary Caching            | ✅ Yes             | ✅ Yes             | ❌ Source only       | ✅ Yes (wheels)       | ✅ Yes                |
| Remote Repositories       | ✅ Configurable    | ⚠️ Less flexible   | ❌ None              | ✅ PyPI               | ✅ Conda Forge         |
| Cross-Compilation         | ✅ Strong support  | ⚠️ Limited         | ⚠️ Manual            | ❌ Not applicable     | ⚠️ Basic support       |
| Build System Integration  | ✅ CMake, Meson    | ✅ CMake           | ✅ CMake             | ❌                   | ⚠️ Limited (non-native) |

---

## 🤖 In a Robotics Context

| Scenario                                 | Conan Utility                                      |
|------------------------------------------|----------------------------------------------------|
| Using Eigen, Boost, OpenCV in C++        | Manages consistent versions across platforms       |
| Embedded robotics build toolchains       | Profile-based cross-compilation setups             |
| Dockerized ROS2 builds                   | Package dependencies during Docker image builds    |
| Shared libraries for robot firmware      | Push/pull to/from private Conan remotes            |
| CI/CD C++ pipelines                      | Install and cache dependencies with Conan          |

---

## 🔧 Useful Commands (One-Liners)

- `conan install . --build=missing` – Install dependencies and build missing ones  
- `conan create .` – Package and register a Conan library  
- `conan upload mylib/1.0@myuser/stable --all -r=myremote` – Upload package to a remote  
- `conan remote add myremote https://myrepo.com/artifactory/api/conan/conan-local` – Add a remote  
- `conan profile show default` – View active build profile  
- `conan search` – List packages in the local cache  

---

## 🔧 Compatible Items

- [[C++]] – Primary language Conan supports  
- [[CMake]] – Tight integration via `conan.cmake` helper  
- [[Dockerfile]] – Conan used to manage dependencies inside build images  
- [[CI-CD]] – Cache and share built C++ packages  
- [[ROS2 Package]] – For C++-based packages with external dependencies  
- [[vcpkg]] – Alternative C++ package manager worth comparing  

---

## 🔗 Related Concepts

- [[CMake]] (Conan integrates directly)  
- [[ROS2 Package]] (Conan manages dependencies for C++ nodes)  
- [[Docker]] (Conan used in containerized builds)  
- [[CI-CD]] (Automated Conan install/build/test)  
- [[vcpkg]] (Alternative with Microsoft support)  
- [[pkg-config]] (Lower-level tool often superseded by Conan in modern builds)  

---

## 📚 Further Reading

- [Conan Official Website](https://conan.io/)
- [Getting Started with Conan](https://docs.conan.io/en/latest/getting_started.html)
- [Conan + CMake Best Practices](https://docs.conan.io/en/latest/integrations/cmake/cmake_paths.html)
- [Conan Center](https://conan.io/center/)
- [Conan vs vcpkg Comparison](https://github.com/conan-io/conan-vs-vcpkg)

---
