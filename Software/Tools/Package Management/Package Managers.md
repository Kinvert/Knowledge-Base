# Package Managers

**Package Managers** are tools used to automate the installation, upgrade, configuration, and removal of software packages. They are essential in managing dependencies and ensuring reproducible environments in robotics, embedded systems, cloud computing, AI/ML, and more. Different ecosystems—such as programming languages, operating systems, and container orchestration—use specialized package managers.

---

## 📚 Overview

Package managers simplify the process of handling third-party libraries, system dependencies, and tooling for engineers and developers. They allow consistent builds, dependency resolution, and version control. In robotics, where stacks often span multiple languages and platforms, understanding various package managers is crucial for building, deploying, and maintaining complex systems.

---

## 🧠 Core Concepts

- **Repositories**: Centralized or distributed servers where packages are stored.
- **Dependency Resolution**: Automatically identifies and installs required libraries.
- **Versioning**: Allows installation of specific versions of software.
- **Lock Files**: Capture exact versions to ensure deterministic builds.
- **Build Systems Integration**: Often tied to the build or deployment tools in a given ecosystem.

---
---

## 📦 Categorized List of Package Managers

### 🧪 Programming Languages

- **Python**
  - `pip` – Standard Python package manager
  - `conda` – Handles environments and packages (especially in ML/data science)
  - `poetry` – Dependency management and packaging
- **C++**
  - `vcpkg` – Cross-platform C++ package manager from Microsoft
  - `conan` – Dependency manager for C++ with binary support
- **JavaScript / TypeScript**
  - `npm` – Node.js package manager
  - `yarn` – Faster, deterministic alternative to npm
  - `pnpm` – Space-efficient package manager
- **Rust**
  - `cargo` – Built-in package manager and build system
- **Go**
  - `go modules` – Built-in dependency tracking and fetching
- **Java**
  - `Maven` – XML-based build and dependency manager
  - `Gradle` – Groovy/Kotlin-based build tool
- **Lua**
  - `luarocks` – Dependency manager for Lua packages
- **Julia**
  - `Pkg.jl` – Integrated package manager in Julia
- **R**
  - `install.packages()` – Base function in R for installing packages
  - `pacman` – Wrapper around multiple package systems

---

### 🖥️ Operating Systems

- **Debian/Ubuntu**
  - `apt` / `apt-get` – System-level package manager
  - `dpkg` – Lower-level tool for managing `.deb` packages
- **RedHat/CentOS/Fedora**
  - `dnf` – Modern replacement for `yum`
  - `yum` – Legacy manager
  - `rpm` – Low-level RedHat package manager
- **Arch Linux**
  - `pacman` – Handles both source and binary packages
- **macOS**
  - `brew` (Homebrew) – Popular open-source manager for macOS
- **Windows**
  - `choco` (Chocolatey) – CLI package manager for Windows
  - `winget` – Microsoft's native CLI package manager
  - `scoop` – Minimalist Windows package manager
- **Snap / Flatpak / AppImage** – OS-agnostic, sandboxed formats

---

### 🤖 Robotics & Embedded

- `rosdep` – ROS/ROS2 dependency installer
- `colcon` – ROS2 build system with dependency resolution
- `rosinstall` – Tool for managing ROS workspaces
- `nix` – Functional package manager used for reproducible builds
- `bitbake` – Used with Yocto Project for embedded Linux builds
- `opkg` – Lightweight package manager for embedded systems (e.g. OpenWRT)
- `west` – Zephyr RTOS meta-tool that includes dependency management

---

### 📦 Containers & Orchestration

- `docker` – Uses Dockerfiles and `docker pull` to manage container images
- `docker-compose` – Declarative container orchestration
- `Helm` – Package manager for Kubernetes (uses Helm Charts)
- `kustomize` – Kubernetes native configuration manager
- `snapcraft` – Creates Snap packages for Linux applications

---

### 🧠 AI / ML / Data Science

- `pip` + `virtualenv` or `conda` – Python environments and packages
- `mlflow` – Manages ML experiments, includes dependency packaging
- `dvc` – Data version control, includes pipeline and dependency tracking
- `poetry` – Common in MLOps for reproducibility

---

### 🧰 Build Tools with Package Manager Roles

- `bazel` – Google's build tool with package management aspects
- `meson` – Fast build system with Python scripting
- `cmake` + `FetchContent` or `ExternalProject` – Common in C++, includes dependency handling
- `make` – Basic build tool, not a full package manager, but often used with one

---
---

## 📊 Comparison Table

| Name           | Ecosystem       | Type              | Version Control | Offline Use | Dependency Resolution |
|----------------|------------------|-------------------|------------------|-------------|------------------------|
| `pip`          | Python            | Language-level     | ✅ Yes            | ⚠️ Limited  | ✅ Yes                |
| `conda`        | Python, C/C++     | Env + packages     | ✅ Yes            | ✅ Yes       | ✅ Yes                |
| `apt`          | Ubuntu/Debian     | OS-level           | ✅ Yes            | ✅ Yes       | ✅ Yes                |
| `npm`          | JavaScript        | Language-level     | ✅ Yes            | ⚠️ Limited  | ✅ Yes                |
| `cargo`        | Rust              | Language-level     | ✅ Yes            | ✅ Yes       | ✅ Yes                |
| `rosdep`       | ROS               | Robotics           | ⚠️ Partial        | ⚠️ Limited  | ✅ Yes                |
| `vcpkg`        | C++               | Language-level     | ✅ Yes            | ⚠️ Partial  | ✅ Yes                |
| `Helm`         | Kubernetes        | Cluster-level      | ✅ Yes            | ✅ Yes       | ✅ Yes                |

---

## 🤖 In a Robotics Context

| Role                    | Tools Used                                |
|-------------------------|-------------------------------------------|
| System packages         | `apt`, `dnf`, `pacman`                    |
| Python tooling          | `pip`, `conda`, `poetry`                 |
| ROS stack builds        | `rosdep`, `colcon`, `rosinstall`          |
| Simulation dependencies | `apt`, `brew`, `pip`, `snap`             |
| CI/CD packaging         | `docker`, `helm`, `pip`, `npm`, `vcpkg`  |

---

## 🔧 Compatible Items

- [[Helm Chart]], [[Docker Container]], [[CI-CD]]
- [[Microservices Architecture]], [[Kubernetes]], [[colcon]]
- [[vcpkg]], [[rosdep]], [[nix]], [[pip]]

---

## 🔗 Related Concepts

- [[CI-CD]] (Often install packages from here)
- [[Dockerfile]] (Uses package managers inside containers)
- [[colcon]] (ROS2 build tool + dependency handling)
- [[Helm Chart]] (Kubernetes package manager)
- [[Microservices Architecture]] (Service dependencies)

---

## 🛠 Developer Tools

- `apt search`, `apt install <pkg>`
- `pip install <package>` or `conda install <package>`
- `helm install`, `rosdep install`, `colcon build`
- `brew install`, `choco install`, `winget install`

---

## 📚 Further Reading

- [Linux Package Management Guide](https://wiki.debian.org/Teams/Apt)
- [Python Packaging User Guide](https://packaging.python.org/)
- [Helm Docs](https://helm.sh/docs/)
- [ROS2 Dependency Management](https://docs.ros.org/en/rolling/How-To-Guides/Dependency-Management.html)
- [Conda vs Pip](https://docs.conda.io/projects/conda/en/latest/user-guide/tasks/manage-pkgs.html)

---
