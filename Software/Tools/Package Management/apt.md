# apt

**apt** (Advanced Package Tool) is the standard package manager used by Debian-based Linux distributions, including Ubuntu. It simplifies the installation, upgrade, and removal of software packages and is widely used in robotics environments—especially with platforms like ROS, which rely on apt for binary installation.

---

## 📚 Overview

`apt` works with `.deb` packages and manages dependencies automatically. It pulls from online repositories or local mirrors to install system-level software. In robotics, apt is critical for setting up environments with ROS, Gazebo, OpenCV, and other dependencies. While higher-level than source-based tools like `pip` or `Conan`, it’s often the first step in preparing a system.

---

## 🧠 Core Concepts

- **Repositories**: Sources of packages (official, PPA, custom mirrors)
- **Sources List**: Configured in `/etc/apt/sources.list` or `/etc/apt/sources.list.d/`
- **Package Caching**: Packages are downloaded to `/var/cache/apt/archives`
- **GPG Keys**: Used to verify repository authenticity
- **Dependencies**: Automatically resolved during install/upgrade

---

## 🧰 Use Cases

- Install robotics middleware like ROS or DDS
- Set up simulation tools like Gazebo or Ignition
- Manage compilers, build tools, and libraries (gcc, cmake, boost)
- Install utilities for debugging, networking, and monitoring
- Automate OS provisioning in Docker containers or CI pipelines

---

## ✅ Pros

- Widely supported and stable
- System-wide installation of precompiled binaries
- Automatic handling of dependencies and security updates
- Works well with shell scripting and automation
- Large ecosystem of software for robotics and development

---

## ❌ Cons

- System-wide changes may affect unrelated packages
- Slower updates for newer versions than source builds
- Less flexibility than language-specific package managers (e.g., pip, npm)
- Installing conflicting versions often requires custom workarounds

---

## 📊 Comparison Chart

| Feature                  | apt               | pip               | conda             | npm               | snap              |
|--------------------------|-------------------|-------------------|-------------------|-------------------|-------------------|
| Language Support         | ✅ All (system)    | ❌ Python-only     | ✅ Multi-language  | ❌ JS-only         | ✅ System          |
| Source of Packages       | ✅ Binary Repos    | ✅ PyPI            | ✅ Conda Repos     | ✅ npm Registry     | ✅ Snap Store       |
| Version Flexibility      | ⚠️ Moderate         | ✅ High            | ✅ High            | ✅ High            | ⚠️ Fixed per snap  |
| Isolation                | ❌ System-wide     | ✅ w/ venv         | ✅ w/ envs         | ✅ per project     | ✅ per app         |
| Cross-Platform Support   | ⚠️ Debian-based only | ✅ All OS       | ✅ All OS          | ✅ All OS          | ✅ All OS          |

---

## 🤖 In a Robotics Context

| Task                              | Example apt Usage                              |
|-----------------------------------|-------------------------------------------------|
| Install ROS2                      | `sudo apt install ros-humble-desktop`          |
| Set up image tools                | `sudo apt install v4l-utils`                   |
| Add real-time kernel              | `sudo apt install linux-image-lowlatency`      |
| Install OpenCV                    | `sudo apt install libopencv-dev`               |
| Build from source prerequisites   | `sudo apt install build-essential`             |

---

## 🔧 Useful Commands (One-Liners)

- `sudo apt update` – Update package list  
- `sudo apt upgrade` – Upgrade installed packages  
- `sudo apt install cmake` – Install a package  
- `sudo apt remove ros-humble-*` – Remove a package  
- `apt list --installed` – View installed packages  
- `apt search opencv` – Search for a package  
- `sudo apt autoremove` – Remove unused dependencies  

---

## 🔧 Compatible Items

- [[CMake]] – Installed via apt for C++ builds  
- [[Docker]] – Often installs apt packages in base images  

---

## 🔗 Related Concepts

- [[dpkg]] (Low-level tool that apt wraps)  
- [[Dockerfile]] (Often uses `apt install` in image layers)  
- [[Conda]] (Higher-level, multi-language alternative)  
- [[Snap]] (Another system package manager for Ubuntu)

---

## 📚 Further Reading

- [apt Documentation](https://wiki.debian.org/Apt)
- [Ubuntu apt Guide](https://help.ubuntu.com/community/AptGet/Howto)
- [Package Search](https://packages.ubuntu.com/)
- [Adding Custom Repos](https://linuxize.com/post/how-to-add-apt-repository-in-ubuntu/)
- [Debian vs Snap vs Flatpak](https://www.omgubuntu.co.uk/2020/06/deb-vs-snap-vs-flatpak)

---
