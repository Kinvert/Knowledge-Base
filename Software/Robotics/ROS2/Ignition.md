# 🚀 Ignition Gazebo (Gazebo Sim)

**Ignition Gazebo**, now rebranded under the unified **Gazebo Sim** umbrella (https://gazebosim.org), is the next-generation simulation platform developed by Open Robotics to succeed the original [[Gazebo]] (now referred to as *Gazebo Classic*). It introduces a modular, scalable, and modern architecture built to handle advanced robotic simulation with enhanced flexibility, better performance, and long-term maintainability.

---

## 📜 Overview

| Feature               | Details |
|-----------------------|---------|
| Official Name         | Ignition Gazebo (Gazebo Sim) |
| Developer             | Open Robotics / OSRF |
| Written In            | C++ with plugin support |
| License               | Apache 2.0 |
| Platforms             | Linux (Ubuntu primary), macOS (experimental), Windows (partial) |
| Release Series        | Citadel, Edifice, Fortress, Garden, Harmonic (most recent) |
| Default Physics       | DART, ODE, Bullet, Simbody |
| Rendering             | OGRE 2 |
| Interprocess Comms    | ZeroMQ, DDS (Data Distribution Service) |
| Native Format         | [[SDF]] (Simulation Description Format) |

---

## 🧱 Key Features

- 🔌 **Modular Architecture**: Each component (GUI, physics, rendering, sensors, communication) is a separate library.
- 🧠 **Plugin-based**: Highly extensible with dynamic system plugins and sensor/actuator modules.
- 🌐 **Modern Communication**: Uses Ignition Transport or DDS middleware.
- 🎮 **Improved GUI**: Fully modular UI built with Qt.
- 🏎️ **Better Performance**: Optimized multithreading, GPU support.
- 🧪 **Testable Systems**: Ideal for CI/CD workflows in robotic software.
- 🌍 **Multi-Robot Support**: Designed for complex, large-scale simulations.

---

## 🆚 Ignition Gazebo vs Gazebo Classic

| Feature              | Gazebo Classic        | Ignition Gazebo (Gazebo Sim) |
|----------------------|------------------------|-------------------------------|
| Architecture         | Monolithic             | Modular (library-based)       |
| GUI Framework        | Integrated             | Qt-based, plugin-driven       |
| Rendering Engine     | OGRE 1.x               | OGRE 2.x                      |
| Physics Abstraction  | Basic                  | Clean abstraction + multiple backends |
| Communication        | Custom Pub/Sub         | DDS / ZeroMQ                  |
| ROS Integration      | `gazebo_ros_pkgs`      | `ros_ign_bridge`              |
| Extensibility        | Harder to manage       | Strong plugin system          |
| Performance          | Good                   | Improved (esp. on large scenes) |
| Actively Maintained  | Legacy Support Only    | Actively Maintained & Improved |

---

## 🛠️ Major Components

| Component             | Description |
|----------------------|-------------|
| `ign-gazebo`         | The simulation engine |
| `ign-fuel-tools`     | Asset fetching from online models |
| `ign-transport`      | Inter-process communication |
| `ign-gui`            | Modular GUI for simulation control |
| `ign-physics`        | Interface for multiple physics engines |
| `ign-common`         | Shared math/utilities |
| `ign-rendering`      | Handles 3D scene rendering |
| `ign-sensors`        | Camera, IMU, LiDAR, etc. |
| `ign-msgs`           | Message definitions |
| `ign-tools`          | Utilities like `ign topic`, `ign service` |

---

## 📚 Use Cases

- High-fidelity simulation for [[Autonomous Vehicles]]
- Multi-robot interaction scenarios
- Drone flight simulations
- Robotics education and research
- Sensor testing: depth cameras, IMUs, LiDAR
- Embedded systems simulation before hardware deployment

---

## 🔧 Supported Physics Engines

- ODE (Open Dynamics Engine)
- Bullet
- DART
- Simbody
- NVIDIA Isaac PhysX (experimental)

---

## 🖥️ Compatible OS and Languages

- **Primary OS**: Ubuntu 20.04 / 22.04
- **Languages**: C++, Python (via `pybind11`), some ROS/ROS2 integrations

---

## 🌉 ROS2 Integration

- **Bridge**: `ros_ign_bridge`
- ROS topics/services/actions can be bridged to Ignition’s communication system.
- Deep compatibility with [[ROS2]] middleware (DDS).

---

## 🔌 Typical Workflow

1. Define robot/environment in URDF or SDF.
2. Load into `ign-gazebo`.
3. Visualize and control via `ign-gui`.
4. Interact through plugins, sensors, topics.
5. Integrate with [[ROS2]] for control logic.

---

## 🛠️ Installation

- Via APT for Ubuntu (e.g. `sudo apt install ros-humble-ros-ign`)
- Source build options available via colcon or classic CMake
- Docker images provided by OSRF

---

## 🔍 Comparison Table

| Simulator         | Ignition Gazebo | [[Gazebo]] Classic | [[Webots]] | [[CoppeliaSim]] |
|------------------|------------------|---------------------|------------|------------------|
| Architecture     | Modular          | Monolithic          | Semi-modular | Monolithic |
| Language         | C++              | C++                 | C++ / Python | Lua / C++ |
| ROS2 Integration | ✅ Native via bridge | ✅ Legacy support | Partial     | Partial |
| Sensors          | ✅ Extensive      | ✅                  | ✅          | ✅          |
| Physics Engines  | 5+ options        | ODE, Bullet, etc.   | ODE        | Bullet     |
| GUI              | Qt-based         | Integrated          | Integrated | Integrated |
| Licensing        | Apache 2.0       | Apache 2.0          | Apache 2.0 | Proprietary |

---

## ✅ Strengths

- Modular architecture for custom setups
- Multi-robot capable and scalable
- Modern tooling and rendering
- ROS2-friendly
- Open source with active development

---

## ❌ Weaknesses

- Still evolving (plugin APIs change)
- Documentation lags in some areas
- Less intuitive GUI vs simpler systems like [[Webots]]
- Source builds can be complex

---

## 🔗 Internal Links

- [[Gazebo]]
- [[ROS2]]
- [[Simulation Tools]]
- [[SDF]]
- [[URDF]]
- [[CARLA Simulator]]
- [[Webots]]
- [[CoppeliaSim]]
- [[Physics Engines]]

---

## 📎 References

- https://gazebosim.org/
- https://github.com/gazebosim
- https://github.com/ignitionrobotics
- https://classic.gazebosim.org/
- https://github.com/ros-simulation/ros_ign
