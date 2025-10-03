# 🤖 Gazebo Simulator

Gazebo is a powerful open-source 3D robotics simulator that enables testing, development, and validation of robotic systems in virtual environments. It provides accurate physics simulation, sensor modeling, and interfaces with robotic frameworks like [[ROS2]].

---

## 🧭 Overview

| Feature | Details |
|--------|---------|
| Name | Gazebo |
| Developer | Open Source Robotics Foundation (OSRF) |
| License | Apache 2.0 |
| Language | C++ (core), Python (bindings and tools) |
| Platforms | Linux (primary), macOS (partial), Windows (experimental) |
| ROS Integration | Deep integration with [[ROS2]] |
| Rendering | OpenGL |
| Physics Engines | ODE, Bullet, Simbody, DART |
| Simulation | Real-time and accelerated |

---

## 🧰 What Gazebo Offers

- **Physics Simulation**: Gravity, inertia, collisions, joint constraints
- **Sensor Models**: Cameras, IMUs, GPS, LiDARs, depth sensors
- **Robot Models**: Uses URDF / SDF formats to represent robots
- **Environment Design**: Buildings, terrain, lighting, objects
- **Plugins**: Extend functionality using C++ or Python
- **Visualization**: 3D visual feedback with GUI
- **Multi-Robot**: Simulate swarms or collaborative robotic systems
- **Headless Mode**: CLI-only execution for batch simulations

---

## 🔄 Gazebo vs Ignition Gazebo (Fortress, Citadel, etc.)

Gazebo classic has been rebranded as **Gazebo Classic**, while a modern reimplementation known as **Ignition Gazebo** (now simply referred to as Gazebo under the new architecture) is the future-forward version with modular, scalable architecture.

| Feature | Gazebo Classic | Ignition Gazebo |
|--------|----------------|-----------------|
| Architecture | Monolithic | Modular (library-based) |
| GUI | Integrated | Separate (`ign-gui`) |
| Communication | Custom | DDS / ZeroMQ |
| SDF Format | V1.6 | V1.9+ |
| Performance | Good | Improved (esp. with GPU support) |
| ROS Integration | ros_gazebo / gazebo_ros_pkgs | ros_ign_bridge (still maturing) |

---

## 📐 Supported Formats

- **URDF** (Unified Robot Description Format): Primary for ROS/ROS2
- **SDF** (Simulation Description Format): Native format for Gazebo

---

## 🧪 Use Cases

- Testing perception algorithms
- Validating SLAM and localization systems
- Simulating robot arms, drones, autonomous cars, rovers
- Education and prototyping
- Collaborative swarm simulation

---

## 📦 Notable Packages and Integrations

| Name | Function |
|------|----------|
| `gazebo_ros_pkgs` | ROS integration plugins and tools |
| `ign_gazebo` | Modular Ignition-based version |
| `ros_ign` | Bridge between ROS and Ignition Gazebo |
| `turtlebot3_gazebo` | Gazebo environment for Turtlebot3 |
| `fetch_gazebo` | For simulating Fetch robots |

---

## 🧠 Commonly Simulated Robots

- [[TurtleBot]]
- PR2
- Jackal
- Fetch
- Drone simulations (PX4 SITL with Gazebo)
- Custom robots via URDF/SDF

---

## ✅ Strengths

- Accurate, real-time physics
- Large community and ROS/ROS 2 support
- Broad sensor support
- Highly extensible with plugins
- Good for both research and education
- Realistic rendering (lighting, shadows, etc.)

---

## ❌ Weaknesses

- Can be CPU/GPU intensive
- Steep learning curve for full customization
- Integration with ROS 2 still maturing (especially Ignition bridge)
- Occasional bugs in physics engines
- Limited Windows support

---

## 🔍 Related Tools

- [[Webots]] – Lightweight alternative, good for education
- [[V-REP / CoppeliaSim]] – Graphical programming focus
- [[Unity ML-Agents]] – Machine learning simulation using Unity
- [[CARLA Simulator]] – Focused on autonomous driving
- [[RViz]] – ROS visualization, not physics simulation

---

## 🔗 Internal Links

- [[ROS2]]
- [[URDF]]
- [[SDF]]
- [[Ignition Gazebo]]
- [[Simulation Tools]]
- [[Physics Engines]]
- [[Robotics Middleware]]
- [[TurtleBot3]]

---

## 📚 References

- https://gazebosim.org
- https://github.com/gazebosim
- https://classic.gazebosim.org
- https://ignitionrobotics.org
