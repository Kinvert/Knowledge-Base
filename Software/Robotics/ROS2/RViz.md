# 🟣 RViz

**RViz (ROS Visualization)** is a 3D visualization tool tightly integrated with [[ROS2]] (and ROS 1). It allows developers and researchers to visualize the state of a robot and its environment in real time. By subscribing to ROS topics, RViz can render data streams such as lasers, point clouds, cameras, maps, and robot models, making it a central debugging and demonstration tool in the ROS ecosystem.

---

## 🧠 Summary

- Dedicated 3D visualization tool for ROS/ROS2.
- Displays sensor data, coordinate transforms, 3D models, and robot state.
- Aids in debugging, development, and presenting robotic systems.

---

## ⚙️ Key Features

| Feature | Description |
|--------|-------------|
| **Visualization Markers** | Render points, lines, arrows, text, and shapes from topic data. |
| **PointCloud2 Viewer** | Display LIDAR or depth data from `sensor_msgs/PointCloud2`. |
| **Image Viewer** | Show camera feeds from `sensor_msgs/Image` or compressed formats. |
| **TF Display** | Visualize coordinate frames using the `tf2` transform system. |
| **Robot Model Viewer** | Load URDF or [[XACRO]] robot models with live joint updates. |
| **Interactive Markers** | Provide GUIs for interacting with robots in real time. |
| **Map Display** | Render occupancy grids (`nav_msgs/OccupancyGrid`) for SLAM/mapping. |
| **Path/Trajectory Display** | Visualize trajectories from navigation or motion planning. |

---

## 🚀 Use Cases

- Visualizing robot pose and sensor data in development.
- Debugging [[SLAM]] pipelines and navigation stacks.
- Testing motion planning with [[MoveIt]] or [[OMPL]].
- Creating operator GUIs for robot teleoperation.
- Demonstrations and educational presentations of robotic systems.

---

## 🧪 Common Inputs (ROS Topics)

- `tf`, `tf_static` – Frame transforms.
- `/camera/image_raw` – Raw camera images.
- `/scan` – 2D laser scans ([[sensor_msgs-LaserScan]]).
- `/map` – Occupancy grids.
- `/odom`, `/pose`, `/path` – Localization and trajectory data.
- `/joint_states` – Feedback from robot joints.
- `/marker`, `/marker_array` – Visualization cues.

---

## 🛠 Supported Formats

- Robot models: URDF, [[XACRO]].
- Meshes: STL, DAE, Collada.
- Images: `rgb8`, `bgr8`, `mono8`, etc.
- Extensions: Plugin support for custom visualization.

---

## 🏁 Tips and Tricks

- Always set a **Fixed Frame** (`map`, `odom`, or `base_link`) to avoid mismatches.
- Launch with `ros2 run rviz2 rviz2`.
- Save/load configurations via `.rviz` files.
- Combine with [[Foxglove]] for remote visualization and log playback.

---

## ⚠️ Limitations

- High computational cost for dense point clouds or large maps.
- No physics simulation (use [[Gazebo]] or [[Ignition]] for that).
- Limited remote use unless paired with external tools (Foxglove, VNC).

---

## 🔍 Comparison to Other Visualization & Graphics Tools

| Tool      | Primary Use | 3D Support | ROS1/ROS2 Integration | Physics Simulation | Notes |
|-----------|-------------|------------|-----------------------|-------------------|-------|
| **RViz** | ROS data visualization | ✅ | Full | ❌ | Core ROS debugging tool. |
| **Gazebo / Ignition** | Simulation + visualization | ✅ | Full | ✅ | Physics + visualization; complements RViz. |
| **Foxglove Studio** | Telemetry & visualization | ✅ | Strong ROS1/ROS2 support | ❌ | Focused on log playback and dashboards. |
| **PyGame** | 2D game/graphics | ❌ | None | ❌ | General graphics library, not ROS-oriented. |
| **RayLib** | 2D/3D graphics/game dev | ✅ | None | ❌ | Lightweight graphics library. |
| **Unity (ROS-TCP Connector)** | Game engine & robotics visualization | ✅ | Optional (via plugin) | ✅ | High-fidelity rendering + physics. |
| **Unreal Engine (AirSim)** | Robotics + simulation | ✅ | Partial (via bridges) | ✅ | Photorealistic simulation for drones/AV. |

---

## 🔗 Related Notes

- [[ROS2]]
- [[URDF]]
- [[XACRO]]
- [[Gazebo]]
- [[Foxglove]]
- [[MoveIt]]
- [[OMPL]]
- [[TF2]]
- [[SLAM]]

---

## 🌐 External References

- [RViz2 GitHub (ROS2)](https://github.com/ros2/rviz)
- [RViz Wiki (ROS 1)](http://wiki.ros.org/rviz)
- [ROS2 RViz Tutorials](https://docs.ros.org/en/rolling/Tutorials/Intermediate/RViz/Using-RViz.html)
