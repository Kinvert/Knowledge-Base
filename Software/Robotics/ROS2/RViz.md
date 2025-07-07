# 🟣 RViz

**RViz (ROS Visualization)** is a 3D visualization tool that is part of the [[ROS2]] (and ROS 1) ecosystem. It is used to visualize the state of a robot and its environment in real time. Developers and researchers use RViz to debug perception, navigation, SLAM, and motion planning components by displaying data from ROS topics such as lasers, point clouds, cameras, maps, and robot models.

---

## 🧠 Summary

- 3D visualization environment for ROS data streams.
- Displays sensor data, transforms, 3D models, navigation goals, and more.
- Useful for development, debugging, and demonstrations.

---

## ⚙️ Key Features

| Feature | Description |
|--------|-------------|
| **Visualization Markers** | Show points, lines, arrows, text, shapes from topic data. |
| **PointCloud2 Viewer** | Render LIDAR or depth camera data from `sensor_msgs/PointCloud2`. |
| **Image Viewer** | Display camera streams from `sensor_msgs/Image` or `CompressedImage`. |
| **TF Display** | Show real-time transforms using the `tf2` system. |
| **Robot Model Viewer** | Load URDF or [[XACRO]] robot models with dynamic joint state updates. |
| **Interactive Markers** | Add clickable GUI elements for manipulating objects or robot arms. |
| **Map Display** | Show occupancy grids (`nav_msgs/OccupancyGrid`) for SLAM or mapping. |
| **Path/Trajectory Display** | Visualize planned paths and executed trajectories from planners. |

---

## 🚀 Use Cases

- Viewing real-time camera feeds, LIDAR scans, and IMU data.
- Debugging [[SLAM]] and [[Navigation]] pipelines.
- Visualizing robot state in 3D with links, joints, and sensor outputs.
- Testing motion planners like [[MoveIt]] or [[OMPL]].
- Interacting with a robot via interactive markers.

---

## 🧪 Common Inputs (ROS Topics)

- `tf`, `tf_static` – Transforms between coordinate frames.
- `/camera/image_raw` – From [[sensor_msgs-Image]].
- `/scan` – From 2D laser scanners ([[sensor_msgs-LaserScan]]).
- `/map` – Occupancy grid maps.
- `/odom`, `/pose`, `/path` – Robot pose and trajectories.
- `/joint_states` – Joint position feedback.
- `/marker`, `/marker_array` – Visual cues for debugging.

---

## 🛠 Supported Formats

- URDF and [[XACRO]] robot models.
- Image encodings: `rgb8`, `bgr8`, `mono8`, etc.
- 3D formats: STL, DAE, Collada (via robot_description param).
- Supports plugins and extensions.

---

## 🏁 Tips and Tricks

- Use fixed frame (often `map`, `odom`, or `base_link`) to ensure consistent visualization.
- Launch RViz with `ros2 run rviz2 rviz2`.
- Load config files (.rviz) to retain layout and settings.
- Integrate with [[Foxglove]] for enhanced remote visualization and logging.

---

## ⚠️ Limitations

- Can be heavy on system resources (especially for dense point clouds).
- Doesn’t support simulation (use [[Gazebo]] or [[Ignition]] for physics).
- Limited remote access without extra tools (e.g., VNC, Foxglove).

---

## 🔗 Related Notes

- [[sensor_msgs]]
- [[sensor_msgs-Image]]
- [[sensor_msgs-PointCloud2]]
- [[cv_bridge]]
- [[Foxglove]]
- [[URDF]]
- [[XACRO]]
- [[Gazebo]]
- [[MoveIt]]
- [[TF2]]
- [[rosbag2]]

---

## 🌐 External References

- [RViz2 ROS 2 Docs](https://github.com/ros2/rviz)
- [RViz Wiki (ROS 1)](http://wiki.ros.org/rviz)
- [RViz Tutorials for ROS2](https://docs.ros.org/en/rolling/Tutorials/Intermediate/RViz/Using-RViz.html)

---
