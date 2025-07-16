# Foxglove Studio

**Foxglove Studio** is a modern visualization and debugging tool designed for robotics and embedded systems. It offers a polished UI for inspecting sensor data, plotting topics, replaying logs, and debugging systems built on top of middleware like [[ROS2]] and [[ROS]]. Foxglove Studio is cross-platform and web-based, with support for cloud workspaces, live streaming, and customizable panels.

This complements and extends what traditional tools like [[rqt]] and [[RViz]] offer, especially in modern distributed and remote workflows.

---

## 🧠 Overview

- Built with web technologies and runs as a desktop or web app  
- Supports ROS1 and ROS2 natively, plus custom protocols like Foxglove WebSocket  
- Ideal for teams working on multi-sensor systems with rich data (e.g., LiDAR, IMU, cameras)  
- Includes time-synchronized visualization and message inspection tools  
- Supports extensions via plugins and custom panels

---

## 🧪 Use Cases

- Visualizing robot logs in post-mortem debugging  
- Inspecting sensor alignment and time sync issues  
- Building dashboards for teams working on SLAM or autonomous navigation  
- Collaborating remotely with ROS bagfile playback and screen sharing  
- Prototyping visual interfaces for robot telemetry in simulation or on hardware

---

## 🔧 Key Features

- Native support for `rosbag2` and `rosbridge`  
- Timeline view for time-synced messages  
- 3D panel for point clouds and robot models (like [[RViz]])  
- Plugin architecture for custom panels or data types  
- Works with live streams and logs (e.g., `.mcap` support)  
- WebSocket protocol for custom integration

---

## 📊 Comparison with Related Tools

| Feature                  | Foxglove Studio   | [[RViz]]        | [[rqt]]        | [[Webviz]]     | [[PlotJuggler]] |
|--------------------------|-------------------|------------------|----------------|----------------|----------------|
| 3D Visualization         | ✅                 | ✅                | ❌              | ✅              | ❌              |
| Custom Panels            | ✅ (plugin system) | 🟡 (via RViz plugins) | ✅ (via plugins) | ❌              | ❌              |
| Web-Based                | ✅                 | ❌                | ❌              | ✅              | ❌              |
| ROS1 Support             | ✅                 | ✅                | ✅              | ✅              | ✅              |
| ROS2 Support             | ✅                 | ✅                | ✅              | 🟡 (limited)    | ✅              |
| Bagfile Playback         | ✅ (.mcap, .db3)   | 🟡 (only `.bag`)  | 🟡              | ✅              | ✅              |
| Multi-User Remote Debug  | ✅ (via cloud)     | ❌                | ❌              | 🟡              | ❌              |

---

## ✅ Pros

- Intuitive interface for debugging large systems  
- Works well in distributed development environments  
- Plugin system allows extensibility  
- Cloud and local workflows  
- Real-time or recorded data support

---

## ❌ Cons

- Heavy dependency on modern web stack (can be slower on older hardware)  
- Some advanced features require proprietary Foxglove platform use  
- Fewer advanced 3D visualization features compared to RViz in simulation use cases

---

## 🔗 Related Concepts

- [[Foxglove]]  
- [[RViz]]  
- [[rqt]]  
- [[rosbridge_suite]]  
- [[rosbag2]]  
- [[MCAP]]  
- [[Visualization Tools]]  
- [[Simulation Environments]]

---

## 📚 Further Reading

- [Foxglove Studio Website](https://foxglove.dev/studio)  
- [MCAP (Log Format)](https://mcap.dev)  
- [Foxglove GitHub](https://github.com/foxglove/studio)  

---
