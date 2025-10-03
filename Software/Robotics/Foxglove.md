# 🦊 Foxglove Studio

**Foxglove Studio** is a powerful open-source visualization and debugging platform built for robotics and autonomous systems. It serves as a modern alternative (or complement) to tools like [[RViz]] and [[rqt]] in the ROS ecosystem, supporting [[ROS2]] data formats, as well as other middleware and log-based data workflows.

---

## 🧠 What Is Foxglove Studio?

Foxglove Studio is a **desktop application** and web-based tool that allows roboticists to **visualize, inspect, and debug** real-time and recorded telemetry from robots, simulations, or logs. It provides a modular, highly extensible interface with customizable panels for viewing:

- Messages and topics (ROS, ROS 2, MCAP)
- 3D scenes
- Time-series plots
- Transforms (TFs)
- Images and camera feeds
- Diagnostics and health
- Logs and console output

---

## 📦 Key Features

- **Multi-protocol support**: ROS2, [[MCAP]] logs, custom WebSocket-based sources
- **Extensible plugins**: Add new visualization panels or data sources
- **Cross-platform**: Mac, Linux, Windows, browser
- **Cloud sync**: Optionally integrates with Foxglove Data Platform
- **Time synchronization**: Useful for debugging sensor fusion and perception issues
- **Live and offline modes**: View data in real-time or post-process log files
- **Multi-user collaboration**: Share layouts and diagnostics remotely

---

## 🧰 Common Use Cases

- Visualizing perception pipelines
- Debugging robot control logic
- Viewing camera and lidar outputs
- Monitoring system diagnostics
- Investigating edge case scenarios in simulation logs
- Logging and analyzing teleoperation sessions

---

## 🔗 Supported Formats & Integrations

- ✅ ROS1 & ROS2
- ✅ MCAP (log file format developed by Foxglove)
- ✅ WebSocket bridge (custom or integrated into simulators)
- ✅ [[Ignition Gazebo]] (via bridge or ROS2 integration)
- ✅ Custom integrations via extensions and plugins

---

## 📊 Comparison Table

| Tool              | Real-Time View | Log Playback | ROS1 | ROS2 | Extensible UI | Web-Based | Plugins |
|-------------------|----------------|--------------|------|------|----------------|-----------|---------|
| **Foxglove Studio** | ✅              | ✅            | ✅    | ✅    | ✅              | ✅         | ✅       |
| [[RViz]]           | ✅              | ❌            | ✅    | ✅    | ⚠️ (limited)     | ❌         | ❌       |
| [[rqt]]            | ✅              | ❌            | ✅    | ✅    | ✅              | ❌         | ✅       |
| Webviz             | ✅              | ✅            | ✅    | ⚠️ Partial | ⚠️              | ✅         | ⚠️       |

---

## ✅ Pros

- Modern UI and UX
- Multi-source/multi-user friendly
- Works well with log-based workflows (e.g. MCAP)
- Lightweight and fast
- Extensible and modular
- Works well across teams and platforms

## ❌ Cons

- Some features require setup of bridge or MCAP export
- Plugin ecosystem still growing
- Might be unfamiliar for users deeply tied to older ROS tools

---

## 🚀 Getting Started

1. Download from [foxglove.dev](https://foxglove.dev/)
2. Launch and connect to:
   - ROS master
   - ROS2 DDS discovery
   - WebSocket bridge
   - MCAP file
3. Add and customize panels:
   - 3D, Plot, Raw Messages, TF Tree, Logs, etc.
4. Share layout with team via `.foxglove.json` files

---

## 🧩 Ecosystem Components

- **Foxglove Studio**: The core app (desktop and browser)
- **Foxglove WebSocket Bridge**: To connect arbitrary robots or systems
- **Foxglove Extensions**: Create new panels or message interpreters
- **Foxglove Data Platform** (optional): Commercial cloud features for fleet management and collaboration

---

## 🔗 Internal Links

- [[RViz]]
- [[ROS2]]
- [[MCAP]]
- [[rqt]]
- [[Gazebo]]
- [[Ignition]]
- [[Simulation Tools]]
- [[Telemetry]]
- [[Robot Visualization Tools]]

---

## 🌍 External Links

- [Foxglove.dev (Official Site)](https://foxglove.dev/)
- [Foxglove GitHub](https://github.com/foxglove)
- [MCAP log format](https://mcap.dev)
- [Installing Foxglove Studio](https://foxglove.dev/download)

---
