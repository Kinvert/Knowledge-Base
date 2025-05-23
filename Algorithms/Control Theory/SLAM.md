---
title: SLAM (Simultaneous Localization and Mapping)
tags: [algorithms, robotics, mapping, localization, sensor-fusion, navigation]
aliases: [Simultaneous Localization and Mapping, SLAM Algorithm, SLAM Mapping]
---

# 🗺️ SLAM (Simultaneous Localization and Mapping)

## 🧭 Overview

**SLAM (Simultaneous Localization and Mapping)** is a computational problem and set of algorithms that enable a robot or autonomous system to build a map of an unknown environment while simultaneously determining its own position within that map. SLAM is foundational in robotics, autonomous vehicles, drones, and AR/VR, where GPS is unavailable or unreliable.

SLAM fuses data from various sensors (e.g., LiDAR, cameras, IMUs, wheel encoders) to estimate both the map and the robot’s pose in real time.

---

## 🛠️ Key Features

- **Real-Time Mapping**: Builds a map of the environment as the robot moves.
- **Localization**: Continuously estimates the robot’s position and orientation.
- **Sensor Fusion**: Integrates data from multiple sensors for robustness.
- **Loop Closure**: Detects when the robot revisits a location to correct accumulated errors.
- **Adaptable**: Works with 2D or 3D environments and various sensor types.

---

## 📦 Common Use Cases

- **Mobile Robotics**: Indoor navigation, warehouse robots, service robots.
- **Autonomous Vehicles**: Self-driving cars, drones, and underwater vehicles.
- **Augmented/Virtual Reality**: Headset tracking and spatial mapping.
- **Surveying and Mapping**: Generating maps of unknown or hazardous environments.

---

## 🧩 Types of SLAM

- **Visual SLAM (vSLAM)**: Uses cameras (monocular, stereo, RGB-D) as primary sensors.
- **LiDAR SLAM**: Uses laser scanners for high-precision mapping.
- **RGB-D SLAM**: Combines color and depth data from sensors like Kinect or RealSense.
- **Multi-Sensor SLAM**: Fuses IMU, wheel odometry, GPS, and other sensors for improved accuracy.

---

## 🆚 SLAM vs. Related Algorithms

| Algorithm         | Mapping | Localization | Sensor Fusion | Loop Closure | Typical Sensors      |
|-------------------|---------|--------------|---------------|--------------|---------------------|
| **SLAM**          | ✅ Yes  | ✅ Yes       | ✅ Yes        | ✅ Yes       | LiDAR, Camera, IMU  |
| **Odometry**      | ❌ No   | ✅ Yes       | ✅ Yes        | ❌ No        | Encoders, IMU       |
| **Mapping Only**  | ✅ Yes  | ❌ No        | ✅ Yes        | ❌ No        | LiDAR, Camera       |
| **Localization Only** | ❌ No | ✅ Yes      | ✅ Yes        | ❌ No        | GPS, IMU            |

---

## ✅ Pros and ❌ Cons

### ✅ Advantages
- **Enables Autonomy**: Essential for robots and vehicles in unknown environments.
- **Sensor Agnostic**: Works with a wide range of sensors.
- **Real-Time Operation**: Many SLAM algorithms run in real time.

### ❌ Disadvantages
- **Computationally Intensive**: Can require significant processing power.
- **Drift and Errors**: Accumulated errors without loop closure or good sensor fusion.
- **Complexity**: Implementation and tuning can be challenging.

---

## 🧰 Common SLAM Packages, Libraries, and Frameworks

### Python

- **RTAB-Map** (via ROS): Real-Time Appearance-Based Mapping, accessible through ROS Python bindings.
- **Open3D**: Provides SLAM examples and point cloud processing tools.
- **PySLAM**: Lightweight Python library for monocular visual SLAM.
- **ORB-SLAM2/3 (Python Bindings)**: Community projects provide Python wrappers for ORB-SLAM.

### C++

- **ORB-SLAM2 / ORB-SLAM3**: Widely used open-source visual SLAM systems for monocular, stereo, and RGB-D cameras.
- **RTAB-Map**: Real-time 3D SLAM library with LiDAR and visual support, ROS integration.
- **Google Cartographer**: 2D and 3D SLAM for LiDAR and other sensors, ROS compatible.
- **GTSAM**: Factor graph optimization library often used in SLAM backends.
- **LSD-SLAM**: Large-Scale Direct Monocular SLAM.

### ROS (Robot Operating System)

- **slam_gmapping**: 2D laser-based SLAM.
- **hector_slam**: 2D SLAM for systems without odometry.
- **cartographer_ros**: ROS integration for Google Cartographer.
- **rtabmap_ros**: ROS wrapper for RTAB-Map.

### MATLAB

- **MATLAB Robotics System Toolbox**: Built-in SLAM algorithms for simulation and prototyping.
- **VINS-Mono**: Visual-inertial SLAM, with MATLAB support for research.

### Other Languages/Frameworks

- **SLAM Toolbox (C++)**: Advanced 2D SLAM for ROS2.
- **Karto SLAM (C++)**: Efficient 2D laser-based SLAM.
- **MRPT (C++)**: Mobile Robot Programming Toolkit, includes SLAM modules.
- **Cartographer (Go)**: Experimental Go bindings for Google Cartographer.

**Note:** Most state-of-the-art SLAM libraries are written in C++ for performance, with Python or ROS bindings for ease of use and integration.

---

## 🔗 Related Topics

- [[State Estimation]]
- [[Kalman Filter]]
- [[Sensor Fusion]]
- [[IMU]]
- [[LiDAR]]
- [[Visual Odometry]]
- [[Robotics]]
- [[Point Cloud]]
- [[Path Planning]]

---

## 📚 Further Reading

- [Wikipedia: Simultaneous Localization and Mapping](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping)
- [Probabilistic Robotics (Book)](https://www.probabilistic-robotics.org/)
- [OpenSLAM.org](https://openslam.org/)
- [ORB-SLAM2 (Visual SLAM)](https://github.com/raulmur/ORB_SLAM2)
- [Google Cartographer (LiDAR SLAM)](https://github.com/cartographer-project/cartographer)
- [ROS SLAM Tutorials](https://wiki.ros.org/slam_gmapping)

---

## 🧠 Summary

SLAM is a cornerstone technology for autonomous systems, enabling them to navigate and map unknown environments using sensor fusion and real-time estimation. Its versatility and importance span robotics, vehicles, and AR/VR, though it remains a challenging and active area of research.
