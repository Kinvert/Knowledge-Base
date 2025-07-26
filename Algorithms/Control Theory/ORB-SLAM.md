# 🔷 ORB-SLAM

**ORB-SLAM** (Oriented FAST and Rotated BRIEF SLAM) is a feature-based Simultaneous Localization and Mapping (SLAM) system. It is widely used in robotics and computer vision for real-time tracking and mapping using monocular, stereo, or RGB-D cameras.

---

## 🧠 Summary

- Developed initially by Raúl Mur-Artal, José María M. Montiel, and Juan D. Tardós.
- Uses ORB (Oriented FAST + Rotated BRIEF) features for efficient and robust performance.
- Works in indoor and outdoor environments.
- Supports monocular, stereo, and RGB-D camera configurations.
- Open-source under GPL license.

---

## ⚙️ Key Components

- **Tracking**: Estimates the camera pose frame-by-frame using ORB feature matching.
- **Local Mapping**: Builds and optimizes a local map with keyframes and landmarks.
- **Loop Closing**: Detects when the system revisits a previously mapped area and performs loop closure to correct accumulated drift.
- **Relocalization**: Quickly recovers from tracking failure using a bag-of-words place recognition system.

---

## 🏆 Strengths

- Real-time performance on standard CPUs.
- Robust in various environments with no need for external infrastructure.
- Effective relocalization and loop closure.
- Scales well to large environments.

---

## ⚠️ Weaknesses

- Performance degrades in feature-poor or highly dynamic environments.
- Monocular mode suffers from scale ambiguity.
- Sensitive to motion blur and low-texture scenes.

---

## 📊 ORB-SLAM Version Comparison

| Feature / Capability       | ORB-SLAM1                 | ORB-SLAM2                        | ORB-SLAM3                                 |
|----------------------------|----------------------------|----------------------------------|-------------------------------------------|
| 🕵️ Feature Type            | ORB                        | ORB                              | ORB (with better tracking integration)     |
| 📷 Supported Sensors       | Monocular                  | Monocular, Stereo, RGB-D         | Monocular, Stereo, RGB-D, IMU             |
| 🧠 Sensor Fusion           | None                       | Limited (RGB-D depth fusion)     | Tight IMU integration (Visual-Inertial)   |
| 🔄 Loop Closure            | Yes                        | Yes                              | Yes (more robust and modular)             |
| 🌍 Map Reuse               | Yes                        | Yes                              | Yes                                       |
| ⛓️ Multi-Map Support       | No                         | No                               | Yes (multi-session/multi-map)             |
| 🛠️ Initialization          | Monocular only             | Improved for RGB-D and Stereo    | Unified initialization across modes       |
| ⚙️ Backend Optimization     | g2o                        | g2o                              | g2o + tightly integrated IMU preintegration |
| 🎯 Visual-Inertial (VIO)   | No                         | No                               | Yes                                       |
| ⌛ Real-Time Performance    | Yes                        | Yes                              | Yes (but more complex and heavier)        |
| 💾 Dataset Support         | TUM, KITTI, EuRoC (monocular only) | TUM, KITTI, EuRoC            | TUM, KITTI, EuRoC (full)                  |
| 🧩 Modularity              | Monolithic                 | Semi-modular                     | Fully modular (tracking, mapping, etc.)   |
| 🔬 SLAM Mode Support       | Only tracking              | SLAM + basic VIO from RGB-D      | SLAM + full VIO                           |
| 📅 Release Year            | 2015                       | 2017                             | 2020                                      |
| 📎 Paper                   | ORB-SLAM                   | ORB-SLAM2                        | ORB-SLAM3                                 |

The **original ORB-SLAM (ORB-SLAM1)** introduced a powerful SLAM system using only a **monocular camera**, relying on ORB features for tracking, loop closure, and map management. While groundbreaking for its time, it was limited to monocular setups and lacked integration with other sensors or support for dense reconstruction.

**ORB-SLAM2** significantly improved upon this by adding support for **stereo and RGB-D cameras**, allowing for better scale estimation and robustness in indoor environments. It maintained the core architecture from the first version but added flexibility in sensor input, making it more practical for real-world applications in robotics and AR/VR. However, it still lacked tight integration with IMUs or other sensor fusion techniques.

**ORB-SLAM3** is a major evolution. It introduces **full visual-inertial SLAM (VIO)** with tightly coupled IMU integration, making it far more robust in situations with fast motion or temporary visual occlusions. It also introduces **multi-map support**, more modular components, and unified initialization across sensor types. ORB-SLAM3 can handle **monocular, stereo, RGB-D, and VIO configurations** with a unified backend, making it one of the most versatile open-source SLAM systems available to date.

---

## 🔄 Comparison to Similar Systems

| Feature                | ORB-SLAM          | [[LSD-SLAM]]       | [[DSO]] (Direct Sparse Odometry) | [[RTAB-Map]]            |
|------------------------|------------------|-------------------|----------------------------------|------------------------|
| Type                   | Feature-based     | Direct (semi-dense) | Direct (sparse)                  | Feature + Graph-based   |
| Camera types supported  | Mono, Stereo, RGB-D | Mono               | Mono                             | RGB-D, Stereo, LIDAR    |
| Loop closure            | ✅ Yes            | ⚠️ Limited          | ❌ No                            | ✅ Yes                   |
| Real-time CPU           | ✅ Yes            | ✅ Yes              | ✅ Yes                           | ✅ Yes                   |
| Relocalization          | ✅ Yes            | ⚠️ Limited          | ⚠️ Limited                       | ✅ Yes                   |

---

## 🚀 Use Cases

- Autonomous drones and ground robots
- AR/VR headset tracking
- 3D reconstruction
- Visual-inertial odometry (in extensions of ORB-SLAM)

---

## 🌐 External References

- [ORB-SLAM GitHub](https://github.com/raulmur/ORB_SLAM2)
- [Original Paper](https://arxiv.org/abs/1502.00956)
- [ORB-SLAM3 (extended version)](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- https://github.com/arthurfenderbucker/indoor_drone
- https://www.youtube.com/watch?v=DxqzwBQVCNw

---

## 🔗 Related Notes

- [[SLAM]]
- [[Monocular SLAM]]
- [[Binocular SLAM]]
- [[PCL]]
- [[Feature Detection]]
- [[Visual-Inertial Odometry]]
- [[g2o]]
- [[TUM Dataset]]
- [[KITTI Dataset]]
- [[EuRoC Dataset]]

---
