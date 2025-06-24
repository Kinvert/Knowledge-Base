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

---

## 🔗 Related Notes

- [[SLAM]]
- [[Monocular SLAM]]
- [[Binocular SLAM]]
- [[PCL]]
- [[Feature Detection]]

---
