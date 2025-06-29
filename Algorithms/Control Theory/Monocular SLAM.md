# 🧭 Monocular SLAM

**Monocular SLAM (Simultaneous Localization and Mapping)** is the process of estimating the trajectory of a moving camera and building a 3D map of the environment using a single camera. It is a cost-effective and compact approach often used in robotics, augmented reality, autonomous drones, and mobile devices.

---

## 🧠 Overview

- **Input**: A stream of images from a single RGB camera.
- **Goal**: Estimate both the camera’s motion (localization) and the environment's structure (mapping).
- **Core Challenge**: Depth information is not directly available from a single frame, requiring triangulation across multiple frames and motion to infer depth.
- **Main Difference from Binocular SLAM**: Lacks direct depth data—relies heavily on motion parallax.

---

## 🏗️ How It Works

1. **Feature Detection**: Keypoints are extracted from frames (e.g., ORB, FAST, SIFT).
2. **Feature Matching**: Keypoints are tracked across frames.
3. **Motion Estimation**: From the keypoint matches, motion is estimated using epipolar geometry.
4. **Triangulation**: 3D points are inferred by observing keypoints from multiple camera poses.
5. **Map Building**: A sparse 3D map is constructed incrementally.
6. **Loop Closure**: Recognizes previously visited locations to correct drift.
7. **Optimization**: Bundle adjustment and pose graph optimization refine the trajectory and map.

---

## 🔁 Monocular vs Binocular SLAM

| Feature                      | Monocular SLAM                      | Binocular (Stereo) SLAM               |
|-----------------------------|-------------------------------------|---------------------------------------|
| **Sensor Type**             | Single camera                        | Two synchronized cameras              |
| **Depth Perception**        | Indirect via motion triangulation   | Direct via stereo disparity           |
| **Scale Recovery**          | Up to scale (absolute scale ambiguous) | Absolute scale available            |
| **Hardware Cost**           | Lower                                | Higher                                |
| **Complexity**              | Lower hardware, higher algorithmic  | Higher hardware, lower algorithmic    |
| **Sensitivity to Motion**   | Requires sufficient camera motion   | Less reliant on motion                |
| **Initialization**          | More sensitive and difficult        | Easier with baseline depth info       |
| **Use Cases**               | AR/VR, small drones, mobile phones  | Autonomous driving, robotics          |

---

## 🧪 Common Monocular SLAM Algorithms

| Algorithm           | Description                                                  | Notes                               |
|---------------------|--------------------------------------------------------------|--------------------------------------|
| **Mono ORB-SLAM**   | Feature-based SLAM using ORB features                        | Popular, robust, loop closure        |
| **LSD-SLAM**        | Direct SLAM using image intensities (not features)           | Good for texture-less environments   |
| **DSO (Direct Sparse Odometry)** | Direct, sparse pixel-based optimization         | Lightweight, accurate, real-time     |
| **PTAM (Parallel Tracking and Mapping)** | Early academic SLAM system            | Influential, separate tracking/mapping |
| **TUM MonoVO**      | Visual Odometry, not full SLAM (no map/loop closure)         | Benchmark dataset and methods        |

---

## 🎓 Key Concepts & Terms

- **Epipolar Geometry**: The geometric relationship between two views of the same scene.
- **Scale Drift**: Accumulated error due to scale ambiguity in monocular systems.
- **Loop Closure**: Detecting re-visited places to correct trajectory.
- **Bundle Adjustment**: Non-linear optimization to refine camera poses and landmarks.
- **Keyframe Selection**: Choosing representative frames for mapping.

---

## ✅ Strengths of Monocular SLAM

- Simple and inexpensive hardware
- Smaller footprint (ideal for phones, AR headsets, drones)
- Vast research and open-source ecosystem

## ❌ Weaknesses

- No direct depth sensing
- Scale ambiguity (cannot recover real-world distances without aid)
- More sensitive to camera motion and poor lighting
- Initialization more challenging
- Fragile in low-texture scenes

---

## 🔗 Related Topics

- [[Binocular SLAM]]
- [[Visual Odometry]]
- [[Point Cloud Notes]]
- [[Sensor Fusion]]
- [[LiDAR]]
- [[SLAM]]
- [[ROS2]]
- [[OpenCV]]
- [[ORB-SLAM2]]
- [[RTAB-Map]]

---

## 📚 Further Reading

- [ORB-SLAM2 GitHub](https://github.com/raulmur/ORB_SLAM2)
- [LSD-SLAM Paper](https://vision.in.tum.de/research/vslam/lsdslam)
- [DSO Paper](https://vision.in.tum.de/research/vslam/dso)
- https://github.com/UZ-SLAMLab/ORB_SLAM3
- https://github.com/Mechazo11/ros2_orb_slam3
- https://github.com/luigifreda/pyslam
- https://github.com/jagennath-hari/FusionSLAM-Unifying-Instant-NGP-for-Monocular-SLAM
- https://github.com/rayvburn/ORB-SLAM2_ROS
- https://github.com/jagennath-hari/FusionSLAM-Unifying-Instant-NGP-for-Monocular-SLAM/tree/main

---
