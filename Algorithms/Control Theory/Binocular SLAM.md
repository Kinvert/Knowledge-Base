# 👀 Binocular SLAM

**Binocular SLAM** (Simultaneous Localization and Mapping using Stereo Vision) is a SLAM technique that uses **two synchronized cameras** (a stereo camera rig) to infer both camera motion and a 3D map of the environment. The stereo configuration allows it to estimate **absolute depth** directly from disparity between the left and right images, providing an immediate advantage over monocular systems.

---

## 🧠 Overview

- **Input**: Two calibrated and synchronized image streams from stereo cameras.
- **Goal**: Simultaneously estimate the position of the camera (localization) and create a map of the environment (mapping).
- **Primary Advantage**: Direct depth estimation from stereo disparity—no need to rely on motion for triangulation.

---

## 🏗️ How It Works

1. **Stereo Calibration**: The intrinsic and extrinsic parameters of the two cameras are calibrated.
2. **Feature Detection**: Keypoints are extracted from both left and right frames.
3. **Stereo Matching**: Disparity is calculated from corresponding keypoints to determine depth.
4. **3D Point Generation**: Points are reconstructed in 3D directly from the disparity map.
5. **Tracking**: Motion is estimated by matching features across frames.
6. **Mapping**: A 3D map is created incrementally using the depth information and poses.
7. **Loop Closure and Optimization**: Reduces drift and refines the pose and map using global optimization.

---

## 🧬 Binocular vs Monocular SLAM

| Feature                      | Binocular SLAM                      | Monocular SLAM                     |
|-----------------------------|-------------------------------------|------------------------------------|
| **Sensor Input**            | Stereo camera pair                  | Single camera                      |
| **Depth Estimation**        | Direct from disparity               | Triangulated from motion           |
| **Scale**                   | Absolute scale                      | Relative (scale drift possible)    |
| **Initialization**          | Immediate and stable                | Requires movement and feature-rich scene |
| **Cost**                    | Higher due to dual cameras          | Lower                              |
| **Accuracy**                | Generally more accurate             | Depends heavily on motion quality  |
| **Environmental Robustness**| Better performance in static scenes | Sensitive to low motion/texture    |

---

## 🧪 Notable Binocular SLAM Systems

| System               | Description                                                    | Notes                                   |
|---------------------|----------------------------------------------------------------|-----------------------------------------|
| **Stereo ORB-SLAM2**| Feature-based SLAM extended for stereo inputs                  | Accurate, widely used, robust           |
| **RTAB-Map**        | Graph-based SLAM supporting stereo and RGB-D                   | Integrates well with [[ROS2]]           |
| **LSD-SLAM (stereo)**| Direct SLAM variant adapted for stereo vision                 | Less commonly used than monocular LSD   |
| **ElasticFusion**   | Real-time dense mapping system (works with RGB-D or stereo)    | Great for volumetric maps               |

---

## 🔍 Key Advantages

- Immediate access to **metric scale** and real depth
- More stable and robust tracking in feature-poor environments
- Less drift over time compared to monocular SLAM
- Better performance in static scenes or during slow motion

## ⚠️ Limitations

- Hardware complexity (synchronization, calibration)
- Higher cost and power usage
- Susceptible to stereo matching errors in low-texture areas or lighting inconsistencies
- Requires careful alignment of stereo cameras

---

## 🛠️ Use Cases

- Autonomous vehicles
- Robotics and drones
- Augmented and Virtual Reality (AR/VR)
- Industrial automation
- 3D scanning and reconstruction

---

## 🔗 Related Topics

- [[Monocular SLAM]]
- [[SLAM]]
- [[Sensor Fusion]]
- [[RGB-D Cameras]]
- [[Stereo Vision]]
- [[Point Cloud Notes]]
- [[Visual Odometry]]
- [[ROS2]]
- [[OpenCV]]
- [[RTAB-Map]]
- [[Depth Estimation]]

---

## 📚 Further Reading

- [ORB-SLAM2 Stereo GitHub](https://github.com/raulmur/ORB_SLAM2)
- [RTAB-Map Documentation](https://introlab.github.io/rtabmap/)
- [KITTI Benchmark](http://www.cvlibs.net/datasets/kitti/)

---
