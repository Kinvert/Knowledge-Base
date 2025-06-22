# 🚗 KITTI Dataset

**KITTI** is one of the most widely used datasets for developing, training, and benchmarking computer vision algorithms in autonomous driving and robotics. Released by the Karlsruhe Institute of Technology (KIT) and the Toyota Technological Institute at Chicago (TTIC), KITTI provides real-world sensor data collected from a car driving in and around Karlsruhe, Germany.

---

## 🧠 Summary

- **Full name**: Karlsruhe Institute of Technology and Toyota Technological Institute dataset
- **Type**: Real-world driving dataset
- **Purpose**: Benchmarking for computer vision, SLAM, depth estimation, object detection, and autonomous driving tasks
- **Data collected**: Stereo cameras, LiDAR, GPS/IMU, odometry

---

## 📦 Main Features

- High-resolution stereo camera images
- 3D point clouds from Velodyne HDL-64E LiDAR
- GPS and IMU data for ground truth localization
- Calibrated and synchronized multi-sensor setup
- Diverse urban, rural, and highway driving scenarios

---

## 🔬 Common Use Cases

- [[SLAM]]
- [[Monocular SLAM]]
- [[Binocular SLAM]]
- [[Depth Estimation]]
- [[Object Detection]]
- Lane detection
- 3D object tracking
- Sensor fusion experiments

---

## 📊 Comparison to Other Datasets

| Dataset        | Environment      | Sensors                  | Size (approx.)   | Example Tasks                   |
|----------------|-----------------|--------------------------|-----------------|----------------------------------|
| KITTI          | Real-world roads  | Stereo, LiDAR, GPS/IMU    | ~180 GB (raw)    | SLAM, 3D detection, segmentation |
| nuScenes       | Urban             | LiDAR, radar, cameras     | ~1.4 TB          | 3D detection, tracking           |
| Waymo Open     | Urban, suburban   | LiDAR, cameras            | ~1.9 PB          | 3D detection, tracking           |
| ApolloScape     | Urban             | LiDAR, cameras            | ~1 TB            | Semantic segmentation            |

---

## ✅ Strengths

- High-quality calibration and synchronization between sensors
- Provides ground truth poses for odometry and SLAM
- Open and well-documented, with many existing benchmarks
- Suitable for stereo vision and LiDAR-based research

---

## ❌ Weaknesses

- Limited diversity compared to newer datasets (mostly Karlsruhe region)
- No radar data (unlike nuScenes or Waymo)
- Smaller dataset size compared to recent large-scale autonomous driving datasets

---

## 🌐 External References

- [Official KITTI website](http://www.cvlibs.net/datasets/kitti/)
- [KITTI benchmarks](http://www.cvlibs.net/datasets/kitti/eval_object.php)

---

## 🔗 Related Notes

- [[SLAM]]
- [[Depth Estimation]]
- [[LiDAR]]
- [[Sensor Fusion]]
- [[Autonomous Driving]]
- [[PCL]]
- [[Open3D]]

---
