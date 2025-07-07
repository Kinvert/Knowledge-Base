# 🟣 UBA (Unified Bundle Adjustment) in SLAM

**UBA**, or **Unified Bundle Adjustment**, is a concept in [[SLAM]] (Simultaneous Localization and Mapping) that refers to the comprehensive optimization of 3D points and camera poses in a consistent and unified framework. While not always called "UBA" explicitly in all literature, this term often implies a single optimization routine that accounts for both **local** and **global** constraints, potentially across **multiple sensor types** or **camera views**.

---

## 🧠 Summary

- Bundle Adjustment (BA) is a crucial step in SLAM for refining estimates of map points and poses.
- UBA refers to approaches that **combine local and global bundle adjustment** into one cohesive framework.
- It may also involve integrating **multi-camera**, **visual-inertial**, or **multi-session** data in a unified optimization step.
- UBA frameworks are especially valuable in **visual SLAM**, **multi-sensor fusion**, and **large-scale mapping** systems.

---

## 🔍 Variants and Usage

| Type                     | Description                                                                 |
|--------------------------|-----------------------------------------------------------------------------|
| Local BA                 | Optimizes a sliding window of recent keyframes and observations.            |
| Global BA                | Performs full-map optimization, often after loop closure.                   |
| Unified BA (UBA)         | Combines local/global BA logic into a single, scalable routine.             |
| Multi-camera UBA         | Performs BA across synchronized stereo or multi-camera rigs.                |
| Visual-Inertial UBA      | Fuses IMU data into BA to constrain orientation and velocity.               |
| Multi-session UBA        | Applies UBA across SLAM sessions or maps merged over time.                  |

---

## ✅ Benefits

- **Consistency**: Fewer switching points between different optimization modes.
- **Improved accuracy**: Unified constraints lead to globally consistent maps.
- **Sensor fusion**: Seamless integration of visual, inertial, and other modalities.
- **Scalability**: Handles large maps more efficiently by integrating information more holistically.

---

## ⚠️ Challenges

- **Computational Cost**: Global optimization is expensive; UBA often needs approximation or sparsity exploitation.
- **Memory Usage**: Large datasets require optimized data structures.
- **Real-time Use**: True UBA is rare in real-time applications; often approximated with hybrid approaches.

---

## 🧱 Related Concepts

- [[Bundle Adjustment]]
- [[Pose Graph Optimization]]
- [[Visual SLAM]]
- [[Visual-Inertial Odometry]]
- [[Loop Closure]]
- [[g2o]] / [[Ceres Solver]] — common libraries used for BA and optimization.

---

## 🌐 External References

- [Bundle Adjustment Overview - Wikipedia](https://en.wikipedia.org/wiki/Bundle_adjustment)
- [Ceres Solver](http://ceres-solver.org/)
- [g2o Optimization Framework](https://github.com/RainerKuemmerle/g2o)
- Papers on ORB-SLAM2, VINS-Fusion, etc., that apply global/local BA concepts.

---
