# 📸 Bundle Adjustment

**Bundle Adjustment (BA)** is a key optimization technique in computer vision and robotics, particularly used in **SLAM**, **Structure-from-Motion (SfM)**, and **3D reconstruction**. It refines the 3D coordinates of observed points (landmarks) and camera poses simultaneously to minimize the **reprojection error**—the difference between observed and predicted image points.

---

## 🧠 Summary

- Simultaneously optimizes **camera parameters** and **3D point positions**.
- Minimizes **reprojection error** across all views and points.
- Typically solved using **nonlinear least squares optimization**.
- Essential for high-accuracy 3D mapping and localization.

---

## ⚙️ Key Concepts

| Term                  | Description                                                                 |
|-----------------------|-----------------------------------------------------------------------------|
| **Camera Pose**       | The 6-DOF position and orientation of the camera in space.                  |
| **Landmark**          | A 3D point observed by one or more cameras.                                 |
| **Reprojection Error**| The 2D distance between observed feature locations and reprojected points.  |
| **Jacobian**          | Matrix of partial derivatives used in nonlinear optimization.               |

---

## 🔄 Process Overview

1. Detect and match features across images.
2. Estimate initial camera poses and 3D point locations.
3. Construct the optimization problem:
   - Variables: camera poses and 3D points.
   - Constraints: observed 2D image features.
4. Use an optimizer (e.g. [[Ceres]], [[g2o]], [[GTSAM]]) to minimize total reprojection error.
5. Refine and repeat if needed.

---

## 🧪 Common Libraries

| Library     | Language | Features                            | Notes                          |
|-------------|----------|-------------------------------------|--------------------------------|
| [[Ceres]]   | C++      | Auto-diff, robust loss functions     | Google’s flagship optimizer    |
| [[g2o]]     | C++      | Highly customizable, efficient       | Used in [[ORB-SLAM]]          |
| [[GTSAM]]   | C++      | Probabilistic modeling, factor graphs| Ideal for SLAM + sensor fusion |

---

## 🔧 Applications

- [[SLAM]] back-end optimization (refining map + trajectory).
- [[SfM]] (Structure from Motion) in 3D reconstruction pipelines.
- Loop closure corrections in pose graphs.
- Multi-view stereo and panoramic stitching.

---

## 🆚 Related Concepts

| Concept               | Relationship to Bundle Adjustment                              |
|------------------------|----------------------------------------------------------------|
| [[Pose Graph Optimization]] | Optimizes poses only, not 3D landmarks                        |
| [[Triangulation]]          | Provides initial 3D point guesses before BA                    |
| [[Feature Detection]]       | Supplies the keypoint observations used in BA                 |
| [[Epipolar Geometry]]      | Governs the geometric relations between 2D projections         |

---

## 🧩 Strengths

- Greatly improves accuracy in camera trajectory and 3D structure.
- Fully nonlinear optimization over all parameters.
- Can incorporate camera intrinsics, distortion, etc.

---

## ⚠️ Challenges

- Computationally intensive, especially for large datasets.
- Requires good initial guesses to converge well.
- Scalability limits in real-time SLAM systems (often approximated or delayed).

---

## 🔗 Related Notes

- [[SLAM]]
- [[g2o]]
- [[Ceres]]
- [[GTSAM]]
- [[Pose Graph Optimization]]
- [[Triangulation]]
- [[Visual Odometry]]

---

## 🌐 External References

- [Bundle Adjustment — Wikipedia](https://en.wikipedia.org/wiki/Bundle_adjustment)
- [Google Ceres Solver](http://ceres-solver.org/)
- [Multiple View Geometry in Computer Vision — Hartley & Zisserman]

---
