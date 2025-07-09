# Epipolar Geometry

Epipolar Geometry describes the intrinsic projective geometry between two views of a 3D scene. It is a fundamental concept in stereo vision and structure-from-motion, used to constrain and match corresponding points between two camera images. In robotics and computer vision, it enables depth estimation, camera pose estimation, and 3D reconstruction.

---

## 📚 Overview

When a 3D point is observed by two different cameras, its projections lie along corresponding epipolar lines. The relationship between these lines is captured by the **essential matrix** (when intrinsic parameters are known) or the **fundamental matrix** (when they are not). These matrices encode the relative pose (rotation and translation) between the two views.

---

## 🧠 Core Concepts

- **Epipole**: The projection of one camera center onto the other camera's image plane.
- **Epipolar Line**: The line on the second image where the corresponding point must lie.
- **Fundamental Matrix (F)**: Encodes the epipolar geometry without knowing camera intrinsics.
- **Essential Matrix (E)**: Encodes epipolar geometry assuming known camera intrinsics.
- **Triangulation**: Estimating 3D point position from two projections.

---

## 🧰 Use Cases

- Stereo vision and depth estimation
- Visual Odometry (VO)
- Structure-from-Motion (SfM)
- SLAM initialization
- Matching features across views
- Outlier rejection using epipolar constraint

---

## ✅ Pros

- Constrains feature matching to 1D lines, improving robustness
- Enables depth estimation from stereo pairs
- Key part of multi-view geometry and 3D reconstruction

---

## ❌ Cons

- Sensitive to noise and outliers in point correspondences
- Assumes rigid scenes with static cameras
- Fails with degenerate configurations (e.g., planar scenes)

---

## 📊 Comparison Chart

| Concept             | Requires Intrinsics? | Used For               | Matrix Type | Notes |
|---------------------|----------------------|------------------------|-------------|-------|
| **Fundamental Matrix (F)** | ❌ No               | Epipolar geometry      | 3x3         | Computed from point correspondences |
| **Essential Matrix (E)**   | ✅ Yes              | Relative pose          | 3x3         | Intrinsic calibration required |
| **Homography Matrix (H)**  | ❌ No               | Planar scenes or pure rotation | 3x3 | Not for general 3D scenes |
| **Projection Matrix**      | ✅ Yes              | 3D-to-2D projection    | 3x4         | Combines intrinsics and extrinsics |
| **Camera Matrix (K)**      | ✅ Yes              | Camera intrinsics      | 3x3         | Used to compute E from F |

---

## 🔧 Compatible Items

- [[OpenCV]] (`cv2.findFundamentalMat`, `cv2.findEssentialMat`)
- [[RANSAC]] (Used to robustly estimate E or F)
- [[Triangulation]] (3D point reconstruction)
- [[SIFT]] / [[ORB]] (Feature descriptors used for matching)
- [[Pose Estimation]] (Decomposing E into R and t)
- [[Stereo Vision]] (Leverages epipolar geometry)

---

## 🔗 Related Concepts

- [[Feature Matching]] (Initial step before computing E or F)
- [[Homography]] (Alternative model used in special cases)
- [[Pinhole Camera Model]] (Foundation of projection)
- [[Stereo Calibration]] (Estimates camera parameters and epipolar constraints)
- [[Structure from Motion]] (Relies heavily on epipolar geometry)

---

## 🛠 Developer Tools

- `cv2.findFundamentalMat()` with `cv2.RANSAC` option
- `cv2.findEssentialMat()` and `cv2.recoverPose()`
- MATLAB: `estimateFundamentalMatrix`, `estimateEssentialMatrix`
- ROS stereo calibration tools

---

## 📚 Further Reading

- Hartley & Zisserman, *Multiple View Geometry in Computer Vision*
- OpenCV Epipolar Geometry tutorials
- Richard Szeliski, *Computer Vision: Algorithms and Applications*

---
