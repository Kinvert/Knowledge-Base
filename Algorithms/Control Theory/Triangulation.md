# Triangulation

Triangulation is the process of estimating a 3D point in space by intersecting multiple lines of sight (rays) from different viewpoints. In robotics and computer vision, triangulation is a foundational technique for 3D reconstruction, stereo vision, visual SLAM, and structure-from-motion (SfM).

---

## 📚 Overview

Given the 2D projections of a point in two or more images and the corresponding camera poses, triangulation reconstructs the original 3D coordinates of that point. This relies on the principle of epipolar geometry and accurate camera calibration.

---

## 🧠 Core Concepts

- **Correspondences**: Matching 2D points across multiple views.
- **Back-projection**: Each 2D point defines a ray in 3D space from the camera center.
- **Intersection**: The 3D point is found where rays from different cameras intersect or come closest.
- **Linear vs Nonlinear Methods**: Linear triangulation (e.g. DLT) is fast; nonlinear methods minimize reprojection error.

---

## 🧰 Use Cases

- 3D scene reconstruction
- Landmark initialization in SLAM
- Stereo vision depth estimation
- Structure-from-Motion (SfM)
- Multi-view geometry in camera calibration

---

## ✅ Pros

- Passive 3D sensing (no need for active sensors)
- Simple and effective with calibrated cameras
- Scales to multi-camera systems

---

## ❌ Cons

- Highly sensitive to noise and calibration error
- Accuracy decreases with narrow baseline or poor correspondences
- Occlusions and dynamic scenes complicate matching

---

## 📊 Comparison Chart

| Method                  | Input               | Optimizes      | Accuracy       | Notes                          |
|--------------------------|--------------------|----------------|----------------|--------------------------------|
| **Linear Triangulation** | 2D pts + P matrices | Algebraic error| ⚠️ Moderate     | Fast, but less accurate        |
| **Nonlinear Triangulation** | 2D pts + P matrices | Reprojection error | ✅ High    | Uses optimization (e.g., Levenberg-Marquardt) |
| **Stereo Block Matching** | Dense stereo images | Disparity map  | ⚠️ Low to Medium | Good for dense reconstructions |
| **Multiview Triangulation** | >2 images          | Reprojection error | ✅ Very High  | Used in SfM, improves accuracy |

---

## 🔧 Compatible Items

- [[OpenCV]] (`cv2.triangulatePoints`)
- [[Camera Calibration]] (Required for accurate triangulation)
- [[Epipolar Geometry]] (Constrains 2D point search)
- [[Stereo Vision]] (Triangulation from stereo pairs)
- [[SLAM]] (Triangulates landmarks from multiple poses)

---

## 🔗 Related Concepts

- [[PnP]] (Estimates pose using 3D points from triangulation)
- [[Epipolar Geometry]] (Supports triangulation)
- [[Structure from Motion]] (Uses triangulation for 3D points)
- [[Feature Matching]] (Provides 2D correspondences)
- [[Pose Estimation]] (Triangulated points improve pose accuracy)

---

## 🛠 Developer Tools

- `cv2.triangulatePoints()` in OpenCV
- `cv::sfm::triangulatePoints()` in OpenCV’s SfM module
- Ceres Solver (for nonlinear triangulation)
- MATLAB: `triangulate()` in the Computer Vision Toolbox

---

## 📚 Further Reading

- Hartley & Zisserman, *Multiple View Geometry*
- Richard Szeliski, *Computer Vision: Algorithms and Applications*
- OpenCV stereo vision and triangulation tutorials

---
