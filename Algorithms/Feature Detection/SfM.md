# Structure from Motion (SfM)

**Structure from Motion (SfM)** is a computer vision technique that reconstructs 3D structure from a series of 2D images taken from different viewpoints. It simultaneously estimates both the scene geometry and the camera motion, and is commonly used in robotics, photogrammetry, and 3D modeling.

---

## 📚 Overview

SfM works by identifying feature correspondences across multiple images and using geometric constraints to infer the 3D structure of the scene and the trajectory of the camera. It's foundational in offline visual SLAM systems and serves as a precursor to more real-time capable algorithms.

SfM typically includes:
1. **Feature Detection & Matching** ([[SIFT]], [[ORB]], etc.)
2. **Camera Pose Estimation** ([[Pose Estimation]])
3. **Triangulation** of matched points into 3D
4. **Bundle Adjustment** for global optimization

---

## 🧠 Core Concepts

- **Epipolar Geometry**: Relationship between image pairs used in camera pose recovery  
- **Triangulation**: Reconstructs 3D points from matching 2D points  
- **Camera Calibration**: Needed for accurate reconstructions  
- **Incremental or Global Approaches**: Add cameras one by one or optimize all at once  
- **Bundle Adjustment**: Minimizes reprojection error across all views  

---

## 🧰 Use Cases

- Offline 3D mapping and modeling  
- Aerial photogrammetry (e.g., drone surveys)  
- Archaeological and architectural reconstructions  
- Autonomous vehicle map building  
- Preprocessing for SLAM or 3D localization  

---

## ✅ Pros

- Provides dense or sparse 3D reconstructions from 2D images  
- Accurate camera motion and 3D structure estimation  
- Works with unordered image sets  
- Can operate without GPS or IMU  

---

## ❌ Cons

- Computationally expensive, especially with many images  
- Not suitable for real-time applications without adaptation  
- Requires good feature matches and sufficient parallax  
- Sensitive to lighting and texture variation  

---

## 📊 Comparison Chart: SfM vs Related Techniques

| Technique            | Input           | Real-Time | Output             | Use Case                  |
|----------------------|------------------|-----------|----------------------|----------------------------|
| Structure from Motion| Images           | No        | Sparse 3D + Camera Poses | Offline 3D modeling     |
| Visual SLAM          | Images (live)    | Yes       | Pose + Map (sparse)     | Real-time robotics        |
| Stereo Vision        | Calibrated pair  | Yes       | Dense 3D               | Depth estimation          |
| LiDAR Mapping        | 3D scans         | Yes       | Dense 3D Map           | Autonomous vehicles       |
| Photogrammetry       | Images           | No        | Dense 3D + Textures    | Surveying, modeling       |

---

## 🤖 In a Robotics Context

| Scenario                  | SfM Role                                      |
|---------------------------|-----------------------------------------------|
| UAV Reconnaissance        | Creates maps from aerial images               |
| Autonomous Ground Robots  | Used in map initialization                    |
| Visual Localization       | Builds reference maps from past journeys      |
| 3D Scene Reconstruction   | Provides ground truth models                  |
| Research & Simulation     | Useful for generating synthetic environments  |

---

## 🔧 Common Libraries & Tools

- `COLMAP` – State-of-the-art SfM tool with GUI and CLI  
- `OpenMVG` – Modular SfM pipeline with easy customization  
- `Theia` – C++ library focused on SfM research  
- `Meshroom` – Node-based photogrammetry GUI using AliceVision  
- `OpenCV` – Low-level primitives for feature extraction, triangulation, etc.  

---

## 🔧 Compatible Items

- [[Keypoint Matching]] – Core to finding image correspondences  
- [[Triangulation]] – Converts 2D matches into 3D structure  
- [[Pose Estimation]] – Estimates camera pose from correspondences  
- [[Bundle Adjustment]] – Optimizes full model  
- [[OpenCV]] – For custom low-level SfM pipelines  

---

## 🔗 Related Concepts

- [[Epipolar Geometry]] (Used to estimate relative camera pose)  
- [[Triangulation]] (Transforms matched 2D points to 3D)  
- [[Feature Descriptors]] (Required for keypoint matching)  
- [[Visual Odometry]] (Related real-time motion estimation)  
- [[SLAM]] (SfM often used for map building in SLAM)  

---

## 📚 Further Reading

- [COLMAP Documentation](https://colmap.github.io/)  
- [Structure from Motion Pipeline (CMU)](https://cmu-robotics.github.io/sfm/)  
- [Snavely et al., "Photo Tourism" (2006)](https://www.cs.cornell.edu/projects/phototourism/)  
- [OpenMVG Docs](https://openmvg.readthedocs.io/en/latest/)  
- [SfM on Wikipedia](https://en.wikipedia.org/wiki/Structure_from_motion)

---
