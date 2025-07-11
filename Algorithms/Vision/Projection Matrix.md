# Projection Matrix

The **Projection Matrix** is a fundamental concept in computer vision and robotics used to project 3D world points onto a 2D image plane. It encapsulates both the intrinsic parameters of a camera (how the camera sees) and the extrinsic parameters (where the camera is and how it's oriented in the world). This matrix is essential in tasks such as [[Pose Estimation]], [[SfM]], [[SLAM]], and [[Triangulation]].

---

## 📚 Overview

The projection matrix defines the transformation from 3D world coordinates to 2D image coordinates using the equation:

`x = P * X`

Where:  
- `X` is a 3D point in homogeneous coordinates  
- `P` is the 3×4 projection matrix  
- `x` is the projected 2D image point in homogeneous coordinates  

The matrix `P` is constructed as:

`P = K [R | t]`

Where:  
- `K` is the **intrinsic matrix** (camera parameters)  
- `[R | t]` is the **extrinsic matrix** (camera pose: rotation and translation)

---

## 🧠 Core Concepts

- **Intrinsic Matrix (K)**: Includes focal length, principal point, and skew  
- **Extrinsic Matrix ([R | t])**: Describes the camera's position and orientation  
- **Homogeneous Coordinates**: Enables projection using matrix multiplication  
- **Perspective Division**: Final image coordinates are obtained by normalizing the result (dividing by the last element)  
- **Multiple View Geometry**: Projection matrices are key to stereo and multi-view setups  

---

## 🧰 Use Cases

- Converting 3D points into 2D image coordinates  
- Computing reprojection error in optimization problems  
- [[Triangulation]] from multiple views  
- [[SfM]]: Estimating camera poses and 3D structure  
- Rendering synthetic views in simulations  

---

## ✅ Pros

- Provides a unified framework for projection  
- Encapsulates both camera calibration and pose  
- Works with any pinhole camera setup  
- Extensively used in academic and real-world vision systems  

---

## ❌ Cons

- Assumes perfect pinhole model unless corrected for distortion  
- Can be numerically unstable without proper calibration  
- Requires accurate K, R, and t to be meaningful  
- Nonlinear optimization may be needed for refinement  

---

## 📊 Comparison: Projection Matrix vs Related Concepts

| Concept             | Dimensions | Purpose                          | Includes Camera Pose | Includes Calibration |
|---------------------|------------|----------------------------------|-----------------------|----------------------|
| Projection Matrix P | 3×4        | Projects 3D to 2D                | Yes                   | Yes                  |
| Intrinsic Matrix K  | 3×3        | Describes internal camera params | No                    | Yes                  |
| Extrinsic Matrix    | 3×4        | Pose: rotation and translation   | Yes                   | No                   |
| Homography Matrix   | 3×3        | 2D-2D planar transformation      | No                    | Sometimes            |
| Fundamental Matrix  | 3×3        | Epipolar constraint between views| No                    | No                   |

---

## 🤖 In a Robotics Context

| Task                         | Use of Projection Matrix                  |
|------------------------------|-------------------------------------------|
| [[Pose Estimation]]          | Reproject 3D model points to 2D to estimate pose  
| [[SfM]]                      | Estimate P matrices from matched keypoints  
| [[SLAM]]                     | Transform landmarks to camera frame  
| [[Triangulation]]            | Use multiple projection matrices to estimate 3D structure  
| Camera Simulation            | Render synthetic scenes using known camera parameters  

---

## 🔧 Libraries and Tools

- `cv::projectPoints()` – Projects 3D points using P in [[OpenCV]]  
- `cv::calibrateCamera()` – Computes K and P matrices  
- `image_geometry::PinholeCameraModel` – ROS wrapper for camera models  
- `COLMAP`, `OpenMVG`, `Theia` – SfM libraries that compute P matrices  
- `Kalibr` – Calibrates multi-camera systems and produces projection matrices  

---

## 🔧 Compatible Items

- [[Pinhole Camera Model]] – Underlying model used to form projection matrix  
- [[Pose Estimation]] – Projection matrix is central to reprojection error  
- [[SfM]] – Constructs projection matrices for all frames  
- [[Triangulation]] – Requires known projection matrices  
- [[Camera Calibration]] – Determines intrinsic part of P  

---

## 🔗 Related Concepts

- [[Pinhole Camera Model]] (Defines how projection works)  
- [[Pose Estimation]] (Uses P to estimate camera position)  
- [[SfM]] (Recovers projection matrices and structure)  
- [[Triangulation]] (Requires P from at least two views)  
- [[Image Plane]] (Target of 3D projection)  

---

## 📚 Further Reading

- [Multiple View Geometry by Hartley & Zisserman](https://www.cambridge.org/core/books/multiple-view-geometry-in-computer-vision/75C4BBF3D38D7D7EC70D582893BF78C6)  
- [OpenCV Camera Calibration](https://docs.opencv.org/master/dc/dbb/tutorial_py_calibration.html)  
- [Wikipedia - Camera Matrix](https://en.wikipedia.org/wiki/Camera_resectioning)  
- [OpenCV Projection Docs](https://docs.opencv.org/master/d9/d0c/group__calib3d.html#ga29d58cf4c9ce8b305be72f6dfdbff3ee)

---
