# Pinhole Camera Model

The **Pinhole Camera Model** is a simplified mathematical model that describes how a 3D point in the world is projected onto a 2D image plane through a single point (the "pinhole"). It is widely used in computer vision, robotics, and photogrammetry for tasks involving image formation, camera calibration, 3D reconstruction, and pose estimation.

---

## 📚 Overview

In the pinhole model, the camera is treated as a simple box with a tiny hole on one side. Light from the scene enters through the pinhole and projects an inverted image on the opposite side. Although idealized (real cameras have lenses), this model is foundational for understanding and implementing many computer vision algorithms.

The central equation is:

`x = K [R | t] X`

Where:  
- `X` is the 3D point in world coordinates  
- `[R | t]` is the extrinsic matrix (rotation and translation)  
- `K` is the intrinsic camera matrix  
- `x` is the 2D image point in homogeneous coordinates  

---

## 🧠 Core Concepts

- **Intrinsic Parameters**: Describe the internal camera geometry (focal length, principal point, skew)  
- **Extrinsic Parameters**: Describe the camera's position and orientation in the world  
- **Projection Matrix**: Combines intrinsic and extrinsic parameters to map 3D → 2D  
- **Homogeneous Coordinates**: Used to represent points and transformations consistently  
- **Radial/Tangential Distortion**: Not modeled in basic pinhole camera but added during calibration  

---

## 🧰 Use Cases

- Camera calibration and lens modeling  
- [[Pose Estimation]] from known 2D-3D correspondences  
- [[Structure from Motion]] and [[SLAM]]  
- Depth estimation from stereo cameras  
- 3D to 2D projection for synthetic rendering  

---

## ✅ Pros

- Simple and computationally efficient  
- Forms the mathematical basis for many vision techniques  
- Easily extended with distortion models  
- Compatible with various camera types (RGB, IR, stereo)  

---

## ❌ Cons

- Ignores lens distortion by default  
- Assumes ideal pinhole projection (no blur, aperture effects)  
- Inaccurate at wide fields of view without corrections  
- Real cameras often require more complex models  

---

## 📊 Comparison: Camera Models

| Model                   | Lens Distortion | Complexity | Accuracy        | Common Use                  |
|-------------------------|------------------|------------|------------------|------------------------------|
| Pinhole Camera Model    | No               | Low        | Moderate         | Foundational vision models   |
| Pinhole + Distortion    | Yes              | Medium     | High             | Real-world camera calibration|
| Fisheye Model           | Yes              | High       | Very High        | Wide-angle lenses            |
| Orthographic Projection | No               | Very Low   | Low              | 2D applications or rendering |
| Thin Lens Model         | Yes              | High       | High             | Optical simulation           |

---

## 🤖 In a Robotics Context

| Application                | Use of Pinhole Model                      |
|----------------------------|-------------------------------------------|
| SLAM & Visual Odometry     | Projects 3D map points onto 2D image plane  
| Robot Navigation           | Camera-based localization and mapping  
| 3D Reconstruction          | Uses projection model to infer depth  
| Stereo Vision              | Assumes calibrated pinhole for triangulation  
| Camera Calibration         | Determines `K`, `R`, and `t` using the model  

---

## 🔧 Common Tools and Libraries

- `cv::projectPoints()` – Projects 3D points using pinhole model in [[OpenCV]]  
- `cv::calibrateCamera()` – Estimates intrinsic/extrinsic params from patterns  
- `image_geometry::PinholeCameraModel` – [[ROS]] package for camera models  
- `CameraInfo.msg` – ROS message type containing intrinsic parameters  
- `Kalibr`, `COLMAP`, `OpenMVG` – Use this model for structure-from-motion  

---

## 🔧 Compatible Items

- [[Pose Estimation]] – Requires 3D → 2D projection  
- [[SfM]] (Structure from Motion) – Uses pinhole model in camera pose estimation  
- [[Camera Calibration]] – Based on estimating this model's parameters  
- [[Stereo Vision]] – Assumes rectified pinhole model for disparity  
- [[Projection Matrix]] – Built from intrinsic and extrinsic components  

---

## 🔗 Related Concepts

- [[Pose Estimation]] (Projects known 3D points to 2D image points)  
- [[Camera Calibration]] (Estimates parameters of the pinhole model)  
- [[Structure from Motion]] (Uses projection to infer structure and motion)  
- [[Triangulation]] (Requires 2D points from a known projection)  
- [[Image Plane]] (2D surface where projections land)  

---

## 📚 Further Reading

- [OpenCV Camera Calibration](https://docs.opencv.org/master/dc/dbb/tutorial_py_calibration.html)  
- [Hartley & Zisserman, "Multiple View Geometry"](https://www.cambridge.org/core/books/multiple-view-geometry-in-computer-vision/75C4BBF3D38D7D7EC70D582893BF78C6)  
- [ROS image_geometry package](http://wiki.ros.org/image_geometry)  
- [Colmap Pinhole Model](https://colmap.github.io/format.html#camera-models)  
- [Wikipedia - Pinhole Camera Model](https://en.wikipedia.org/wiki/Pinhole_camera_model)

---
