# Image Plane

The **Image Plane** is the 2D surface within a camera model where 3D world points are projected to form an image. It is a conceptual tool used in the **[[Pinhole Camera Model]]** and other projection models to understand how cameras convert the real world into 2D imagery.

---

## 📚 Overview

In the pinhole camera abstraction, light rays from 3D objects pass through a single point (the camera's center or focal point) and intersect the image plane. This intersection defines where and how the 3D point appears in the final image. The image plane is positioned perpendicular to the camera’s optical axis and contains the origin of pixel coordinates (usually at the principal point).

It serves as the destination of the projection transformation, where coordinates are measured in pixels and referenced for image processing tasks like feature detection, segmentation, and reconstruction.

---

## 🧠 Core Concepts

- **Principal Point**: The point where the optical axis intersects the image plane  
- **Pixel Coordinates**: Coordinates of points projected onto the image plane  
- **Intrinsic Matrix (K)**: Maps camera frame points to the image plane  
- **Projection Equation**: `x = K [R|t] X`, where `x` lies on the image plane  
- **Focal Length**: Determines the scale of projection on the image plane  

---

## 🧰 Use Cases

- 2D projections in [[Pose Estimation]]  
- Reprojecting 3D landmarks during [[SLAM]]  
- Simulating synthetic views for [[Camera Calibration]]  
- Mapping sensor data onto visual imagery  
- Computing reprojection error during optimization  

---

## ✅ Pros

- Simplifies modeling of how images are formed  
- Forms the foundation of most vision algorithms  
- Directly corresponds to camera sensor output  
- Easily understood and applied in both real and simulated systems  

---

## ❌ Cons

- Idealized: ignores sensor-specific noise, blur, and distortion  
- Inaccurate for wide-angle or fisheye lenses without corrections  
- 2D simplification—loses depth and occlusion information  

---

## 📊 Comparison: Projection Concepts

| Concept             | Domain       | Dimensionality | Includes Intrinsics? | Notes                          |
|---------------------|--------------|----------------|------------------------|---------------------------------|
| World Coordinates   | 3D global    | 3D             | No                     | Real-world frame                |
| Camera Frame        | 3D local     | 3D             | No                     | Relative to camera pose         |
| Image Plane         | 2D sensor    | 2D             | Yes (via `K`)          | Where pixels are defined        |
| Image Sensor        | Hardware     | 2D             | Yes                    | Physical counterpart to image plane |

---

## 🤖 In a Robotics Context

| Application              | Role of Image Plane                          |
|--------------------------|-----------------------------------------------|
| [[Pose Estimation]]      | Projects known 3D landmarks onto the image plane  
| [[SLAM]]                 | Matches 2D observations to 3D map landmarks  
| [[Camera Calibration]]   | Maps image points to real-world geometry  
| Stereo Vision            | Disparity is measured across image planes  
| Visual Odometry          | Tracks features on the image plane across frames  

---

## 🔧 Compatible Items

- [[Pinhole Camera Model]] – Projects 3D points onto the image plane  
- [[Projection Matrix]] – Final step results in 2D points on the plane  
- [[Pose Estimation]] – Relates world points to their image plane projections  
- [[Feature Detectors]] – Detect points in the image plane  
- [[Camera Calibration]] – Computes intrinsic parameters tied to image plane geometry  

---

## 🔗 Related Concepts

- [[Pinhole Camera Model]] (Defines geometry of projection)  
- [[Projection Matrix]] (Used to compute image plane location)  
- [[Camera Calibration]] (Relates real-world coordinates to image plane)  
- [[Image Coordinates]] (Pixel representation on the image plane)  
- [[Intrinsic Matrix]] (Maps camera frame to image plane)  

---

## 📚 Further Reading

- [Hartley & Zisserman – Multiple View Geometry](https://www.cambridge.org/core/books/multiple-view-geometry-in-computer-vision/75C4BBF3D38D7D7EC70D582893BF78C6)  
- [OpenCV Camera Calibration Tutorial](https://docs.opencv.org/master/dc/dbb/tutorial_py_calibration.html)  
- [Wikipedia – Image Plane](https://en.wikipedia.org/wiki/Image_plane)  
- [Coursera: Camera Models and Calibration (UC San Diego)](https://www.coursera.org/learn/introduction-computer-vision)  

---
