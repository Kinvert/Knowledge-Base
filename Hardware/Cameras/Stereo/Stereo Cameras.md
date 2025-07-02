# 👀 Stereo Cameras

**Stereo Cameras** are vision systems that use two or more image sensors placed at known distances apart to estimate depth information. By mimicking human binocular vision, stereo cameras capture images from slightly different perspectives and compute disparity to infer 3D structure in the scene.

---

## 🧠 Summary

Stereo cameras are a key part of many [[Computer Vision]] and [[Robotics]] systems, especially for tasks like [[SLAM]], [[Visual Odometry]], [[3D Reconstruction]], and [[Obstacle Avoidance]].

They are widely used in:

- Autonomous vehicles
- Drones and UAVs
- Robotics (e.g., manipulation and navigation)
- AR/VR applications
- Research and education

---

## 🧪 How Stereo Vision Works

1. **Image Capture**: Two (or more) cameras capture the same scene from slightly different horizontal positions.
2. **Calibration**: Intrinsic and extrinsic parameters of each camera are determined (focal length, distortion, baseline, etc.).
3. **Rectification**: Images are aligned so that epipolar lines are horizontal.
4. **Disparity Map**: Difference in the location of the same object in both images is computed (disparity).
5. **Depth Estimation**: Using the disparity and known baseline/focal length, a depth map is generated.

---

## 📦 Affordable Hobbyist Stereo Camera Options

| Model                        | Resolution         | Interface   | Depth Range   | Software Support              | Notes                                   |
|-----------------------------|--------------------|-------------|----------------|-------------------------------|-----------------------------------------|
| **Intel RealSense D435**    | 1280×720 @ 30 FPS  | USB 3.0     | 0.1 – 10m      | ROS, Python, C++, OpenCV       | Compact, RGB + Depth, good support      |
| **Luxonis OAK-D Lite**      | 1280×800 + AI      | USB-C       | ~0.2 – 10m     | DepthAI, ROS2, Python, OpenCV | Onboard AI, good for embedded vision    |
| **StereoPi V2**             | Depends on Pi Cams | CSI         | ~0.2 – 10m     | Raspbian, OpenCV, ROS         | DIY, flexible, Raspberry Pi-based       |
| **ZED Mini**                | 720p–1080p         | USB 3.0     | 0.1 – 15m      | ZED SDK, ROS, Unity, OpenCV   | Great SDK, works with Jetson            |
| **OpenCV AI Kit (OAK-D)**   | 4K + stereo        | USB-C       | ~0.3 – 10m     | DepthAI, ROS, Python          | Onboard inference, stereo + RGB         |

---

## 💼 Industrial (More Expensive) Example

| Model                        | Resolution          | Interface   | Depth Range   | Software Support             | Notes                                  |
|-----------------------------|---------------------|-------------|----------------|------------------------------|----------------------------------------|
| **Stereolabs ZED 2i**       | 2K/1080p/720p        | USB 3.0     | 0.2 – 20m      | ZED SDK, ROS, Unity, OpenCV  | High accuracy, IMU/Barometer onboard   |

> 🧠 Note: The **ZED 2i** is a common reference in research and industry but significantly more expensive ($400–$700+). Most hobbyists stick to RealSense or OAK-D level cameras.

---

## 📌 Pros and Cons

| Pros                              | Cons                                             |
|-----------------------------------|--------------------------------------------------|
| Passive depth sensing (no active IR) | Sensitive to lighting and texture               |
| Can work outdoors                  | Requires careful calibration                     |
| No signal interference             | Often lower depth precision than LiDAR          |
| Affordable options available       | Baseline/field of view trade-offs                |
| Integrates well with OpenCV/ROS    | Requires processing power for stereo matching   |

---

## 🧰 Typical Use Cases

- [[SLAM]] and [[Visual Odometry]]
- Mapping and localization
- Gesture recognition
- Obstacle detection in drones and mobile robots
- AR/VR scene depth understanding
- 3D reconstruction

---

## 🔗 Related Notes

- [[Depth Estimation]]
- [[Visual Odometry]]
- [[Stereo Matching]]
- [[SLAM]]
- [[ROS2]]
- [[Point Cloud Algorithms]]

---

## 🌐 External References

- [Intel RealSense Official](https://www.intelrealsense.com/)
- [Luxonis OAK](https://www.luxonis.com/)
- [Stereolabs ZED](https://www.stereolabs.com/)
- [StereoPi](https://stereopi.com/)
- [OpenCV AI Kit](https://docs.luxonis.com/)

---
