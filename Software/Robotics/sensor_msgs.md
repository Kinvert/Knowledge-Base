# 🟣 sensor_msgs

**sensor_msgs** is a standard [[ROS2]] (and ROS 1) message package that defines commonly used message types for representing sensor data. These messages provide a consistent structure for exchanging information from sensors like cameras, LiDARs, IMUs, and more between ROS nodes.

---

## 🧠 Summary

- Contains message definitions for a wide range of sensors.
- Designed to promote interoperability between nodes and systems.
- Used extensively in robotics applications involving perception, navigation, and mapping.

---

## ⚙️ Common Message Types

| Message Type              | Description                                              |
|---------------------------|----------------------------------------------------------|
| `Image`                   | 2D image data (e.g. from a camera)                       |
| `CompressedImage`          | JPEG/PNG-compressed image data                           |
| `CameraInfo`               | Camera calibration and intrinsic parameters              |
| `LaserScan`                | 2D LiDAR or range scanner data                          |
| `PointCloud` / `PointCloud2`| 3D point cloud data (e.g. from LiDAR or depth cameras) |
| `Imu`                      | Inertial Measurement Unit data (accel, gyro, orientation)|
| `NavSatFix`                | GPS position data                                       |
| `Range`                    | Simple range data (e.g. ultrasonic sensor)              |
| `FluidPressure`            | Barometric pressure readings                            |
| `MagneticField`            | Magnetometer data                                       |
| `JointState`               | Joint position, velocity, and effort (often in `sensor_msgs` though conceptually fits elsewhere) |

---

## 🚀 Example Use Cases

- Publishing camera feeds using `sensor_msgs/Image` or `sensor_msgs/CompressedImage`.
- Exchanging LiDAR scans via `LaserScan` or `PointCloud2`.
- Sharing IMU data between perception and control nodes.
- Broadcasting GPS location using `NavSatFix`.

---

## 🏆 Strengths

- Standardized message formats simplify integration between components.
- Wide tool support (e.g. RViz, [[Foxglove]], ROS bag recording).
- Well-documented and mature.

---

## ⚠️ Weaknesses

- Some messages (e.g. `PointCloud`) are older and superseded by more efficient alternatives (e.g. `PointCloud2`).
- Can require careful configuration (e.g. coordinate frames, timestamps) to ensure correct interpretation.

---

## 🔗 Related Notes

- [[ROS2]]
- [[ROS2 Topics]]
- [[PCL]]
- [[OpenCV]] (often works alongside `sensor_msgs/Image`)
- [[Foxglove]]
- [[RViz]]

---

## 🌐 External References

- [ROS 2 sensor_msgs API](https://docs.ros2.org/latest/api/sensor_msgs/index.html)
- [ROS Wiki sensor_msgs (ROS 1, still relevant)](http://wiki.ros.org/sensor_msgs)

---
