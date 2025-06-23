# 🚗 Visual Odometry

**Visual Odometry (VO)** is the process of estimating the motion of a camera (or robot) over time by analyzing the sequence of images it captures. It plays a key role in robotics, AR/VR, autonomous vehicles, and drones where GPS may be unavailable or unreliable.

---

## 🧠 Summary

- VO estimates **incremental motion** (translation + rotation) from frame to frame using visual input.
- Typically used when wheel odometry, GPS, or IMU data are absent, unreliable, or complementary.
- Can work with monocular, stereo, or RGB-D cameras.

---

## ⚙️ Types of Visual Odometry

| Type           | Input                    | Notes                                      |
|----------------|--------------------------|--------------------------------------------|
| **Monocular VO** | Single camera             | Scale ambiguity; sensitive to drift        |
| **Stereo VO**    | Stereo camera pair        | Recovers metric scale                      |
| **RGB-D VO**     | RGB camera + depth sensor | Provides direct depth measurements         |
| **Visual-Inertial** | Camera + IMU             | Fuses VO with inertial data for robustness |

---

## 📌 Key Steps in VO

1. **Feature detection and matching** (e.g., [[ORB]], [[SIFT]], [[FAST]])
2. **Motion estimation** (compute relative pose between frames)
3. **Outlier rejection** (e.g., RANSAC)
4. **Pose integration** (accumulate motion over time)
5. (Optional) **Bundle adjustment** (refine poses and 3D structure globally)

---

## 🔄 Comparison with SLAM

| Aspect                  | Visual Odometry                | [[SLAM]]                         |
|--------------------------|--------------------------------|-----------------------------------|
| Builds map?              | ❌ No (or minimal local map)   | ✅ Yes (persistent map of environment) |
| Loop closure             | ❌ No                          | ✅ Yes                            |
| Accumulated drift        | ✅ Yes (drift grows over time) | ⚠️ Reduced by loop closure        |
| Complexity               | ⚡ Lower                       | 🐢 Higher                        |

---

## 🏆 Strengths

- Lightweight and fast.
- Suitable for real-time systems.
- Works well in GPS-denied environments.

---

## ⚠️ Weaknesses

- Drift accumulation without loop closure.
- Sensitive to poor lighting, motion blur, and textureless scenes.
- Monocular VO suffers from scale ambiguity.

---

## 🚀 Use Cases

- Autonomous vehicles
- Drones and UAVs
- AR/VR headsets
- Robotics (indoor and outdoor navigation)

---

## 🔗 Related Notes

- [[SLAM]]
- [[Monocular SLAM]]
- [[Binocular SLAM]]
- [[ORB-SLAM]]
- [[Feature Detection]]

---

## 🌐 External References

- [Wikipedia - Visual Odometry](https://en.wikipedia.org/wiki/Visual_odometry)
- [OpenCV VO Example](https://docs.opencv.org/master/dc/dc3/tutorial_py_matcher.html)

---
