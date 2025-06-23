# 🔹 ORB (Oriented FAST and Rotated BRIEF)

**ORB (Oriented FAST and Rotated BRIEF)** is a fast, efficient, and rotation-invariant feature detector and descriptor widely used in computer vision and robotics applications. It combines the FAST keypoint detector with the BRIEF descriptor and extends them with orientation and scale invariance for robustness.

---

## 🧠 Summary

- Published in 2011 by Ethan Rublee et al.
- Designed as a free alternative to SIFT and SURF with similar accuracy and much faster computation.
- Works well for real-time applications on low-power devices (e.g., embedded systems, mobile platforms).

---

## ⚙️ Key Components

- **[[FAST]] (Features from Accelerated Segment Test)**: Detects keypoints quickly.
- **Orientation assignment**: Adds rotation invariance by computing the intensity centroid of the patch.
- **[[BRIEF]] (Binary Robust Independent Elementary Features)**: Describes the keypoint using binary strings.
- **Rotation & scale invariance**: ORB modifies BRIEF to be rotation-invariant and robust against scale changes.

---

## 🏆 Strengths

- Very fast to compute; suitable for real-time applications.
- Good balance between speed and descriptive power.
- Robust to rotation and moderate scale changes.
- Open source and patent-free.

---

## ⚠️ Weaknesses

- Less robust to large scale changes compared to SIFT/SURF.
- Performance can degrade in low-texture or highly repetitive patterns.
- Binary descriptor can be less discriminative in some cases compared to floating-point descriptors.

---

## 🔄 Comparison with Other Feature Detectors

| Feature                    | ORB               | [[SIFT]]               | [[SURF]]               | [[AKAZE]]           |
|----------------------------|------------------|-------------------|-------------------|--------------------|
| Descriptor type            | Binary            | Float              | Float              | Binary              |
| Scale invariant            | ⚠️ Limited        | ✅ Yes             | ✅ Yes             | ✅ Yes              |
| Rotation invariant         | ✅ Yes             | ✅ Yes             | ✅ Yes             | ✅ Yes              |
| Speed                      | 🚀 Very fast       | 🐢 Slow            | 🐢 Slow            | ⚡ Fast              |
| Patent restrictions        | ✅ None            | ❌ Patent-encumbered | ❌ Patent-encumbered | ✅ None             |

---

## 🚀 Use Cases

- [[ORB-SLAM]] and other SLAM systems
- Augmented reality (AR) tracking
- 3D reconstruction
- Object detection and tracking
- Visual odometry

---

## 🌐 External References

- [Original ORB Paper](https://ieeexplore.ieee.org/document/6126544)
- [OpenCV ORB Documentation](https://docs.opencv.org/master/db/d95/classcv_1_1ORB.html)

---

## 🔗 Related Notes

- [[Feature Detection]]
- [[ORB-SLAM]]
- [[AKAZE]]
- [[FAST]]
- [[BRIEF]]

---
