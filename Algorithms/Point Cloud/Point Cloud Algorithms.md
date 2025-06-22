# 🟢 Point Cloud Algorithms

**Point cloud algorithms** process and analyze sets of 3D points (point clouds), typically produced by LiDAR, stereo vision, structured light, or depth cameras. These algorithms extract meaningful information such as surfaces, shapes, objects, and features from raw 3D data.

---

## 🧠 Summary

- **Input**: Set of (x, y, z) points (optionally with color, intensity, normal vectors, etc.)
- **Goal**: Understand structure, extract features, reconstruct surfaces, or enable downstream tasks like SLAM or object recognition
- **Common domains**: Robotics, autonomous vehicles, 3D scanning, AR/VR, geospatial mapping

---

## 🛠️ Common Types of Point Cloud Algorithms

| Category                   | Example Algorithms                   | Purpose                                       |
|----------------------------|---------------------------------------|-----------------------------------------------|
| **Filtering / Preprocessing** | Voxel grid, Statistical Outlier Removal | Downsample, denoise point clouds             |
| **Segmentation**            | RANSAC, Region growing, Euclidean clustering | Divide cloud into meaningful parts           |
| **Registration**            | ICP (Iterative Closest Point), NDT    | Align two or more point clouds                |
| **Feature extraction**       | FPFH, SHOT, ISS                      | Describe local geometry, find keypoints       |
| **Surface reconstruction**   | Greedy Triangulation, Poisson, MLS   | Create mesh/surface from point cloud          |
| **Clustering**              | DBSCAN, K-Means, Euclidean clustering | Group points into objects                     |

---

## ✅ Strengths

- Captures rich 3D structure information
- Non-invasive sensing (e.g. LiDAR)
- Enables advanced perception in robotics, ADAS, mapping

---

## ❌ Weaknesses

- Large datasets → computationally heavy
- Sensitive to noise and outliers
- Sparse data can make feature extraction or reconstruction difficult

---

## 🌐 External References

- [PCL (Point Cloud Library)](https://pointclouds.org/)
- [Open3D](http://www.open3d.org/)
- [Stanford 3D Scanning Repository](http://graphics.stanford.edu/data/3Dscanrep/)

---

## 🔗 Related Notes

- [[Algorithms]]
- [[PCL (Point Cloud Library)]]
- [[Open3D]]
- [[SLAM]]
- [[Depth Estimation]]
- [[ICP]]
- [[3D Reconstruction]]
- [[LiDAR]]

---
