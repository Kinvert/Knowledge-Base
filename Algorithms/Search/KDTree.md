# KDTree (k-Dimensional Tree)

A KDTree is a space-partitioning data structure for organizing points in a k-dimensional space. It is widely used in robotics, computer vision, and machine learning for tasks like nearest neighbor search, clustering, and fast data retrieval. In robotics, it enables efficient lookup of spatial data such as feature points, LiDAR scans, and occupancy maps.

---

## 🧠 Overview

KDTree recursively splits data along axis-aligned hyperplanes to build a binary tree. This hierarchical structure enables efficient search operations, making it ideal for real-time systems that need to handle large volumes of high-dimensional data.

---

## ⚙️ How It Works

1. Choose an axis (usually alternating axes or the one with highest variance).
2. Sort data points along that axis.
3. Choose the median point as the node.
4. Recursively build left and right subtrees from the points on either side.
5. For nearest neighbor queries, prune large portions of the search space by comparing distances to the splitting planes.

---

## 🧩 Core Concepts

- **Balanced Tree**: Ensures logarithmic query time by selecting medians during construction.
- **Bounding Hyperplanes**: Used to prune the search space efficiently.
- **Dimensionality**: Typically best for dimensions < 30; performance degrades in higher-dimensional spaces (curse of dimensionality).

---

## 🔍 Use Cases

- Nearest neighbor search in SLAM
- Fast correspondence matching in ICP
- Clustering in high-dimensional sensor data
- Path planning proximity queries
- Obstacle avoidance and environment modeling

---

## ✅ Pros

- Fast nearest neighbor search in low-to-moderate dimensions
- Simple and well-supported in libraries
- Efficient memory use

---

## ❌ Cons

- Construction can be slow for large datasets
- Not well-suited for dynamic data (requires rebuilding)
- Degrades in high dimensions (>20–30)

---

## 📊 Comparison Chart

| Data Structure   | Search Time (Low-D) | Good for High-D? | Dynamic Updates | Notes |
|------------------|---------------------|------------------|------------------|-------|
| **KDTree**       | ✅ Fast             | ❌ No            | ❌ No            | Great for 2D/3D |
| **Ball Tree**    | ✅ Fast             | ⚠️ Moderate      | ❌ No            | Better for uneven distributions |
| **Octree**       | ✅ Fast (3D)        | ❌ No            | ✅ Yes           | Common in point clouds |
| **R-Tree**       | ✅ Good             | ⚠️ Limited       | ✅ Yes           | Often used in spatial indexing |
| **Brute Force**  | ❌ Slow             | ✅ Yes           | ✅ Yes           | Simple but slow |

---

## 🔧 Compatible Items

- [[PCL]] (Offers KDTree for point cloud search)
- [[OpenCV]] (Has FLANN-based KDTree implementation)
- [[FLANN]] (Fast Library for Approximate Nearest Neighbors)
- [[ICP]] (Uses KDTree for point matching)
- [[ROS]] (Often used internally in point cloud filters)

---

## 📚 Related Concepts

- [[Octree]] (Alternative spatial data structure for 3D)
- [[Nearest Neighbor Search]] (Core use of KDTree)
- [[Point Cloud Processing]] (Common context for KDTree usage)
- [[BoW]] (Often uses KDTree for fast descriptor matching)
- [[KMeans]] (KDTree may be used for acceleration)

---

## 🛠 Developer Tools

- `scipy.spatial.KDTree` (Python)
- `sklearn.neighbors.KDTree` (Python, more features)
- `cv::flann::KDTreeIndexParams` (OpenCV C++)
- `pcl::KdTreeFLANN` (C++ PCL)

---

## 📚 Further Reading

- Bentley, J. L. (1975). *Multidimensional binary search trees used for associative searching.*
- PCL tutorials on KDTree search
- OpenCV and SciPy documentation

---
