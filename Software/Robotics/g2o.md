# 🔧 g2o (General Graph Optimization)

**g2o** is a C++ framework for performing graph-based optimization, widely used in robotics and computer vision, especially in problems involving **SLAM**, **Bundle Adjustment**, and **Pose Graph Optimization**. It allows users to define custom vertex and edge types to model complex optimization problems as graphs.

---

## 🧠 Summary

- Developed at the University of Freiburg.
- Short for **General Graph Optimization**.
- Extensively used in **SLAM**, **Visual Odometry**, and **Structure from Motion** tasks.
- Enables efficient optimization over nonlinear error functions using techniques like Levenberg–Marquardt and Gauss–Newton.

---

## ⚙️ Core Concepts

| Concept         | Description                                                                 |
|-----------------|-----------------------------------------------------------------------------|
| **Vertex**      | Represents an unknown variable to optimize (e.g., camera pose, 3D point).   |
| **Edge**        | Represents a constraint or observation involving one or more vertices.      |
| **Optimization**| Minimizes the error functions defined by the edges over the vertices.       |
| **Solvers**     | Supports various sparse matrix solvers (e.g., Cholmod, CSparse, Eigen).     |

---

## ✅ Key Features

- Highly customizable: users can define new vertex and edge types.
- Supports both sparse and dense optimization problems.
- Efficient implementation based on sparse matrix libraries.
- Widely adopted in SLAM systems (e.g., [[ORB-SLAM2]], [[ORB-SLAM3]], [[LSD-SLAM]]).
- Supports various optimization algorithms.

---

## 🚀 Example Use Cases

- [[Pose Graph Optimization]] in SLAM back-ends.
- [[Bundle Adjustment]] in visual structure-from-motion.
- Loop closure adjustment in visual SLAM pipelines.
- Landmark optimization in robot mapping.

---

## 🆚 Comparison with Alternatives

| Library     | Language | Use Case                      | Strengths                          | Weaknesses                         |
|-------------|----------|-------------------------------|------------------------------------|------------------------------------|
| **g2o**     | C++      | SLAM, graph-based optimization| Very efficient, extensible         | Steep learning curve               |
| [[Ceres]]   | C++      | BA, optimization              | Great auto-diff, easy interface    | More general-purpose               |
| [[GTSAM]]   | C++      | SLAM, Factor Graphs           | Probabilistic framework, readable  | Larger dependencies, heavier       |
| [[iSAM2]]   | C++      | Incremental SLAM              | Incremental updates                | GTSAM-specific                     |

---

## 🔗 Related Topics

- [[Pose Graph Optimization]]
- [[Bundle Adjustment]]
- [[ORB-SLAM]]
- [[Visual Odometry]]
- [[SLAM]]
- [[Ceres]]
- [[GTSAM]]

---

## 🌐 External References

- [g2o GitHub Repository](https://github.com/RainerKuemmerle/g2o)
- [Original g2o Paper](https://www.openslam.org/g2o.html)

---
