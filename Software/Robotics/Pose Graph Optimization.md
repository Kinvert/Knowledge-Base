# 📍 Pose Graph Optimization

**Pose Graph Optimization (PGO)** is a core technique in **Simultaneous Localization and Mapping ([[SLAM]])**, used to refine robot or camera trajectories by minimizing the global error across a graph of poses and constraints. It is particularly common in **Graph-Based SLAM**, where the robot's trajectory and map are represented as a graph.

---

## 🧠 Summary

- Models robot poses as graph nodes.
- Models spatial constraints (e.g., odometry, loop closures) as graph edges.
- Uses nonlinear optimization (e.g., Gauss-Newton, Levenberg-Marquardt) to find the most likely configuration of poses.
- Widely used in back-end SLAM systems.

---

## 🔧 Graph Structure

- **Nodes**: Robot or camera poses (often 2D/3D poses like [[SE2]] or [[SE3]])
- **Edges**: Spatial constraints, typically relative pose measurements with associated uncertainty (covariance)
- **Error Function**: Sum of residuals between measured and estimated constraints

---

## 🔄 Optimization Techniques

| Algorithm              | Description                                                                 |
|------------------------|-----------------------------------------------------------------------------|
| Gauss-Newton           | Fast convergence for near-optimal initial guesses                          |
| Levenberg-Marquardt    | Blends gradient descent with Gauss-Newton for robustness                   |
| Dogleg                 | Intermediate technique for improved stability                              |
| Incremental Solvers    | Used in real-time applications (e.g., iSAM in [[GTSAM]])                   |

---

## 🔥 Common Libraries

| Library    | Language | Notes |
|------------|----------|-------|
| [[g2o]]    | C++      | Lightweight, used in [[ORB-SLAM2]], customizable cost functions |
| [[GTSAM]]  | C++      | Factor graph based, supports symbolic and numerical optimization |
| [[Ceres Solver]] | C++ | General optimization library with automatic differentiation |
| [[Sophus]]  | C++      | Often used alongside g2o or GTSAM to represent poses on manifolds |

---

## 📈 Applications

- Loop closure correction in SLAM
- Bundle adjustment in structure-from-motion
- Optimizing robot trajectories with noisy odometry
- Multi-sensor fusion (e.g., LiDAR + IMU)

---

## ✅ Advantages

- Provides global consistency across trajectory and map
- Robust to drift in odometry or local SLAM estimates
- Can incorporate loop closures and GPS constraints

---

## ⚠️ Challenges

- Computationally expensive for large graphs
- Requires good initialization (e.g., from front-end SLAM)
- Outlier loop closures can severely affect results

---

## 🔗 Related Topics

- [[Graph-Based SLAM]]
- [[Factor Graph]]
- [[Loop Closure]]
- [[Bundle Adjustment]]
- [[g2o]]
- [[GTSAM]]
- [[Sophus]]
- [[Ceres Solver]]

---

## 🌐 External Links

- [Graph-Based SLAM Overview (Thrun et al.)](https://ais.informatik.uni-freiburg.de/publications/papers/grisetti-tro10.pdf)
- [GTSAM Documentation](https://gtsam.org/)
- [g2o GitHub](https://github.com/RainerKuemmerle/g2o)

---
