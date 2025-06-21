# 🔗 Factor Graph

A **Factor Graph** is a type of bipartite graphical model that represents the factorization of a function (typically a probability distribution or cost function). It consists of **variable nodes** and **factor nodes**. Variable nodes represent unknown quantities, and factor nodes represent constraints or relationships among variables.

---

## 🧠 Summary

- **Structure**: Bipartite graph with variable nodes and factor nodes
- **Purpose**: Encode how a global function (e.g., joint probability or cost) decomposes into local functions (factors)
- **Applications**: Probabilistic inference, optimization, SLAM, error correction codes

---

## 🎯 Factor Graphs in SLAM

In the context of **Simultaneous Localization and Mapping (SLAM)**, factor graphs provide a compact and flexible way to represent the estimation problem. They capture the relationships (constraints) between robot poses, landmarks, and measurements.

- **Variable nodes**: Robot poses, landmark positions
- **Factor nodes**: Sensor measurements, odometry constraints, prior knowledge
- **Optimization goal**: Find variable assignments (poses, map) that best satisfy all factors (i.e., minimize error or maximize likelihood)

Factor graphs are the foundation of **graph-based SLAM**, where solving SLAM reduces to solving a sparse nonlinear least squares problem.

---

## ⚙️ How Factor Graph SLAM Works

1️⃣ **Pose Nodes**: Each robot pose at a given timestep is a variable node.

2️⃣ **Landmark Nodes**: Each map feature (e.g., a landmark) is a variable node.

3️⃣ **Factors**: Constraints such as odometry measurements, GPS updates, loop closures, or landmark observations.

4️⃣ **Graph Construction**: As the robot moves and observes, new nodes and factors are added.

5️⃣ **Optimization**: Solve for the configuration of variables that best satisfies all constraints using solvers like Gauss-Newton or Levenberg-Marquardt.

---

## 🛠️ SLAM Libraries Using Factor Graphs

| Library          | Notes |
|------------------|-------|
| **GTSAM**        | Widely used C++ library for factor graph SLAM |
| **g2o**          | General framework for graph optimization |
| **Ceres Solver** | Can represent and solve factor graphs indirectly |
| **Cartographer** | Uses factor-graph-like backend for pose graph SLAM |

---

## 📊 Comparison: Factor Graph SLAM vs EKF SLAM

| Feature              | Factor Graph SLAM | EKF SLAM |
|----------------------|------------------|----------|
| Scalability          | ✅ Better for large maps | ❌ Scales poorly |
| Flexibility          | ✅ Easily integrate new constraints | ⚠️ Rigid model |
| Computational Cost   | ⚠️ High upfront (batch) | ✅ Incremental update |
| Loop Closure Handling | ✅ Robust | ❌ Can degrade accuracy |

---

## ✅ Strengths

- Sparse structure ⇒ efficient storage and computation
- Naturally handles nonlinear constraints
- Easy to add new types of measurements or constraints
- Supports batch and incremental solvers (e.g., iSAM)

---

## ❌ Weaknesses

- Solving can be computationally intensive in large-scale, real-time applications without incremental solvers
- Requires careful graph construction to ensure stability

---

## 🔗 Related Topics

- [[SLAM]]
- [[Monocular SLAM]]
- [[Binocular SLAM]]
- [[Pose Graph]]
- [[Odometry]]
- [[GTSAM]]
- [[g2o]]

---

## 🌐 External References

- [Wikipedia: Factor graph](https://en.wikipedia.org/wiki/Factor_graph)
- [GTSAM](https://gtsam.org/)
- [g2o](https://github.com/RainerKuemmerle/g2o)

---
