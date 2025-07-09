# Lie Group

Lie Groups are continuous groups that are also differentiable manifolds, enabling smooth mathematical operations like interpolation and optimization. In robotics and computer vision, Lie Groups are fundamental for representing 2D and 3D transformations—such as rotations and poses—with precision and mathematical consistency.

---

## 📚 Overview

Lie Groups combine algebraic and geometric structures, making them ideal for modeling rigid-body transformations. Popular examples include SO(2), SO(3) for rotation, and SE(2), SE(3) for pose (rotation + translation). These groups enable efficient composition, inversion, and interpolation of transformations in a non-Euclidean space.

---

## 🧠 Core Concepts

- **Group**: A set with associative composition, an identity, and inverses.
- **Manifold**: A smooth space that locally resembles Euclidean space.
- **Lie Algebra**: The tangent space at the identity element; allows linear operations.
- **Exponential / Logarithmic Maps**: Used to move between Lie Group and Lie Algebra.
- **Composition / Inversion**: Efficient, closed-form transformation chaining.

---

## 🔧 Common Lie Groups in Robotics

- **SO(2)**: 2D rotation
- **SE(2)**: 2D rotation + translation
- **SO(3)**: 3D rotation
- **SE(3)**: 3D rigid-body transformation (rotation + translation)

---

## 🧰 Use Cases

- Robot localization and mapping (SLAM)
- Visual odometry and sensor fusion
- Manipulator kinematics
- Inertial navigation
- Trajectory interpolation and motion planning

---

## ✅ Strengths

- Mathematically elegant and precise
- Avoids singularities and redundancies in pose representation
- Supports efficient optimization on manifolds

---

## ❌ Weaknesses

- Requires specialized knowledge and tooling
- Non-intuitive for beginners
- Some operations (like mean or averaging) are non-trivial

---

## 📊 Comparison Chart

| Group  | Dimension | Rotation | Translation | Used In     | Notes                           |
|--------|-----------|----------|-------------|-------------|----------------------------------|
| SO(2)  | 1         | ✅ Yes   | ❌ No       | 2D SLAM     | Angle in radians                |
| SE(2)  | 3         | ✅ Yes   | ✅ Yes      | 2D Robots   | Compact for planar motion       |
| SO(3)  | 3         | ✅ Yes   | ❌ No       | 3D Rotation | Typically represented by matrix or quaternion |
| SE(3)  | 6         | ✅ Yes   | ✅ Yes      | SLAM, VO    | Full 6-DOF transformation        |
| ℝⁿ     | n         | ❌ No    | ✅ Yes      | Flat spaces | Not a Lie Group (no rotation)   |

---

## 🔧 Compatible Items

- [[Sophus]] (Efficient C++ implementation of SE(2), SO(3), SE(3), etc.)
- [[g2o]] (Pose graph optimization using SE(3))
- [[Ceres Solver]] (Supports Lie groups via local parameterizations)
- [[IMU Preintegration]] (Performs updates on SE(3))
- [[Kalman Filter]] (Can use Lie group extensions)

---

## 🔗 Related Concepts

- [[Lie Algebra]] (Tangent space at the identity of a Lie Group)
- [[SE(3)]] (Special Euclidean group in 3D)
- [[SO(3)]] (Special Orthogonal group in 3D)
- [[Pose Graph Optimization]] (Often uses Lie Group math)
- [[Manifolds]] (Lie Groups are a special case of these)
- [[Exponential Map]] (Group ← Algebra)
- [[Logarithmic Map]] (Algebra ← Group)

---

## 🛠 Developer Tools

- `Sophus::SE3`, `Sophus::SO3` in Sophus (C++)
- `manif` (Modern C++ Lie group library)
- `Eigen` (Basic matrix support, needs extensions for full Lie group operations)
- Python `liegroups` or `manifpy` (experimental)

---

## 📚 Further Reading

- Barfoot, T. D. (2017). *State Estimation for Robotics*
- Sola, J. (2018). *Lie Groups for 2D and 3D Transformations*
- Forster et al. (2017). *IMU Preintegration on Manifold*

---
