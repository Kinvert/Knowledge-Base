# Logarithmic Map

The logarithmic map is a mathematical function that maps elements from a Lie Group to its corresponding Lie Algebra (the tangent space at the identity). It is the inverse of the exponential map and is widely used in robotics for tasks such as computing residuals between poses, linearizing motion models, and performing optimization on manifolds.

---

## 📚 Overview

In robotics and computer vision, the logarithmic map allows a transformation (e.g., a rotation matrix or rigid-body pose) to be converted into a minimal vector representation in the tangent space. This enables linear algebra tools to be used when optimizing or comparing transformations.

---

## 🧠 Core Concepts

- **Lie Group → Lie Algebra**: The logarithmic map projects a transformation back to a vector in the tangent space.
- **Twist Representation**: For SE(3), the log map returns a 6D vector (3 rotation + 3 translation).
- **Matrix Logarithm**: The general form involves computing a matrix logarithm, but practical Lie groups (SO(3), SE(3)) often have closed-form solutions.

---

## 🧰 Use Cases

- Calculating the difference (residual) between two poses
- Error-state estimation in filters (e.g., EKF on manifolds)
- Linearizing transformations for graph optimization
- IMU bias correction and integration
- Local tangent space updates in bundle adjustment

---

## ✅ Pros

- Enables optimization in minimal coordinates
- Converts group differences into additive updates
- Essential for manifold-aware estimation and control

---

## ❌ Cons

- Requires specialized math for stable implementation
- Less intuitive than vector subtraction in ℝⁿ
- Edge cases (e.g., small-angle approximation) need careful handling

---

## 📊 Comparison Chart

| Map Type         | Direction            | Used In                   | Groups       | Notes                                 |
|------------------|----------------------|----------------------------|--------------|----------------------------------------|
| **Logarithmic**  | Group → Lie Algebra  | Residuals, optimization    | SO(3), SE(3) | Converts pose into minimal vector form |
| **Exponential**  | Lie Algebra → Group  | Pose updates               | SO(3), SE(3) | Converts velocity into transform       |
| **Angle-Axis**   | SO(3) → ℝ³           | Rotation residuals         | SO(3)        | Simplified SO(3) logarithm             |
| **Twist Vector** | SE(3) → ℝ⁶           | Full pose residual         | SE(3)        | Rotation + translation in one vector   |

---

## 🔧 Compatible Items

- [[Sophus]] (`log()` for SE(3), SO(3))
- [[g2o]] (Computes pose error using logarithmic map)
- [[Ceres Solver]] (Custom manifolds using log/exp)
- [[IMU Preintegration]] (Uses log to compute state deltas)
- [[Pose Graph Optimization]] (Log map used for residual computation)

---

## 🔗 Related Concepts

- [[Lie Algebra]] (Target space of the log map)
- [[Lie Group]] (Input to the log map)
- [[Exponential Map]] (Inverse function)
- [[SE(3)]] (Log map gives minimal pose difference)
- [[SO(3)]] (Log map gives rotation vector)
- [[Manifolds]] (Non-linear spaces that require log/exp for updates)

---

## 🛠 Developer Tools

- `Sophus::SE3::log()` and `SO3::log()` (C++)
- `manif::SE3::log()` (C++ modern library)
- `scipy.linalg.logm()` for generic matrix logarithms
- `Eigen` (requires custom implementation or use of libraries above)

---

## 📚 Further Reading

- Barfoot, T. D. (2017). *State Estimation for Robotics*
- Sola, J. (2018). *Lie Groups for 2D and 3D Transformations*
- Forster et al. (2017). *IMU Preintegration on Manifold*

---
