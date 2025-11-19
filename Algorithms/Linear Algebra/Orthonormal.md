# Orthonormal

In mathematics and engineering, an **orthonormal** set refers to a collection of vectors that are both **orthogonal** (mutually perpendicular) and **normalized** (each has length 1). Orthonormal structures appear extensively in Reinforcement Learning, robotics, optimization, and numerical linear algebra because they simplify projections, decompositions, and transformations while reducing computational and numerical instability.

---

## 🧩 Overview

An orthonormal basis ensures that each vector is independent, perpendicular to the others, and scaled to unit length. This simplifies geometric reasoning and matrix operations. Many algorithms rely on orthonormality because it stabilizes computations and reduces redundancy. Examples include PCA, QR decomposition, SVD, Fourier transforms, and rotation matrices used in robotics and computer vision.

---

## 🧠 Core Concepts

- **Orthogonal vectors:** Inner product equals zero  
- **Normalized vectors:** Magnitude equals one  
- **Orthonormal basis:** Set of perpendicular, unit-length vectors spanning a vector space  
- **Inner product space:** Environment where orthogonality is defined  
- **Projection:** Simplifies greatly with orthonormal vectors  
- **Unitary/orthogonal matrices:** Matrices whose columns (or rows) form orthonormal sets  
- **Change of basis:** Orthonormal bases simplify coordinate transforms  

---

## 📊 Comparison Chart

| Concept | Meaning | Norm / Magnitude | Angle Requirements | Common Use Cases |
|--------|---------|------------------|--------------------|------------------|
| Orthogonal | Vectors are perpendicular | Any | 90° | PCA, decorrelation |
| Normalized | Vectors have length 1 | 1 | Any | Unit vectors, directions |
| Orthonormal | Orthogonal + normalized | 1 | 90° | Rotations, SVD, QR |
| Orthogonal Matrix | Square matrix with orthonormal columns | 1 per column | 90° | Robotics transforms |
| Unitary Matrix | Complex version of orthogonal matrix | 1 | 90° | Quantum computing |

---

## ⚙️ How It Works

Orthonormal vectors satisfy two constraints:

- **Orthogonality:** The dot product `vᵢ · vⱼ = 0` for all `i ≠ j`  
- **Normalization:** The norm `‖vᵢ‖ = 1`  

Together, they form the basis for many mathematically efficient operations. When a matrix has orthonormal columns, its transpose equals its inverse, i.e., `Qᵀ = Q⁻¹`. This simplifies solving systems, optimizing RL value estimations, and decomposing matrices in control systems.

---

## 🧠 Use Cases

- State-space representations in control theory  
- Orthogonal transformations in robotics kinematics  
- Whitening input features for RL agents  
- QR decomposition in least-squares solutions  
- Stable parameterizations of rotation matrices  
- SVD-based dimensionality reduction  
- Embedding spaces in machine learning  
- Spectral methods in policy evaluation  

---

## 🌟 Strengths

- Reduces numerical instability  
- Simplifies matrix inversion  
- Efficient coordinate transforms  
- Minimizes redundancy and correlation  
- Provides stable decompositions (QR, SVD)

---

## ⚠️ Weaknesses

- Requires careful construction to maintain orthonormality  
- Loss of orthonormality under floating-point drift  
- Gram–Schmidt can be unstable without modifications  
- Not always the best basis for all tasks  

---

## 🔧 Variants and Related Structures

- **Orthonormal basis**  
- **Orthonormal frames**  
- **Orthonormal matrices (orthogonal/unitary)**  
- **Orthonormal polynomials**  
- **ONB (Orthonormal Basis)** used in Hilbert spaces  

---

## 🔍 Related Concepts / Notes

- [[Linear Algebra]]  
- [[QR Decomposition]]  
- [[SVD]]  
- [[Eigenvectors]]  
- [[Rotation Matrices]]  
- [[State Space Models]]  
- [[Feature Normalization]]  
- [[Orthogonality]]  
- [[Vector Norms]]  
- [[Inner Product]]

---

## 🔌 Compatible Items

- Euclidean spaces  
- Hilbert spaces  
- Rotation groups (SO(2), SO(3))  
- Orthogonal matrices  
- Fourier bases  
- Wavelets  

---

## 📚 External Resources

- Gilbert Strang’s Linear Algebra lectures  
- Numerical Linear Algebra textbooks  
- MIT OpenCourseWare: Orthogonality units  
- Wikipedia: Orthonormality  
- “Matrix Computations” (Golub & Van Loan)

---

## 📝 Summary

Orthonormal vectors create a numerically stable, clean, and efficient mathematical basis that simplifies projections, transforms, and decompositions critical to reinforcement learning, robotics, and numerical algorithms. They underpin many of the most widely used algorithms in engineering and scientific computing.
