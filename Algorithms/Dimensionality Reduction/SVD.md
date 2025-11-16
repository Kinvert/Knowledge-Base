# SVD (Singular Value Decomposition)

Singular Value Decomposition (SVD) is a fundamental matrix factorization technique widely used in robotics, control theory, machine learning, computer vision, and numerical optimization. It decomposes a matrix into orthogonal bases and singular values, revealing deep geometric and structural information. SVD is crucial for tasks such as pose estimation, noise reduction, dimensionality reduction, and solving ill-conditioned linear systems.

---

## ⚙️ Overview

SVD expresses any real matrix **A** as the product `U Σ Vᵀ`, where **U** and **V** are orthogonal matrices and **Σ** is a diagonal matrix of singular values. These singular values measure the intrinsic axes of variation in the data or transformation represented by **A**. Because SVD is numerically stable and mathematically robust, it underpins many algorithms in robotics and engineering.

---

## 🧠 Core Concepts

- **Left Singular Vectors (U)**  
  Orthonormal basis describing the range of **A**.

- **Right Singular Vectors (V)**  
  Orthonormal basis describing the domain of **A**.

- **Singular Values (Σ)**  
  Sorted nonnegative scalars giving the "strength" or importance of each principal direction.

- **Rank Approximation**  
  Truncating small singular values produces low-rank approximations (key in PCA and noise reduction).

- **Condition Number**  
  Ratio of largest to smallest singular value; important for assessing numerical stability.

---

## 📊 Comparison Chart

| Technique | Purpose | Relation to SVD | Strengths | Weaknesses |
|----------|---------|----------------|-----------|------------|
| **SVD** | General matrix factorization | Baseline | Very stable, widely applicable | Computationally heavy for large matrices |
| **PCA** | Dimensionality reduction | Uses SVD internally | Interpretable PCs | Requires centering data |
| **QR Decomposition** | Orthogonal + triangular factorization | Simpler alternative | Fast | Less expressive than SVD |
| **Eigendecomposition** | Decomposition of square matrices | Related but requires square matrices | Insightful for symmetric matrices | Not as general as SVD |
| **LU Decomposition** | Triangular factorization | Different goals (solving linear systems) | Efficient | Fails for some matrices |
| **Non-negative Matrix Factorization (NMF)** | Parts-based decomposition | Conceptually different | Interpretability | Constraints limit applicability |

---

## 🔧 Use Cases

- Robot kinematics (e.g., computing pseudoinverses for redundant manipulators)  
- Solving least squares problems including control and estimation pipelines  
- [[PCA]] (Principal Component Analysis) via truncated SVD  
- Vision-based tasks: feature compression, background subtraction, epipolar geometry  
- Noise filtering and signal denoising  
- Shape analysis and 3D reconstruction  
- Finding rank or determining degeneracies in sensor matrices  
- Solving systems with ill-conditioned Jacobians or Hessians

---

## 🏆 Strengths

- Works for **any** matrix (square or rectangular)  
- Numerically stable and robust  
- Provides deep geometric insight  
- Excellent for diagnostics and system understanding  
- Enables powerful low-rank approximations

---

## ⚠️ Weaknesses

- Computationally expensive for very large matrices  
- Not as efficient as problem-specific factorizations (QR, LU)  
- Interpretation requires linear algebra familiarity  
- Full SVD may be overkill when only a few singular values are needed

---

## 🔩 Compatible Items

- [[PCA]]  
- [[Kalman Filter]] (uses observation/jacobian matrices where SVD helps diagnose rank issues)  
- [[Pose Estimation]]  
- [[Matrix Inverse]] and [[Pseudo-Inverse]]  
- [[QR Decomposition]]  
- [[Eigendecomposition]]  
- [[Jacobian]] and [[Kinematics]]  
- [[SLAM]] (observability and linear algebra backends)

---

## 🧱 How It Works

SVD can be conceptually understood in three steps:
1. Determine orthogonal basis vectors that span the input and output spaces.  
2. Scale these basis vectors by singular values.  
3. Rotate into the final orthogonal basis.

This reveals that any matrix is simply: rotate → scale → rotate.

---

## 🗂️ Key Features

- Produces orthonormal bases for domain and range  
- Provides singular values sorted by importance  
- Enables stable pseudoinverses  
- Reveals matrix rank and condition number  
- Supports truncated low-rank approximations  
- Works even when matrices are not full rank or not square

---

## 📚 Related Concepts / Notes

- [[PCA]] (Principal Component Analysis)  
- [[Pseudo-Inverse]] (Moore-Penrose)  
- [[QR Decomposition]]  
- [[Kinematics]]  
- [[SLAM]]  
- [[Noise Filtering]]  
- [[Least Squares]]  
- [[Jacobian]]  
- [[Numerical Stability]]  
- [[Dimensionality Reduction]]

---

## 🌐 External Resources

- Gilbert Strang – Linear Algebra lectures  
- Numerical Linear Algebra textbooks (Trefethen & Bau)  
- LAPACK and BLAS documentation  
- Robotics algorithms by Lynch & Park  
- Computer Vision: A Modern Approach

---

## 📝 Summary

SVD is one of the most powerful tools in applied mathematics, providing a complete and stable decomposition of any matrix into orthogonal bases and intrinsic importance weights. Its applications permeate robotics, computer vision, and control systems, where it aids in solving ill-posed problems, extracting dominant patterns, and improving numerical stability. Understanding SVD is essential for engineers building robust robotic systems and high-performance algorithms.
