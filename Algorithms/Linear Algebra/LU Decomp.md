# LU Decomp

LU Decomposition is a fundamental matrix factorization technique that expresses a square matrix as the product of a lower triangular matrix (L) and an upper triangular matrix (U). This method is widely used in numerical linear algebra for solving systems of equations, inverting matrices, and computing determinants. It is a core building block for many algorithms used in control theory, robotics, simulation, and more.

---

## 🧠 Overview

LU Decomposition transforms a matrix `A` into two matrices `L` and `U` such that `A = LU`, where:
- `L` is a lower triangular matrix (with ones on the diagonal if using Doolittle’s method),
- `U` is an upper triangular matrix.

This method simplifies complex matrix operations by reducing them to operations on triangular matrices, which are computationally more efficient to handle.

---

## 📘 Core Concepts

- **Triangular Matrices:** Matrices with zero elements either above or below the main diagonal.
- **Forward and Backward Substitution:** Methods used to solve linear systems once `L` and `U` are known.
- **Pivoting:** A technique used to improve numerical stability, often yielding LUP decomposition.
- **Determinant Calculation:** The determinant of `A` can be found as the product of the diagonals of `U`.

---

## 🧪 Use Cases

- Solving linear systems `Ax = b`
- Matrix inversion
- Computing matrix determinants
- Kalman filters and state estimation
- Simulation and control models in robotics

---

## 📊 Comparison Chart

| Method               | Matrix Type      | Pivoting Support | Use in Robotics | Speed       | Notes                              |
|----------------------|------------------|------------------|------------------|-------------|------------------------------------|
| LU Decomp            | Square           | Optional          | ✅                | Fast        | Efficient for dense systems        |
| [[Cholesky Decomp]]  | SPD only         | ❌                | ✅                | Very fast   | More efficient when applicable     |
| [[QR Decomp]]        | Any (incl. rectangular) | ❌       | ✅                | Slower      | More stable, useful for least-squares |
| [[SVD]] (Singular Value Decomp) | Any     | ❌                | ✅                | Slowest     | Robust, handles rank deficiency    |
| [[Gauss Elimination]]| Square           | Optional          | ✅                | Moderate    | Basis of LU; less modular          |

---

## ✅ Strengths

- Computationally efficient
- Widely implemented and supported
- Forms basis for other decomposition algorithms
- Works well for large dense matrices

---

## ⚠️ Weaknesses

- Not suitable for all matrix types (e.g., singular or non-square)
- Can be numerically unstable without pivoting
- Less robust than [[SVD]] in ill-conditioned systems

---

## 🔗 Related Concepts

- [[Cholesky Decomp]] (for symmetric positive definite matrices)
- [[QR Decomp]] (for solving least squares problems)
- [[SVD]] (Singular Value Decomposition)
- [[Kalman Filter]] (uses matrix decompositions in prediction-update cycle)
- [[Numerical Linear Algebra]]
- [[Control Theory]]

---

## 🔧 Compatible Items

- Libraries: `Eigen`, `SciPy`, `NumPy`, `MATLAB`, `LAPACK`
- [[Kalman Filter]] implementations
- [[Robot Kinematics]] solvers
- [[State Estimation]] frameworks

---

## 🌐 External Resources

- [Wikipedia: LU Decomposition](https://en.wikipedia.org/wiki/LU_decomposition)
- [Matrix Computations (Golub & Van Loan)](https://www.cs.cornell.edu/~golub/)
- [SciPy LU Documentation](https://docs.scipy.org/doc/scipy/reference/generated/scipy.linalg.lu.html)

---

## 📚 Further Reading

- Gilbert Strang – *Linear Algebra and Its Applications*
- Trefethen & Bau – *Numerical Linear Algebra*
- Golub & Van Loan – *Matrix Computations*

---
