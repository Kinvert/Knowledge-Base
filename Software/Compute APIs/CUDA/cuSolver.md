# cuSolver

**cuSolver** is a GPU-accelerated library from NVIDIA that provides high-performance **dense and sparse linear solvers** as part of the [[CUDA Toolkit]]. It supports various factorizations and solvers for systems of linear equations, eigenvalue problems, and least-squares minimization—all optimized for execution on NVIDIA GPUs.

cuSolver is a key component in GPU-accelerated scientific computing, simulations, and machine learning workflows where linear algebra operations are a bottleneck.

---

## 🧠 Overview

cuSolver includes three main modules:

- **cuSolverDN** (Dense): LU, QR, Cholesky, SVD, and eigenvalue decompositions.
- **cuSolverSP** (Sparse): Solvers for sparse linear systems using CSR matrices.
- **cuSolverRF** (Refactorization): For matrix factorization reuse in iterative solvers.

These APIs offer performance comparable to CPU-based libraries like [[LAPACK]] and [[OpenBLAS]]—but on GPUs—making them highly useful in large-scale robotics, FEM, and deep learning backends.

---

## 🧪 Use Cases

- Solving linear systems in [[Finite Element]] methods (FEM)  
- Accelerated QR/SVD for [[Machine Learning]] and [[Dimensionality Reduction]]  
- Eigenvalue decomposition in control systems or signal processing  
- Robotics simulations involving dynamic systems  
- Inversion and decomposition of large matrices in parallel

---

## ⚙️ Capabilities

- GPU-accelerated matrix decompositions  
- Solvers for dense and sparse systems  
- Batched operations for parallelism  
- Seamless integration with [[cuBLAS]] and [[cuSPARSE]]  
- Works with CUDA Unified Memory or explicit memory management  
- Bindings via [[PyCUDA]] and [[CuPy]] (partial)

---

## 📊 Comparison Table

| Library         | CPU/GPU | Dense | Sparse | Language | Notes                                  |
|------------------|--------|-------|--------|----------|----------------------------------------|
| cuSolver         | GPU    | ✅     | ✅      | CUDA     | Best for NVIDIA GPU dense/sparse solvers |
| [[cuBLAS]]       | GPU    | ✅     | ❌      | CUDA     | GPU-accelerated BLAS (low-level)        |
| [[cuSPARSE]]     | GPU    | ❌     | ✅      | CUDA     | GPU sparse matrix routines              |
| [[OpenBLAS]]     | CPU    | ✅     | ❌      | C        | Fast CPU dense linear algebra           |
| [[SciPy]]        | CPU    | ✅     | ✅      | Python   | Easy interface but CPU-only             |
| [[MAGMA]]        | GPU    | ✅     | ✅      | CUDA     | Alternative to cuSolver with CPU fallback|

---

## ✅ Pros

- High performance on large matrix problems  
- Fully GPU-accelerated, ideal for robotics or ML at scale  
- Covers both dense and sparse systems  
- Batched operations for throughput  
- Backed and maintained by NVIDIA

---

## ❌ Cons

- Tied to NVIDIA GPUs only  
- Requires knowledge of CUDA programming  
- Limited Python support (not as easy as [[SciPy]] or [[NumPy]])  
- Can be overkill for small or simple systems

---

## 🔗 Related Concepts

- [[cuBLAS]]  
- [[cuSPARSE]]  
- [[CUDA Toolkit]]  
- [[MAGMA]]  
- [[PyCUDA]]  
- [[CuPy]]  
- [[Matrix Multiplication]]  
- [[Linear Algebra]]  
- [[Finite Element]]  
- [[Control Theory]]

---

## 📚 Further Reading

- [cuSolver Documentation (NVIDIA)](https://docs.nvidia.com/cuda/cusolver/index.html)  
- [cuSolver GitHub Samples](https://github.com/NVIDIA/cuda-samples/tree/master/Samples)  
- [CUDA Toolkit Downloads](https://developer.nvidia.com/cuda-downloads)  
- cuSolver in Python: [CuPy Linear Algebra](https://docs.cupy.dev/en/stable/reference/linalg.html)

---
