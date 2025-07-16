# SciPy

**SciPy** is an open-source Python-based ecosystem for scientific and technical computing. It builds on top of [[NumPy]] and provides a vast array of high-level algorithms and functions for numerical integration, optimization, signal processing, linear algebra, statistics, and more.

SciPy is widely used in academia, engineering, data science, and robotics for prototyping, modeling, and solving real-world mathematical problems.

---

## 🧠 Overview

SciPy is organized into submodules that target specific domains of scientific computation. These include:
- `scipy.linalg` – linear algebra
- `scipy.integrate` – numerical integration
- `scipy.optimize` – optimization routines
- `scipy.signal` – signal processing
- `scipy.sparse` – sparse matrices and solvers
- `scipy.fft` – Fourier transforms
- `scipy.stats` – statistical functions and distributions

SciPy functions are often thin wrappers over compiled libraries like [[OpenBLAS]], [[LAPACK]], and [[FFTW]], offering both speed and ease of use.

---

## 🧪 Use Cases

- Signal filtering and analysis for robotics sensors  
- PID tuning and control optimization  
- Solving differential equations in simulations  
- Sparse matrix solvers for FEM and CFD  
- Data fitting, curve regression, and interpolation  
- Scientific experimentation and prototyping  
- Probabilistic modeling and hypothesis testing

---

## ⚙️ Capabilities

- High-level interface for numerical methods  
- Dense and sparse matrix operations  
- Numerical integration (e.g., Runge-Kutta, quadrature)  
- Curve fitting and root finding  
- FFT and filtering for time-series data  
- Statistics and distributions  
- Works seamlessly with [[NumPy]] arrays

---

## 📊 Comparison Table

| Library        | Language | Focus Area              | GPU Support | Notes                                       |
|----------------|----------|--------------------------|-------------|---------------------------------------------|
| SciPy          | Python   | Scientific computation   | ❌           | Built on top of NumPy, uses CPU backends    |
| [[NumPy]]      | Python   | Base nD array & math     | ❌           | Foundational, used by SciPy                 |
| [[CuPy]]       | Python   | NumPy-compatible on GPU  | ✅           | GPU-accelerated NumPy replacement           |
| [[SymPy]]      | Python   | Symbolic math            | ❌           | Algebra, calculus, symbolic manipulation    |
| [[MATLAB]]     | Proprietary | Scientific/Engineering | 🟡           | Similar capabilities, proprietary           |

---

## ✅ Pros

- Broad coverage of scientific computation needs  
- Pythonic, intuitive API  
- Built on top of optimized C/Fortran libraries  
- Large and active community  
- Well-documented with many examples

---

## ❌ Cons

- CPU only — no GPU acceleration  
- Some functions can be slower than hand-tuned C/C++  
- Large dependency (not ideal for microcontrollers or embedded)  
- Sparse support for differentiable programming

---

## 🔗 Related Concepts

- [[NumPy]]  
- [[OpenBLAS]]  
- [[LAPACK]]  
- [[CuPy]]  
- [[JAX]]  
- [[SymPy]]  
- [[Linear Algebra]]  
- [[Signal Processing]]  
- [[Optimization Algorithms]]  
- [[Reinforcement Learning]]  
- [[Control Theory]]  
- [[SciPy Optimization]]  
- [[SciPy Signal]]

---

## 📚 Further Reading

- [SciPy.org](https://scipy.org)  
- [SciPy Documentation](https://docs.scipy.org/doc/scipy/)  
- [SciPy GitHub](https://github.com/scipy/scipy)  
- [SciPy Tutorial](https://docs.scipy.org/doc/scipy/tutorial/)

---
