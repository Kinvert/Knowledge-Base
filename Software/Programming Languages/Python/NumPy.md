# NumPy

**NumPy** (Numerical Python) is a foundational library in the Python scientific computing ecosystem. It provides efficient array operations, numerical routines, and tools for integrating C/C++ and Fortran code. In robotics and engineering, it is indispensable for data analysis, matrix computations, and vectorized operations.

---

## 📚 Overview

NumPy offers a powerful `ndarray` object with broadcasting, slicing, and a suite of mathematical and statistical functions. It replaces native Python lists with more compact, performant arrays and allows for operations across entire arrays without explicit loops.

In robotics, NumPy is commonly used for sensor data processing, transformations, kinematics, and simulation. Many high-level libraries like [[OpenCV]], [[SciPy]], [[PyTorch]], and other Python tooling rely on NumPy arrays.

---

## 🧠 Core Concepts

- **[[ndarray]]**: A multi-dimensional array object  
- **Vectorization**: Replace loops with fast, element-wise operations  
- **Broadcasting**: Automatic expansion of arrays for operations  
- **Slicing**: Efficient access and manipulation of subarrays  
- **Linear Algebra**: Matrix multiplication, decomposition, and solving systems  
- **uFuncs**: Universal functions operating element-wise on arrays  

---

## 🧰 Use Cases

- Matrix math for SLAM and kinematics  
- Fast operations on sensor data streams  
- Image filtering and convolution  
- Signal processing and FFT  
- Geometric transformations  
- Point cloud preprocessing  

---

## ✅ Pros

- Extremely fast for numerical operations  
- Replaces many for-loops with vectorized calls  
- Interoperable with C, C++, and Fortran  
- Backbone of many scientific Python packages  
- Works with memory-mapped files and shared memory  

---

## ❌ Cons

- Not GPU-accelerated (see [[CuPy]] or [[PyTorch]] for GPU support)  
- Static typing can be verbose or non-intuitive  
- Debugging broadcasted operations can be tricky  
- Limited support for sparse arrays (use [[SciPy]] for this)  

---

## 📊 Comparison Table

| Feature               | NumPy       | PyTorch     | MATLAB      | SciPy        | CuPy         |
|-----------------------|-------------|-------------|-------------|--------------|--------------|
| Backend               | CPU         | CPU/GPU     | CPU         | CPU          | GPU          |
| Array API             | Yes         | Yes         | Yes         | Built on NumPy| Yes         |
| Broadcasting          | Yes         | Yes         | Partial     | Yes           | Yes         |
| Sparse Matrix Support | No          | Limited     | Yes         | Yes           | No          |
| Robotics Usage        | Very High   | High        | Moderate    | High          | Moderate     |

---

## 🤖 In a Robotics Context

| Application              | Role of NumPy                             |
|--------------------------|-------------------------------------------|
| Sensor Processing        | Fast matrix ops on incoming data  
| SLAM                     | Vector math for transformations  
| Kinematics               | Homogeneous transforms and Jacobians  
| Simulation               | Physics and sensor modeling  
| Control Systems          | Matrix-based state-space modeling  
| Point Cloud              | Fast slicing, thresholding, filtering  

---

## 🔧 Common Functions

- `np.array()`, `np.zeros()`, `np.ones()` – Array creation  
- `np.dot()`, `np.matmul()` – Matrix multiplication  
- `np.linalg.inv()`, `np.linalg.svd()` – Linear algebra  
- `np.mean()`, `np.std()` – Statistics  
- `np.fft.fft()` – Fast Fourier Transform  
- `np.where()`, `np.clip()`, `np.argmax()` – Element-wise operations  

---

## 🔧 Compatible Items

- [[Python]] – Native integration and syntax  
- [[Cython]] – Can wrap `ndarray` with typed memory views  
- [[OpenCV]] – Images as NumPy arrays  
- [[SciPy]] – Extends NumPy for optimization, integration, etc.  
- [[TensorFlow]] / [[PyTorch]] – Accept or convert NumPy arrays  
- [[Pandas]] – Built on NumPy for tabular data  

---

## 🔗 Related Concepts

- [[SciPy]] (Scientific functions built on top of NumPy)  
- [[Cython]] (Use NumPy arrays for fast numerical extensions)  
- [[OpenCV]] (Images as NumPy arrays)  
- [[Matrix Multiplication]] (Core operation in NumPy)  
- [[Python]] (Language NumPy is written for)  

---

## 📚 Further Reading

- [NumPy Official Website](https://numpy.org/)  
- [NumPy User Guide](https://numpy.org/doc/stable/user/)  
- [NumPy for MATLAB Users](https://numpy.org/doc/stable/user/numpy-for-matlab-users.html)  
- [Real Python – NumPy Tutorial](https://realpython.com/numpy-array-programming/)  
- [Scipy Lecture Notes – NumPy](https://scipy-lectures.org/intro/numpy/)

---
