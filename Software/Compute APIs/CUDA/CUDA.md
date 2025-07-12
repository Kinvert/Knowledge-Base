# ⚙️ CUDA (Compute Unified Device Architecture)

CUDA is a parallel computing platform and API model developed by NVIDIA. It provides developers direct access to the virtual instruction set and memory of NVIDIA GPUs. It’s primarily used for general-purpose computing on GPUs (GPGPU) and accelerates workloads by distributing them across thousands of cores.

---

## 🧾 Summary

- **Full Name**: Compute Unified Device Architecture
- **Developer**: NVIDIA
- **Released**: 2007
- **Purpose**: GPU-accelerated general-purpose computing
- **Primary Use**: Scientific computing, deep learning, real-time video processing, financial modeling, simulations, image processing

---

## 🎯 What It's Used For

CUDA is widely used in fields requiring heavy computation:
- Deep learning frameworks (TensorFlow, PyTorch)
- Physics and fluid simulations
- Molecular dynamics
- Video encoding/decoding
- Computer vision
- Financial analysis
- Medical imaging

---

## 🚫 What It's *Not* Used For

- Cross-vendor GPU programming (CUDA only works with NVIDIA GPUs)
- Embedded systems without NVIDIA hardware
- Standard graphics rendering (handled by OpenGL/Vulkan/DirectX)

---

## 🆚 Comparable APIs & Frameworks

| Framework/API | Vendor     | GPU Support   | Cross-Platform | Notes                                 |
|---------------|------------|----------------|----------------|---------------------------------------|
| CUDA          | NVIDIA     | NVIDIA only    | No             | Best performance on NVIDIA GPUs       |
| OpenCL        | Khronos    | Many vendors   | Yes            | More portable, less optimized         |
| ROCm (HIP)    | AMD        | AMD GPUs       | Yes            | CUDA-like experience for AMD          |
| SYCL          | Khronos    | Many via backends | Yes         | Modern C++ wrapper over OpenCL        |
| Vulkan Compute| Khronos    | Many vendors   | Yes            | Unified compute and graphics           |

---

## 🔬 How It Works

CUDA exposes the parallelism of NVIDIA GPUs by letting developers write kernels—functions that execute across many threads in parallel. The CPU (host) schedules work and allocates memory, while the GPU (device) runs the parallel code. CUDA abstracts thread hierarchies, shared memory, and warp scheduling to allow for high performance with explicit control.

Work is split into:
- **Threads** – smallest unit of execution
- **Thread blocks** – groups of threads that share resources
- **Grids** – collections of thread blocks that define the whole job

---

## 💡 Notable Libraries and Frameworks Built on CUDA

- **cuDNN** – Deep neural network primitives for TensorFlow, PyTorch
- **Thrust** – STL-like parallel algorithms
- **cuBLAS** – Linear algebra routines
- **cuFFT** – Fast Fourier Transform
- **eCAL** – Communication middleware using Protobufs, optionally accelerated
- **OpenCV** – Some modules have CUDA acceleration

---

## ✅ Strengths

- Industry-standard for NVIDIA GPU compute
- Deep integration with ML/DL libraries
- Mature, well-supported toolchain
- Best-in-class performance for supported hardware
- Extensive ecosystem (profilers, debuggers, libraries)

---

## ❌ Weaknesses

- Vendor lock-in: only runs on NVIDIA hardware
- Not portable to AMD, Intel, or ARM GPUs
- Requires a good grasp of GPU architecture for best performance
- No automatic parallelization (developer must manage workloads explicitly)

---

## 💻 Supported Languages & Platforms

- **Languages**: C, C++, Fortran (via PGI), Python (via PyCUDA, Numba), MATLAB (via toolbox)
- **Operating Systems**: Linux, Windows, macOS (limited), WSL2

---

## ⚠️ When CUDA Is Overkill

- Simple programs with no need for parallelism
- Projects requiring portability across GPU vendors
- Low-end embedded hardware without NVIDIA GPUs

---

## 📉 When CUDA Is Underkill

- For extreme HPC clusters, custom FPGAs or dedicated ASICs may outperform CUDA
- When tight integration with rendering pipelines (graphics) is needed, Vulkan Compute or similar may be more appropriate

---

## Code Snippets

### 🧑‍💻 C/C++ (Native CUDA)

```cpp
#include <cuda_runtime.h>
#include <iostream>

#define N 100

__global__ void matrixMul(float *A, float *B, float *C) {
    int row = threadIdx.y;
    int col = threadIdx.x;
    float sum = 0.0;

    for (int k = 0; k < N; ++k) {
        sum += A[row * N + k] * B[k * N + col];
    }

    C[row * N + col] = sum;
}

int main() {
    float A[N*N], B[N*N], C[N*N];
    float *d_A, *d_B, *d_C;

    cudaMalloc(&d_A, N*N*sizeof(float));
    cudaMalloc(&d_B, N*N*sizeof(float));
    cudaMalloc(&d_C, N*N*sizeof(float));

    // Fill A and B with 1.0 for simplicity
    for (int i = 0; i < N*N; ++i) {
        A[i] = 1.0f;
        B[i] = 1.0f;
    }

    cudaMemcpy(d_A, A, N*N*sizeof(float), cudaMemcpyHostToDevice);
    cudaMemcpy(d_B, B, N*N*sizeof(float), cudaMemcpyHostToDevice);

    dim3 threads(N, N);
    matrixMul<<<1, threads>>>(d_A, d_B, d_C);

    cudaMemcpy(C, d_C, N*N*sizeof(float), cudaMemcpyDeviceToHost);

    std::cout << "C[0][0] = " << C[0] << std::endl;

    cudaFree(d_A);
    cudaFree(d_B);
    cudaFree(d_C);

    return 0;
}
```
### 🐍 Python (Using Numba with CUDA)

```python
import numpy as np
from numba import cuda

N = 100

@cuda.jit
def matmul(A, B, C):
    row, col = cuda.grid(2)
    if row < N and col < N:
        tmp = 0.
        for k in range(N):
            tmp += A[row, k] * B[k, col]
        C[row, col] = tmp

A = np.ones((N, N), dtype=np.float32)
B = np.ones((N, N), dtype=np.float32)
C = np.zeros((N, N), dtype=np.float32)

d_A = cuda.to_device(A)
d_B = cuda.to_device(B)
d_C = cuda.device_array((N, N), dtype=np.float32)

threadsperblock = (16, 16)
blockspergrid_x = (N + threadsperblock[0] - 1) // threadsperblock[0]
blockspergrid_y = (N + threadsperblock[1] - 1) // threadsperblock[1]

matmul[(blockspergrid_x, blockspergrid_y), threadsperblock](d_A, d_B, d_C)

C = d_C.copy_to_host()
print("C[0][0] =", C[0][0])
```
### 📐 MATLAB (Using GPU Arrays)

```matlab
N = 100;
A = ones(N, N, 'gpuArray');
B = ones(N, N, 'gpuArray');

C = A * B;

C_host = gather(C);
disp(C_host(1,1));
```
### 📊 Fortran (with CUDA Fortran by PGI)

```fortran
program matmul_cuda
  use cudafor
  implicit none

  integer, parameter :: N = 100
  real, device :: d_A(N,N), d_B(N,N), d_C(N,N)
  real :: A(N,N), B(N,N), C(N,N)
  integer :: i, j, k

  A = 1.0
  B = 1.0

  d_A = A
  d_B = B

  call matmul_kernel<<<1, dim3(N,N)>>>(d_A, d_B, d_C, N)

  C = d_C
  print *, 'C(1,1) = ', C(1,1)

contains

  attributes(global) subroutine matmul_kernel(A, B, C, N)
    real, device :: A(:,:), B(:,:), C(:,:)
    integer, value :: N
    integer :: row, col, k
    real :: sum

    row = threadIdx%y
    col = threadIdx%x
    sum = 0.0

    do k = 1, N
       sum = sum + A(row,k) * B(k,col)
    end do

    C(row,col) = sum
  end subroutine matmul_kernel

end program matmul_cuda
```

---

## 🧠 Related Topics

- [[Compute APIs]]
- [[GPGPU]]
- [[OpenCL]]
- [[SYCL]]
- [[ROCm]]
