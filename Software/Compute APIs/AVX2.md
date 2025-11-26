# AVX2

AVX2 is a [[SIMD]] (vector) instruction set extension for [[x86_64]] CPUs that enables processing many numbers in parallel. This massively speeds up numeric compute, including robotics workloads like perception, SLAM, filtering, matrix ops, and control pipelines. Ryzen 9 9950X supports AVX2 (but not full [[AVX-512]]). AVX2 is supported very broadly and is the “safe” baseline for high-perf desktop/server x86 today.

---

## 🧠 Overview

AVX2 operates on 256 bit vector registers. Think: 8 floats at a time, 4 doubles, 32 `int8`s, etc. The parallelism is at the CPU instruction level.

---

## 🧩 Core Concepts

- [[SIMD]] (Single Instruction, Multiple Data) execution
- 256-bit registers
- Aligned memory loads/stores matter
- Highly compiler sensitive (flags like `-mavx2` or `-march=native`)
- Often the single easiest “free multiplier” for robotics numerics

---

## 🏆 Comparison Table

| Tech | Bit width | Ryzen 9 9950X status | Typical use | Notes |
|---|---|---|---|---|
| [[SSE2]] | 128 | supported | legacy baseline | used everywhere for old code |
| [[AVX]] | 256 | supported | float math | often auto-vectorized by compilers |
| AVX2 | 256 | supported | integer + float | the modern baseline for perf |
| [[AVX-512]] | 512 | partial / no full support on consumer AMD | HPC, ML kernels | very powerful but fragmented |
| Neon (ARM) | 128 | N/A on x86 but common on ARM SBCs | embedded robotics (Jetson) | similar idea but different ISA |

---

## 🔍 AVX-512 quick note

- AVX-512 is 512 bit wide, twice AVX2 width.
- Intel server chips have it. Most AMD consumer CPUs do NOT have full AVX-512.
- Code relying on AVX-512 generally falls back to AVX2 on AMD desktop.

Do not assume AVX-512 in robotics code if shipping for general users.

---

## 🧰 Use Cases

- perception filters
- voxel grid downsampling
- point cloud processing
- ICP inner loops
- matrix math acceleration
- model inference kernels

---

## ✅ Strengths

- widely supported
- zero dependencies
- compilers already know to target it
- huge perf multipliers for math

---

## ❌ Weaknesses

- extremely sensitive to memory alignment
- branching kills vectorization
- AVX2 is not portable to ARM Neon (the semantic mapping is non-trivial)

---

## 🔗 Related Concepts / Notes

- [[SIMD]]
- [[Ryzen 9 9950X]]
- [[Ryzen 9 9950X3D]]
- [[Sparse Tensors]] (often vectorized kernels internally)
- [[Linear Algebra]] (AVX2 acceleration inside BLAS libs)

---
## 📚 External Resources

- Intel intrinsics guide (search `intel intrinsics` online)
- Compiler flags: `-mavx2` or `-march=native`

