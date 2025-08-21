# Numba (NumPy-aware JIT for Python)

**Numba** is a just-in-time (JIT) compiler that accelerates numerical Python by compiling a subset of Python and `numpy` into optimized machine code via LLVM. It can target multi-core CPUs (with SIMD and threading) and NVIDIA GPUs (CUDA), often delivering near-C/Fortran performance while keeping Pythonic syntax. In robotics, simulation, planning, perception, and model-based control loops, Numba helps push Python toward real-time constraints without a full rewrite in C/C++.

---

## ⚙️ Overview

Numba specializes in **array-oriented**, **numerical** workloads. It works by specializing functions on first call (per argument dtype/shape/signature), compiling to native code, and caching for reuse. It integrates tightly with `numpy` semantics (universal functions, broadcasting, views/strides) and supports explicit parallelism (`prange`) and GPU kernels (`numba.cuda`).

---

## 🧠 Core Concepts

- **nopython mode**: `@njit` compiles a Python function to native code that operates on primitive types and `numpy` arrays; no Python objects at runtime.
- **Type specialization**: First call triggers compilation for the concrete signature (e.g., `float64[:]`); new signatures compile lazily.
- **Parallel CPU**: `@njit(parallel=True)` enables auto-parallelization and `prange` for loops; vectorization via LLVM autovec.
- **UFuncs/GUFuncs**: `@vectorize` and `@guvectorize` create NumPy-compatible (g)ufuncs for elementwise or batched ops.
- **CUDA target**: `numba.cuda.jit` writes GPU kernels in Python; device arrays use zero-copy/pinned memory strategies where appropriate.
- **Caching**: `cache=True` stores compiled artifacts on disk; environment toggles allow disabling JIT for debugging.

---

## 🧩 How It Works (Compiler Pipeline)

1. **Bytecode → Numba IR** with type inference from arguments and `numpy` ops.  
2. **Lowering** to LLVM IR (CPU) or PTX (CUDA) with loop fusion, bounds checks elimination, and SIMDization where possible.  
3. **Codegen** to machine code; **runtime linking** and **call dispatch** bound to the inferred signature(s).  
4. **GIL release** in nopython sections for true parallel CPU execution.

---

## 📊 Comparison Chart (Numba vs Adjacent Tools)

| Capability / Tool            | **Numba** | **CuPy** | **Cython** | **JAX** | **PyTorch (torch.compile)** | **Pythran** | **Triton** |
|-----------------------------|-----------|----------|------------|---------|-----------------------------|-------------|------------|
| Primary Model               | JIT compile Python/NumPy | NumPy-like on GPU | Static AOT/Py+C | XLA tracing/JIT | Graph capture/JIT | AOT C++ for NumPy subset | GPU kernel DSL |
| CPU Acceleration            | ✅ (SIMD, threads) | ❌ | ✅ (C-level) | ✅ (via XLA) | ✅ (limited to tensors) | ✅ | ❌ |
| GPU Acceleration            | ✅ (CUDA) | ✅ (CUDA) | Via CUDA C libs | ✅ (multiple backends) | ✅ (CUDA) | ❌ | ✅ (CUDA) |
| Python Syntax Kept          | High      | High (NumPy API) | Medium (typed/annotated) | Medium (pure/functional style) | High (tensor ops) | Medium | Medium (kernel language) |
| Best For                    | Custom kernels, tight loops | Drop-in GPU arrays | Fine-grained control & binding | Autodiff/ML, function transforms | DL & tensor programs | NumPy-heavy kernels AOT | Hand-tuned GPU blocks |
| Autodiff                    | ❌        | ❌       | ❌         | ✅      | ✅                          | ❌          | ❌ |
| Ease to Prototype           | High      | High     | Medium     | Medium  | High                        | Medium      | Medium/Low |
| Drop-in NumPy Replacement   | Partial (with ufuncs) | ✅ | ❌ | ❌ | ❌ | Partial | ❌ |

*Rule of thumb*: **Numba** for speeding up *your own* Python loops and custom kernels; **CuPy** for *drop-in* GPU acceleration of NumPy-like array ops; **Cython/Pythran** for AOT and packaging; **JAX/PyTorch** when you need autodiff and end-to-end tensor graphs; **Triton** for hand-optimized GPU kernels.

---

## 🛠️ Quick One-Liners (Cheat Sheet)

- Compile a function lazily: `from numba import njit; f = njit(lambda x: x*x); f(3.0)`
- Parallel loop hint: `from numba import njit, prange; g = njit(parallel=True)(lambda n: sum(i for i in prange(n)))`
- Vectorized ufunc: `from numba import vectorize; add = vectorize(['float64(float64,float64)'])(lambda a,b:a+b)`
- Generalized ufunc (signature): `from numba import guvectorize; scale = guvectorize(['void(float64[:],float64,float64[:])'],'(n),()->(n)')(lambda x,a,out: out.__setitem__(slice(None),x*a))`
- Enable fastmath: `h = njit(fastmath=True)(lambda x: x*x + 2.0*x + 1.0)`
- Inspect types: `f.inspect_types()`  
- Disable JIT (debug): `import os; os.environ['NUMBA_DISABLE_JIT']='1'`
- Set threads: `import os; os.environ['NUMBA_NUM_THREADS']='8'`
- System info dump: `import sys,subprocess; subprocess.run([sys.executable,'-m','numba','-s'])`
- CUDA device array: `from numba import cuda; import numpy as np; d = cuda.to_device(np.arange(8)); d.copy_to_host()`
- CUDA thread indexing: `from numba import cuda; idx = cuda.grid(1)`
- CUDA shared memory: `from numba import cuda; sm = cuda.shared.array(256, dtype=cuda.float32)`

---

## 🚀 Use Cases in Robotics & HPC

- **Perception pipelines**: Accelerate pixel/voxel loops, filters, and scanline algorithms (e.g., local descriptors, occupancy updates).
- **Optimization**: Cost function evaluations and Jacobian-free inner loops in trajectory optimization and MPC (without full autodiff).
- **SLAM & Mapping**: Fast raycasting, TSDF updates, ICP inner loops where NumPy broadcasting is awkward.
- **Simulation**: Physics micro-steps, contact resolution, collision checks.
- **Signal Processing**: FIR/IIR updates, resampling, spectral transforms around NumPy primitives.
- **On-GPU preprocessing**: Custom CUDA kernels for data augmentation, packing/unpacking sensor frames.

---

## ✅ Strengths

- **Low-friction speedups**: Keep Pythonic code; add `@njit` and refactor tight loops.
- **Bridges CPU↔GPU**: Same project can mix CPU `@njit` with GPU `cuda.jit` where it counts.
- **Excellent NumPy interop**: Supports many array ops, broadcasting, and dtypes.
- **Parallel patterns**: `prange`, reduction patterns, ufunc/gufunc abstractions.
- **GIL-free regions**: True multi-threaded execution in compiled sections.

---

## ⚠️ Limitations & Gotchas

- **Python feature subset**: No arbitrary Python objects in nopython; avoid dictionaries/lists at runtime (use typed equivalents or pre-allocate arrays).
- **Compilation overhead**: First-call latency; use `cache=True` or warm-up on startup.
- **Type instability**: Mixed dtypes trigger multiple specializations; keep inputs consistent.
- **NumPy coverage gaps**: Not all `numpy` APIs are supported (especially high-level or object-returning ones).
- **Debuggability**: Harder than pure Python; use inspection and disable JIT to bisect.
- **GPU constraints**: CUDA target requires NVIDIA GPUs and CUDA toolkit/driver alignment.

---

## 🧮 Performance Playbook (CPU)

- Prefer **contiguous arrays** and simple strides; use `np.ascontiguousarray(a)` when needed.
- Hoist **shape/dtype checks** and constants out of loops; avoid Python objects in hot paths.
- Use **`prange`** for outer parallel loops; minimize shared writes (favor reductions).
- Enable **`fastmath=True`** if numerically acceptable (allows reassociation/approx).
- Pre-allocate outputs; avoid allocations in tight loops.
- Use **typed memoryviews-like** logic (slicing with known strides/shapes).

---

## 🧵 Performance Playbook (CUDA)

- Map **threads to elements** with `cuda.grid(1)` and sized launches; align blocks to warp size.
- Use **shared memory** for tile/block-local reuse; minimize global memory traffic.
- Coalesce memory accesses; prefer SoA layouts for vectorized loads.
- Overlap **compute and transfers**; use `cuda.to_device`, streams, and pinned host memory.
- Keep kernels simple; push complex control flow to CPU or multiple kernels.

---

## 🧪 Testing & Debugging Tips

- Compare against `numpy` reference: `np.allclose(f(x), ref(x))`  
- Disable JIT to isolate logic: `os.environ['NUMBA_DISABLE_JIT']='1'`  
- View type inference and LLVM lowering via `f.inspect_types()`  
- Guard numeric behavior: test with different dtypes (`float32`, `float64`, `int32`)  
- Seed/warm-up: call once at startup to pay compile cost predictably.

---

## 🔧 Developer Tools & Env

- **Threading control**: `NUMBA_NUM_THREADS`, `NUMBA_THREADING_LAYER` (`tbb`, `omp`, `workqueue`)
- **Diagnostics**: `python -m numba -s` for environment, `NUMBA_DEBUG_ARRAY_OPT=1` for array opts
- **Caching**: `@njit(cache=True)` to persist compiled artifacts
- **CUDA sim**: `NUMBA_ENABLE_CUDASIM=1` to run kernels in a simulator (slow, but handy)
- **Packaging**: Prefer wheels/conda; pin `llvmlite`/`numba` compat; run CI warm-ups

---

## 🧭 When to Choose What (Numba vs CuPy et al.)

- Choose **Numba** when you have **custom Python loops** or kernels that aren’t expressible as a simple array expression, or when you need **CPU parallelization** without rewriting in C/C++.
- Choose **CuPy** when your workload is mostly **NumPy array ops** and you can benefit from a **drop-in GPU array** with minimal code change.
- Choose **Cython/Pythran** for **AOT** performance, seamless packaging of native extensions, or when you need to interface C/C++ APIs tightly.
- Choose **JAX**/**PyTorch** if you need **autodiff**, SPMD transforms, or end-to-end tensor graphs (and accept graph/tracing constraints).
- Choose **Triton** for **hand-optimized GPU kernels** where you control block-level performance details beyond Numba’s conveniences.

---

## 🧱 Related Concepts / Notes

- [[NumPy]] (Array programming foundation)
- [[CuPy]] (GPU-backed NumPy-like arrays)
- [[Cython]] (AOT compilation for Python/C interop)
- [[JAX]] (Function transformations and XLA)
- [[PyTorch]] (Tensor library with JIT/`torch.compile`)
- [[CUDA]] (GPU programming model for NVIDIA)
- [[LLVM]] (Compiler infrastructure used by Numba)
- [[Triton]] (GPU kernel DSL alternative)

---

## 🌐 External Resources

- Official Numba documentation and user guide
- CUDA target guide and examples
- Parallel acceleration (prange, threading layers) HOWTO
- Numba discourse/forum and GitHub issues for feature support matrix
- Performance tips & case studies (array programming patterns)

---

## 📝 Summary

Numba brings **LLVM-grade performance** to Python-native numerical code with minimal friction. It shines when accelerating custom loops, crafting NumPy-compatible ufuncs, and writing targeted CUDA kernels—all while keeping code in Python. It’s not a universal drop-in like CuPy nor a graph-based autodiff system like JAX/PyTorch, but as a **precision tool for hot paths** on CPU and GPU, it delivers excellent performance with pragmatic ergonomics.
