# JIT Warmup

**JIT Warmup** refers to the initial compilation overhead when running just-in-time compiled code for the first time. In frameworks like [[Numba]], [[CUDA]], and [[PyTorch]]'s `torch.compile`, the first execution triggers compilation from Python/IR to machine code, which can be orders of magnitude slower than subsequent runs. Understanding warmup is essential for accurate benchmarking and production deployment.

---

## Overview

JIT (Just-In-Time) compilation defers code generation until runtime, allowing specialization based on actual input types and shapes. The tradeoff: first execution includes compilation time, making it dramatically slower. A function that runs in microseconds after warmup might take hundreds of milliseconds on first call.

This is not a bug—it's the expected behavior of JIT systems.

---

## Core Concepts

- **Cold Start**: First execution that triggers compilation
- **Warm Run**: Subsequent executions using cached compiled code
- **Compilation Cache**: Stored compiled artifacts to avoid recompilation
- **AOT (Ahead-of-Time)**: Pre-compiling before runtime to eliminate warmup
- **Specialization**: JIT compiles specific versions for input types/shapes
- **Tracing**: Recording operations to compile (used by PyTorch, JAX)

---

## Warmup by Framework

| Framework | Trigger | Typical Overhead | Caching |
|-----------|---------|------------------|---------|
| **Numba @jit** | First call with new types | 100ms - 2s | `cache=True` saves to disk |
| **Numba @cuda.jit** | First call | 50ms - 500ms | Automatic PTX caching |
| **PyTorch torch.compile** | First forward pass | 1s - 30s+ | `torch._dynamo.config.cache_size_limit` |
| **JAX jit** | First call with new shapes | 100ms - 10s | Persistent via `jax.cache` |
| **TensorRT** | Engine build | Minutes | Serialized engine files |
| **ONNX Runtime** | Session creation | 100ms - 1s | Pre-optimized models |

---

## Example: Numba Warmup

```python
from numba import jit
import time

@jit(nopython=True)
def compute(x):
    return x * x + 2 * x + 1

# Cold start - includes compilation
start = time.time()
result = compute(10)
print(f"First call: {time.time() - start:.4f}s")  # ~0.3s

# Warm run - compiled code only
start = time.time()
result = compute(10)
print(f"Second call: {time.time() - start:.6f}s")  # ~0.000006s
```

Difference can be **50,000x** or more.

---

## Benchmarking Best Practices

1. **Always run warmup iterations** before timing
2. **Exclude first N runs** from timing statistics
3. **Use framework-specific timing** (e.g., CUDA events, `torch.cuda.synchronize()`)
4. **Report both** cold and warm performance if relevant
5. **Control for shape changes** that trigger recompilation

```python
# Correct benchmarking pattern
for _ in range(10):  # Warmup
    result = model(input)

torch.cuda.synchronize()
start = time.time()
for _ in range(100):  # Actual timing
    result = model(input)
torch.cuda.synchronize()
elapsed = time.time() - start
```

---

## Caching Strategies

| Strategy | How | When to Use |
|----------|-----|-------------|
| **Disk cache** | `@jit(cache=True)` | Repeated script runs |
| **Persistent compilation** | Save compiled model | Production deployment |
| **AOT compilation** | `numba.pycc`, TorchScript export | Eliminate runtime compilation |
| **Warm in background** | Pre-call with dummy data | Latency-sensitive services |
| **Shape pinning** | Fixed input shapes | Avoid recompilation on shape change |

---

## Numba-Specific Notes

- **CPU JIT (@jit)**: Significant warmup, use `cache=True`
- **CUDA JIT (@cuda.jit)**: Less warmup variance than CPU JIT
- **Eager compilation**: `@jit(signature)` compiles immediately on decoration
- **Parallel (@njit(parallel=True))**: May have additional threading overhead

---

## PyTorch torch.compile Notes

- **dynamo**: Traces Python bytecode
- **inductor**: Generates optimized kernels
- **First compile can take 10-30+ seconds** for large models
- **Recompiles on** shape changes, control flow changes
- **mode="reduce-overhead"**: Reduces recompilation but uses more memory

---

## Comparison: JIT vs AOT

| Aspect | JIT | AOT |
|--------|-----|-----|
| **First run** | Slow (compilation) | Fast (pre-compiled) |
| **Flexibility** | Handles any input type/shape | Fixed signatures |
| **Deployment** | Simpler | Requires build step |
| **Debugging** | Easier | Harder |
| **Use case** | Development, variable workloads | Production, latency-critical |

---

## Related Notes

- [[Numba]] (Python JIT compiler)
- [[CUDA]] (GPU computing)
- [[PyTorch]] (torch.compile JIT)
- [[JAX]] (XLA JIT compilation)
- [[TensorRT]] (Inference optimization)
- [[ONNX]] (Model format with runtime JIT)

---

## External Resources

- [Numba 5-Minute Guide](https://numba.readthedocs.io/en/stable/user/5minguide.html)
- [PyTorch torch.compile Tutorial](https://pytorch.org/tutorials/intermediate/torch_compile_tutorial.html)
- [JAX JIT Mechanics](https://jax.readthedocs.io/en/latest/jax-101/02-jitting.html)
- [NVIDIA Numba CUDA Guide](https://developer.nvidia.com/blog/numba-python-cuda-acceleration/)

---
