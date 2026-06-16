---
title: TensorRT
aliases:
  - NVIDIA TensorRT
  - TensorRT Engine
tags:
  - machine-learning
  - inference
  - optimization
  - cuda
---

# TensorRT

**TensorRT** is NVIDIA’s runtime and compiler stack for high-performance deep learning inference. It lowers latency by graph optimization, precision conversion, kernel tuning, and operator fusion for CUDA-capable targets.

It is commonly paired with exported models from frameworks such as [[ONNX]], [[TensorFlow]], or [[PyTorch]].

---

## Core optimization stages

1. **Import**: Convert a trained model to an internal graph.
2. **Optimization**: Fuse operators, fold constants, and prune unnecessary ops.
3. **Precision calibration**: FP16, INT8 calibration, and quantization-aware choices.
4. **Engine build**: Generate hardware-optimized engine for specific GPU architecture.
5. **Runtime execution**: Load engine and run low-latency inference.

---

## Why teams use TensorRT

- Reduce end-to-end inference latency for real-time systems.
- Increase throughput for batch and stream processing.
- Expose predictable performance for deployment on edge GPUs.

Common use cases include robotics perception pipelines (camera + segmentation), real-time policy inference, and telemetry filtering.

---

## Comparison table

| Runtime | Optimization Depth | Best Deployment | Quantization Support | Ease of Use | Typical Use Case |
|---|---|---|---|---|---|
| TensorRT | High | NVIDIA GPUs | FP16/INT8/INT4 workflows | Medium | High-perf robotics perception |
| ONNX Runtime | Medium | CPU/GPU/Cross-platform | FP16/INT8 (varies by EP) | High | Portable deployment |
| ONNX Runtime + TensorRT EP | Hybrid | mixed | FP16/INT8 | Medium | Existing ONNX ecosystem |
| TensorFlow Lite | Medium | Mobile/embedded CPUs | INT8/PTQ | High | Light mobile inference |
| OpenVINO | Medium-High | Intel CPUs/VPUs | INT8/FP16 | Medium | Intel hardware stacks |
| TorchScript | Low-Medium | PyTorch runtime stacks | limited | High | Fast prototyping in PyTorch |

---

## Practical workflow

1. Export model to ONNX if possible.
2. Validate accuracy baseline before optimization.
3. Build TensorRT engine with conservative precision first.
4. Measure latency/throughput on target GPU.
5. Tighten precision to FP16/INT8 only if accuracy stays acceptable.

---

## Pros and limitations

### ✅ Pros
- Significant inference speedups on NVIDIA hardware.
- Mature CUDA toolchain for deployment.
- Good for strict latency budgets.

### ⚠️ Cons
- GPU lock-in compared with cross-platform runtimes.
- Engine builds can be brittle across CUDA/TensorRT version changes.
- Some operators unsupported without fallback.

---

## Related notes

- [[ONNX Runtime]]
- [[GPU Inference]]
- [[Deep Reinforcement Learning]]
- [[Robot Learning Stack]]
