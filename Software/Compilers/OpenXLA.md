# OpenXLA

**OpenXLA** is an open-source machine learning compiler ecosystem that optimizes computation graphs from ML frameworks ([[JAX]], [[TensorFlow]], [[PyTorch]]) for execution on diverse hardware (GPUs, CPUs, TPUs, custom accelerators). XLA stands for **Accelerated Linear Algebra**.

The project emerged from Google's internal XLA compiler and was open-sourced in 2022 as a collaborative effort between major AI companies. It solves the "M frameworks × N hardware targets" fragmentation problem by providing a common intermediate representation and runtime interface.

---

## ⚙️ Overview

XLA takes high-level tensor operations and transforms them into optimized machine code. Its killer feature is **kernel fusion**—combining multiple operations into single kernels to minimize memory bandwidth usage, which is typically the bottleneck on modern accelerators.

The ecosystem includes several components:
- **XLA** - The core compiler
- **StableHLO** - Portable intermediate representation
- **PJRT** - Hardware plugin interface
- **IREE** - Runtime execution environment
- **Shardy** - Distributed sharding system

---

## 🧠 Core Concepts

### Compilation Pipeline

```
ML Framework (JAX/TF/PyTorch)
           ↓
      StableHLO IR
           ↓
    ┌──────────────────────────────────┐
    │   XLA Compiler                   │
    │   ├─ Target-Independent Passes   │
    │   │   ├─ Common Subexpression    │
    │   │   ├─ Constant Folding        │
    │   │   ├─ Dead Code Elimination   │
    │   │   └─ Operation Fusion        │
    │   └─ Target-Specific Passes      │
    │       ├─ GPU Stream Partitioning │
    │       ├─ Library Call Matching   │
    │       └─ Memory Layout Optimization│
    └──────────────────────────────────┘
           ↓
      LLVM IR (for CPU/GPU)
           ↓
    Native Code (PTX for NVIDIA, etc.)
```

### HLO (High-Level Operations)

XLA's internal representation. Operations like `dot`, `conv`, `reduce`, `broadcast` that map cleanly to tensor algebra. StableHLO is the stable, versioned public interface to this.

### Kernel Fusion

The most important optimization. Instead of:
```
# Without fusion (3 memory round-trips)
temp1 = matmul(A, B)      # write to memory
temp2 = add(temp1, bias)  # read, compute, write
output = relu(temp2)      # read, compute, write
```

XLA produces:
```
# With fusion (1 memory write)
output = fused_matmul_add_relu(A, B, bias)  # single kernel
```

Memory bandwidth is the scarcest resource on accelerators. Fusion often provides 2-10x speedups.

---

## 🧩 Ecosystem Components

### StableHLO

Portable operation set that serves as the interface between frameworks and compilers.

| Aspect | Description |
|--------|-------------|
| Purpose | Framework-agnostic IR for ML models |
| Stability | Backward compatible, versioned |
| Producers | JAX, TensorFlow, PyTorch (via torch-mlir) |
| Consumers | XLA, IREE, custom compilers |
| Format | MLIR dialect |

StableHLO enables "compile once, run anywhere"—a model exported from JAX can run through any StableHLO-compatible compiler.

### PJRT (Pluggable JAX Runtime)

Hardware abstraction layer that decouples frameworks from hardware.

| Aspect | Description |
|--------|-------------|
| Purpose | Uniform device interface for ML runtimes |
| Problem Solved | Eliminates M×N framework-hardware integrations |
| How It Works | Hardware vendors implement PJRT plugin once |
| Frameworks | JAX, TensorFlow, PyTorch/XLA |

**Before PJRT**: Each framework needed custom integration for each hardware target.

**After PJRT**: Hardware vendor implements one plugin; all frameworks work automatically.

### IREE (Intermediate Representation Execution Environment)

Runtime and deployment system for compiled ML models.

| Aspect | Description |
|--------|-------------|
| Purpose | Efficient execution of compiled models |
| Targets | Mobile, embedded, server, WebAssembly |
| Input | StableHLO |
| Features | Ahead-of-time compilation, small runtime |

IREE is becoming the standard execution environment for OpenXLA, replacing older XLA runtime components.

### Shardy

Distributed computation and sharding system.

| Aspect | Description |
|--------|-------------|
| Purpose | Partition models across multiple devices |
| Scale | Thousands of chips |
| Features | Automatic and manual sharding strategies |
| Use Case | Large model training (LLMs, etc.) |

---

## 📊 Comparison Chart

| Compiler | Origin | Input | Targets | Strengths |
|----------|--------|-------|---------|-----------|
| **XLA/OpenXLA** | Google | StableHLO | GPU, CPU, TPU | Fusion, scale, ecosystem |
| **TVM** | Apache | Relay/TIR | Broad hardware | Auto-tuning, edge focus |
| **Triton** | OpenAI | Python DSL | NVIDIA GPU | Easy custom kernels |
| **MLIR** | LLVM | Multiple dialects | Extensible | Infrastructure, not end-to-end |
| **TensorRT** | NVIDIA | ONNX, TF | NVIDIA GPU | Best NVIDIA perf |
| **ONNX Runtime** | Microsoft | ONNX | Broad | Portability |
| **Mojo/MAX** | Modular | Mojo/MLIR | CPU, GPU | Unified language+compiler |

---

## 🔧 Hardware Targets

| Target | Backend | Notes |
|--------|---------|-------|
| NVIDIA GPU | LLVM NVPTX → PTX | Primary GPU target |
| AMD GPU | LLVM AMDGPU → GCN/RDNA | ROCm support |
| Google TPU | Custom | First-class support |
| Intel CPU | LLVM x86 | AVX-512, AMX |
| ARM CPU | LLVM AArch64 | Server and mobile |
| Apple Silicon | LLVM + Metal | M-series chips |
| Custom accelerators | Via PJRT plugins | Cerebras, Graphcore, etc. |

---

## 🏗️ Who Uses OpenXLA

### Frameworks

| Framework | Integration |
|-----------|-------------|
| [[JAX]] | Native (XLA is JAX's only backend) |
| [[TensorFlow]] | Native (`tf.function` with XLA) |
| [[PyTorch]] | PyTorch/XLA (torch_xla) |
| [[Nx]] (Elixir) | Via [[EXLA]] backend |

### Companies (Co-developers)

Google, AMD, Apple, Arm, AWS, Alibaba, Cerebras, Graphcore, Hugging Face, Intel, Meta, NVIDIA, SiFive

---

## 🧰 Use Cases

| Use Case | Why XLA |
|----------|---------|
| Large model training | Sharding, multi-device support |
| TPU workloads | Only compiler for TPUs |
| JAX applications | JAX requires XLA |
| Production inference | Ahead-of-time compilation |
| Custom hardware | PJRT plugin system |
| Cross-framework deployment | StableHLO portability |

---

## ✅ Strengths

- **Kernel fusion** dramatically reduces memory bandwidth usage
- **Multi-framework support** (JAX, TF, PyTorch)
- **Multi-hardware support** (GPU, CPU, TPU, custom)
- **Industry backing** from major AI companies
- **Scale** - handles models across thousands of chips
- **StableHLO** provides stable, versioned IR
- **PJRT** simplifies hardware integration
- **Mature** - production-tested at Google scale

---

## ❌ Weaknesses

- **Compilation overhead** - JIT compilation adds latency on first run
- **Dynamic shapes** - historically weak (improving)
- **Debugging** - compiled code harder to debug than eager execution
- **NVIDIA-specific optimizations** - TensorRT often faster on NVIDIA
- **Complexity** - large codebase, steep learning curve for contributors
- **TPU bias** - some features TPU-first

---

## 🔗 Related Concepts

- [[JAX]] - Primary framework using XLA
- [[TensorFlow]] - Original XLA integration
- [[PyTorch]] - Supported via PyTorch/XLA
- [[Nx]] - Elixir numerical computing (uses XLA via EXLA)
- [[EXLA]] - Elixir XLA backend
- [[CUDA]] - NVIDIA's compute platform (XLA target)
- [[TPU]] - Google's accelerator (native XLA target)
- [[LLVM]] - Backend for CPU/GPU code generation
- [[MLIR]] - Infrastructure XLA is built on
- [[TensorRT]] - Alternative compiler for NVIDIA
- [[ONNX]] - Alternative model interchange format
- [[Triton]] - Alternative GPU compiler

---

## 📚 External Resources

### Official

- [OpenXLA Project](https://openxla.org/)
- [XLA GitHub](https://github.com/openxla/xla)
- [StableHLO GitHub](https://github.com/openxla/stablehlo)
- [IREE GitHub](https://github.com/openxla/iree)
- [XLA Architecture](https://openxla.org/xla/architecture)
- [StableHLO Spec](https://openxla.org/stablehlo/spec)

### Guides

- [PJRT Overview (Google Blog)](https://opensource.googleblog.com/2023/05/pjrt-simplifying-ml-hardware-and-framework-integration.html)
- [OpenXLA Announcement](https://opensource.googleblog.com/2023/03/openxla-is-ready-to-accelerate-and-simplify-ml-development.html)
- [PyTorch/XLA Documentation](https://docs.pytorch.org/xla/)

### Community

- [OpenXLA Discuss](https://groups.google.com/a/openxla.org/g/openxla-discuss)
- Community meetings: ~monthly, see openxla.org for schedule
