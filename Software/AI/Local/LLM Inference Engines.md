# LLM Inference Engines

A practical guide to the software that actually runs LLMs on your hardware. The engine you choose can mean 2-3x performance differences on the same GPU. This covers [[llama.cpp]], [[vLLM]], [[exllamav2]], [[Ollama]], [[LM Studio]], [[TensorRT-LLM]], and when to use each.

For hardware selection see [[LLM Inference Hardware]]. For model selection see [[LLM Model Guide]].

---

## 🎯 The Quick Answer

| Your Situation | Use This |
|----------------|----------|
| Just want to chat locally, simple setup | [[Ollama]] or [[LM Studio]] |
| Developer building apps, need API | [[Ollama]] |
| Maximum single-user speed, quantized models | [[exllamav2]] |
| High-throughput serving, many users | [[vLLM]] |
| CPU offloading, low VRAM | [[llama.cpp]] |
| Production NVIDIA, absolute best performance | [[TensorRT-LLM]] |
| Apple Silicon, M-series Mac | [[LM Studio]] (MLX) or [[llama.cpp]] |
| AMD GPU | [[llama.cpp]] (Vulkan/ROCm) |

---

## 📊 Engine Comparison

| Engine | Speed | Multi-GPU | Ease of Use | Quantization | Best For |
|--------|-------|-----------|-------------|--------------|----------|
| [[llama.cpp]] | Good | Okay | Medium | GGUF (Q2-Q8) | Flexibility, CPU offload |
| [[Ollama]] | Good | Limited | Easy | GGUF | Simple setup, dev API |
| [[LM Studio]] | Good | No | Easiest | GGUF | Beginners, GUI |
| [[exllamav2]] | Excellent | Good | Medium | EXL2 | Fast quantized inference |
| [[vLLM]] | Excellent | Excellent | Medium | AWQ, GPTQ | High-throughput serving |
| [[TensorRT-LLM]] | Best | Excellent | Hard | FP8, INT4 | Production NVIDIA |
| [[text-generation-webui]] | Varies | Okay | Easy | Multiple | Feature-rich UI |

---

## 🔧 The Engines

### llama.cpp

The foundation. Pure C/C++ implementation that most other tools build on.

**What it is:**
- Georgi Gerganov's C++ port of LLaMA inference
- No dependencies, runs anywhere (CPU, CUDA, ROCm, Metal, Vulkan)
- GGUF quantization format (Q2 through Q8, K-quants)
- ~90 MB total size

**Strengths:**
- **CPU offloading** - Can split model between GPU and CPU when VRAM is tight
- **Portability** - Runs on everything from phones to servers
- **Flexibility** - Fine-grained control over layers, threads, batch size
- **Active development** - New optimizations constantly

**Weaknesses:**
- Slower than specialized GPU engines on pure GPU workloads
- Multi-GPU scaling not as good as vLLM
- CLI-focused, no built-in GUI

**Performance:**
- ~150 t/s on RTX 4090 for 8B Q4
- 2.2x slower than exllamav2 for prompt processing
- Best choice when you need CPU+GPU hybrid

**When to use:**
- Low VRAM, need CPU offloading
- Non-NVIDIA GPU (AMD, Intel, Apple)
- Embedded systems, edge devices
- Maximum compatibility

```bash
# Basic usage
./llama-cli -m model.gguf -p "Hello" -n 100

# With GPU layers
./llama-cli -m model.gguf -ngl 35 -p "Hello"

# Server mode
./llama-server -m model.gguf -ngl 35 --port 8080
```

---

### Ollama

The Docker of LLMs. Simple CLI that wraps llama.cpp.

**What it is:**
- User-friendly wrapper around llama.cpp
- `ollama run llama3` - that's it
- Built-in model library
- OpenAI-compatible REST API

**Strengths:**
- **Stupidly easy** - One command to download and run
- **API included** - REST API works out of the box
- **Model management** - `ollama pull`, `ollama list`, `ollama rm`
- **Modelfile** - Docker-like customization

**Weaknesses:**
- Less control than raw llama.cpp
- Multi-GPU support limited
- Can't use your own GGUF files as easily (need to create Modelfile)

**Performance:**
- Same as llama.cpp (it's a wrapper)
- Good request batching for concurrent users
- ~140 t/s on RTX 4090 for 8B Q4

**When to use:**
- Just want it to work
- Building apps that need LLM API
- Don't want to manage model files manually

```bash
# Install and run
curl -fsSL https://ollama.com/install.sh | sh
ollama run llama3.3

# API usage
curl http://localhost:11434/api/generate -d '{
  "model": "llama3.3",
  "prompt": "Hello"
}'
```

---

### LM Studio

GUI for people who don't want CLI.

**What it is:**
- Desktop app with graphical interface
- Built on llama.cpp
- Integrated model browser (HuggingFace)
- Chat interface + local API server

**Strengths:**
- **No command line** - Point and click
- **Model discovery** - Browse and download from UI
- **MLX support** - Optimized for Apple Silicon
- **Nice chat UI** - Actually pleasant to use

**Weaknesses:**
- No multi-GPU
- Less flexible than CLI tools
- Closed source (free but not open)

**Performance:**
- Same as llama.cpp base
- MLX backend very fast on M-series Macs
- CUDA 12.8 support for RTX cards

**When to use:**
- New to local LLMs
- Want GUI, not terminal
- Apple Silicon Mac (MLX optimization)
- Just chatting, not building apps

---

### exllamav2

Speed demon for quantized models.

**What it is:**
- Highly optimized CUDA inference
- EXL2 quantization format (better quality than GGUF at same size)
- Designed for maximum GPU performance

**Strengths:**
- **Fastest quantized inference** - 2.2x faster than llama.cpp on prompts
- **EXL2 format** - Better quality/size ratio than GGUF
- **Multi-GPU** - Good tensor parallelism
- **Low VRAM efficiency** - Excellent for fitting big models

**Weaknesses:**
- CUDA only (no AMD, no CPU)
- Smaller ecosystem than llama.cpp
- Need to convert models to EXL2

**Performance:**
- ~180 t/s on RTX 4090 for 8B (fastest single-GPU)
- 2.2x faster prompt processing than llama.cpp
- Excellent for 70B on 2x 24GB cards

**When to use:**
- NVIDIA GPU, want maximum speed
- Running quantized models (not FP16)
- Multi-GPU setups without NVLink

```python
from exllamav2 import ExLlamaV2, ExLlamaV2Config
config = ExLlamaV2Config("model_path")
model = ExLlamaV2(config)
```

---

### vLLM

The production server. Built for high throughput.

**What it is:**
- Python-based inference server
- PagedAttention for memory efficiency
- Continuous batching
- OpenAI-compatible API

**Strengths:**
- **Throughput king** - 2-4x more requests/sec than naive serving
- **PagedAttention** - Revolutionary memory management
- **Multi-GPU** - Excellent tensor parallelism
- **Production ready** - Used by major companies

**Weaknesses:**
- Higher latency for single requests (optimized for batches)
- Python overhead
- More complex setup than Ollama

**Performance:**
- 2,300-2,500 t/s for Llama 8B on H100 (batched)
- 14-24x higher throughput than HuggingFace Transformers
- Scales well from 10 to 100+ concurrent users

**When to use:**
- Serving many users
- API endpoint for applications
- Multi-GPU datacenter deployment
- Throughput > single-user latency

```bash
# Install
pip install vllm

# Serve
python -m vllm.entrypoints.openai.api_server \
    --model meta-llama/Llama-3-8B-Instruct \
    --tensor-parallel-size 2
```

---

### TensorRT-LLM

NVIDIA's nuclear option. Maximum performance, maximum complexity.

**What it is:**
- NVIDIA's optimized inference library
- Compiles models to TensorRT engines
- FP8/INT4 quantization
- Inflight batching, speculative decoding

**Strengths:**
- **Absolute fastest** on NVIDIA GPUs
- **FP8 support** - H100/B200 tensor cores
- **1.3-2.7x faster** than vLLM in benchmarks
- **10,000+ tokens/sec** on H100 at 64 concurrent

**Weaknesses:**
- NVIDIA only
- Complex setup (compile engines for each model)
- Weeks of engineering investment
- Less model support than vLLM

**Performance:**
- 180-220 req/sec throughput
- 35-50ms time-to-first-token
- 4.6x faster than A100 when using H100 FP8

**When to use:**
- Production at scale (>$50K/month inference spend)
- H100/B200 hardware
- Have engineering resources for optimization
- Absolute performance matters more than flexibility

```bash
# Build engine (simplified)
trtllm-build --model_dir ./model --output_dir ./engine

# Serve
mpirun -n 2 python run.py --engine_dir ./engine
```

---

### text-generation-webui (oobabooga)

The Swiss Army knife.

**What it is:**
- Gradio-based web UI
- Supports multiple backends (llama.cpp, exllamav2, transformers)
- Extensions ecosystem
- Character cards, chat modes

**Strengths:**
- **Multiple backends** - Switch between engines
- **Extensions** - Voice, image gen, RAG, etc.
- **Features** - Most feature-rich interface
- **Flexibility** - GPTQ, AWQ, EXL2, GGUF all supported

**Weaknesses:**
- Jack of all trades, master of none
- Can be slower than dedicated tools
- Complex configuration

**When to use:**
- Want one tool for everything
- Need extensions (voice, multimodal)
- Experimenting with different backends

---

## 📈 Benchmark Summary

### Single-User Latency (8B Q4, RTX 4090)

| Engine | Tokens/Second | Notes |
|--------|---------------|-------|
| TensorRT-LLM | ~200+ t/s | Requires engine build |
| exllamav2 | ~180 t/s | EXL2 format |
| vLLM | ~160 t/s | Better with batching |
| llama.cpp | ~150 t/s | Most flexible |
| Ollama | ~140 t/s | Easiest setup |

### Multi-User Throughput (8B, H100)

| Engine | Tokens/Second (64 users) |
|--------|--------------------------|
| TensorRT-LLM | 10,000+ t/s |
| vLLM | 2,500 t/s |
| exllamav2 | ~1,500 t/s |
| llama.cpp | ~800 t/s |

### Prompt Processing Speed (3200 tokens)

| Engine | Relative Speed |
|--------|----------------|
| exllamav2 | 1.0x (fastest) |
| vLLM | 1.3x slower |
| llama.cpp | 2.2x slower |

---

## 🎯 Decision Tree

```
Need CPU offloading?
├─ Yes → llama.cpp
└─ No
   └─ AMD GPU?
      ├─ Yes → llama.cpp (ROCm) or Ollama
      └─ No (NVIDIA)
         └─ Serving many users?
            ├─ Yes → vLLM or TensorRT-LLM
            └─ No (single user)
               └─ Want easy setup?
                  ├─ Yes → Ollama or LM Studio
                  └─ No (want speed)
                     └─ exllamav2
```

---

## 🔗 Quantization Format Compatibility

| Engine | GGUF | EXL2 | AWQ | GPTQ | FP16 | FP8 |
|--------|------|------|-----|------|------|-----|
| llama.cpp | ✅ | ❌ | ❌ | ❌ | ✅ | ❌ |
| Ollama | ✅ | ❌ | ❌ | ❌ | ✅ | ❌ |
| LM Studio | ✅ | ❌ | ❌ | ❌ | ✅ | ❌ |
| exllamav2 | ❌ | ✅ | ❌ | ✅ | ✅ | ❌ |
| vLLM | ❌ | ❌ | ✅ | ✅ | ✅ | ✅ |
| TensorRT-LLM | ❌ | ❌ | ✅ | ❌ | ✅ | ✅ |

See [[Quantization]] for format details.

---

## 🔗 Related Concepts

- [[LLM Inference Hardware]] - What hardware to run on
- [[LLM Model Guide]] - Which models to run
- [[LLM Under Your Floorboards]] - Running unrestricted models
- [[Serving Local LLMs]] - Multi-user API endpoints and production deployment
- [[Quantization]] - GGUF, EXL2, AWQ formats
- [[llama.cpp]] - The foundation
- [[vLLM]] - High-throughput serving
- [[Ollama]] - Easy local LLMs
- [[LM Studio]] - GUI for local LLMs
- [[TensorRT-LLM]] - NVIDIA optimization
- [[CUDA]] - NVIDIA compute
- [[ROCm]] - AMD compute

---

## 📚 External Resources

- [llama.cpp GitHub](https://github.com/ggerganov/llama.cpp)
- [vLLM Documentation](https://docs.vllm.ai/)
- [exllamav2 GitHub](https://github.com/turboderp/exllamav2)
- [Ollama](https://ollama.com/)
- [LM Studio](https://lmstudio.ai/)
- [TensorRT-LLM GitHub](https://github.com/NVIDIA/TensorRT-LLM)
- [text-generation-webui](https://github.com/oobabooga/text-generation-webui)
- [vLLM vs llama.cpp comparison](https://developers.redhat.com/articles/2025/09/30/vllm-or-llamacpp-choosing-right-llm-inference-engine-your-use-case)

