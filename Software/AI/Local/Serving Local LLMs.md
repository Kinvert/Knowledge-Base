# Serving Local LLMs

A practical guide to exposing local LLMs as API endpoints for multi-user access. This covers serving backends ([[vLLM]], [[Ollama]], [[llama.cpp]]), scaling strategies, load balancing, and production deployment patterns. The goal: turn your local GPU into an AI API that multiple users or applications can hit concurrently.

For single-user inference see [[LLM Inference Engines]]. For hardware requirements see [[LLM Inference Hardware]]. For model selection see [[LLM Model Guide]].

---

## 🎯 Why Serve Instead of Just Run?

| Use Case | Single Instance | Served API |
|----------|-----------------|------------|
| Personal chat | Works fine | Overkill |
| Multiple apps hitting same model | Wasteful (multiple loads) | Single model, multiple clients |
| Team access | Each person loads model | Shared resource |
| Web/mobile app backend | Not possible | Required |
| Programmatic access from scripts | Manual copy-paste | `curl` / SDK calls |
| A/B testing models | Painful | Route to different endpoints |

**The core benefit:** Load the model once (expensive), serve many requests (cheap).

---

## 📊 Serving Engine Comparison

| Engine | Throughput | Latency | Multi-GPU | Ease | Best For |
|--------|------------|---------|-----------|------|----------|
| [[Ollama]] | Good | Low | Limited | Easy | Simple API, dev use |
| [[llama.cpp]] server | Good | Low | Good | Medium | Flexibility, CPU+GPU |
| [[vLLM]] | Excellent | Medium | Excellent | Medium | High throughput |
| [[TensorRT-LLM]] | Best | Low | Excellent | Hard | Production NVIDIA |
| [[text-generation-inference]] | Very Good | Low | Good | Medium | HuggingFace ecosystem |
| [[LocalAI]] | Good | Medium | Limited | Easy | OpenAI drop-in |

---

## 🔧 Ollama Server

The simplest path to an API. Ollama runs a server by default.

### Basic Setup

```bash
# Install
curl -fsSL https://ollama.com/install.sh | sh

# Pull a model
ollama pull llama3.1:70b

# Server starts automatically on install
# Or manually: ollama serve
```

The API is immediately available at `http://localhost:11434`.

### API Usage

```bash
# Generate completion
curl http://localhost:11434/api/generate -d '{
  "model": "llama3.1:70b",
  "prompt": "Explain quantum computing",
  "stream": false
}'

# Chat format
curl http://localhost:11434/api/chat -d '{
  "model": "llama3.1:70b",
  "messages": [{"role": "user", "content": "Hello"}]
}'

# OpenAI-compatible endpoint
curl http://localhost:11434/v1/chat/completions -d '{
  "model": "llama3.1:70b",
  "messages": [{"role": "user", "content": "Hello"}]
}'
```

### Configuration

```bash
# Environment variables
export OLLAMA_HOST=0.0.0.0:11434  # Listen on all interfaces
export OLLAMA_NUM_PARALLEL=4      # Concurrent requests
export OLLAMA_MAX_LOADED_MODELS=2 # Models in memory
export OLLAMA_KEEP_ALIVE=5m       # Unload after idle

# GPU selection
export CUDA_VISIBLE_DEVICES=0,1   # Use specific GPUs
```

### Throughput

| Model | GPU | Concurrent Users | Tokens/Second (Total) |
|-------|-----|------------------|----------------------|
| Llama 3.1 8B | RTX 4090 | 1 | ~140 t/s |
| Llama 3.1 8B | RTX 4090 | 4 | ~280 t/s |
| Llama 3.1 8B | RTX 4090 | 8 | ~320 t/s |
| Llama 3.1 70B | 2x RTX 4090 | 1 | ~25 t/s |
| Llama 3.1 70B | 2x RTX 4090 | 4 | ~45 t/s |

Ollama handles batching automatically but isn't optimized for maximum throughput like vLLM.

---

## 🔧 llama.cpp Server

More control than Ollama, same underlying engine.

### Basic Setup

```bash
# Build with CUDA
git clone https://github.com/ggerganov/llama.cpp
cd llama.cpp
make GGML_CUDA=1 -j

# Run server
./llama-server \
  -m models/llama-3.1-70b.Q4_K_M.gguf \
  -ngl 99 \
  --host 0.0.0.0 \
  --port 8080 \
  -c 8192 \
  --parallel 4
```

### Key Parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| `-ngl` | GPU layers (99 = all) | 0 |
| `-c` | Context size | 2048 |
| `--parallel` | Concurrent slots | 1 |
| `--cont-batching` | Continuous batching | off |
| `--flash-attn` | Flash attention | off |
| `-cb` | Enable batching | off |
| `--metrics` | Prometheus metrics | off |

### Optimized Configuration

```bash
./llama-server \
  -m model.gguf \
  -ngl 99 \
  --host 0.0.0.0 \
  --port 8080 \
  -c 32768 \
  --parallel 8 \
  --cont-batching \
  --flash-attn \
  -cb \
  --metrics \
  -t 8
```

### API Endpoints

```bash
# Completion
curl http://localhost:8080/completion -d '{
  "prompt": "Hello",
  "n_predict": 100
}'

# OpenAI-compatible
curl http://localhost:8080/v1/chat/completions -d '{
  "model": "gpt-3.5-turbo",
  "messages": [{"role": "user", "content": "Hello"}]
}'

# Health check
curl http://localhost:8080/health

# Metrics (Prometheus)
curl http://localhost:8080/metrics
```

---

## 🔧 vLLM (High Throughput)

The production choice for serving many users. PagedAttention enables 2-4x higher throughput than naive serving.

### Basic Setup

```bash
pip install vllm

# Serve a model
python -m vllm.entrypoints.openai.api_server \
  --model meta-llama/Llama-3.1-70B-Instruct \
  --tensor-parallel-size 2 \
  --port 8000
```

### Key Parameters

| Parameter | Description | Notes |
|-----------|-------------|-------|
| `--tensor-parallel-size` | GPUs for tensor parallelism | Must divide evenly |
| `--pipeline-parallel-size` | GPUs for pipeline parallelism | Alternative to TP |
| `--max-model-len` | Maximum context | Affects memory |
| `--gpu-memory-utilization` | VRAM fraction to use | Default 0.9 |
| `--quantization` | AWQ, GPTQ, etc. | Reduces memory |
| `--enable-prefix-caching` | Cache common prefixes | Saves compute |
| `--max-num-seqs` | Max concurrent sequences | Throughput control |

### Production Configuration

```bash
python -m vllm.entrypoints.openai.api_server \
  --model meta-llama/Llama-3.1-70B-Instruct \
  --tensor-parallel-size 4 \
  --max-model-len 32768 \
  --gpu-memory-utilization 0.95 \
  --enable-prefix-caching \
  --max-num-seqs 256 \
  --port 8000 \
  --host 0.0.0.0
```

### Throughput Benchmarks

| Model | Hardware | Concurrent | Throughput | Latency (p50) |
|-------|----------|------------|------------|---------------|
| Llama 3.1 8B | 1x H100 | 64 | 2,500 t/s | 45ms |
| Llama 3.1 8B | 1x RTX 4090 | 32 | 800 t/s | 80ms |
| Llama 3.1 70B | 4x H100 | 64 | 1,200 t/s | 120ms |
| Llama 3.1 70B | 2x RTX 4090 | 16 | 180 t/s | 250ms |
| Llama 3.1 70B | 8x A100 | 128 | 2,000 t/s | 100ms |

vLLM shines with many concurrent users. Single-user latency is higher than llama.cpp.

### Why vLLM is Faster

**PagedAttention:** Instead of pre-allocating memory for maximum sequence length, vLLM allocates memory in pages as needed. This allows:
- More concurrent sequences in same VRAM
- No memory waste on short sequences
- Better GPU utilization

**Continuous Batching:** New requests join the batch without waiting for current batch to complete. Keeps GPU saturated.

---

## 🔧 TensorRT-LLM (Maximum Performance)

NVIDIA's optimized inference. 30-70% faster than vLLM but much harder to set up.

### Setup Overview

```bash
# Requires building TensorRT engines per model
# This is a multi-step process

# 1. Install TensorRT-LLM
pip install tensorrt-llm

# 2. Convert model to TensorRT format
python convert_checkpoint.py \
  --model_dir ./llama-3.1-70b \
  --output_dir ./trt_ckpt \
  --dtype float16 \
  --tp_size 4

# 3. Build TensorRT engine
trtllm-build \
  --checkpoint_dir ./trt_ckpt \
  --output_dir ./trt_engine \
  --gemm_plugin float16 \
  --max_batch_size 64 \
  --max_input_len 4096 \
  --max_seq_len 8192

# 4. Serve
python -m tensorrt_llm.serve \
  --engine_dir ./trt_engine \
  --port 8000
```

### When to Use TensorRT-LLM

| Scenario | Use TensorRT-LLM? |
|----------|-------------------|
| Prototyping | No - too complex |
| Dev/test | No - vLLM easier |
| Production, <100 users | Maybe - depends on latency needs |
| Production, >100 users | Yes - worth the setup |
| H100/B200 hardware | Yes - FP8 support |
| Cost-sensitive at scale | Yes - 30-70% fewer GPUs needed |

---

## 🔧 Text Generation Inference (TGI)

HuggingFace's serving solution. Good middle ground.

```bash
# Docker (easiest)
docker run --gpus all -p 8080:80 \
  -v ~/.cache/huggingface:/data \
  ghcr.io/huggingface/text-generation-inference:latest \
  --model-id meta-llama/Llama-3.1-70B-Instruct \
  --num-shard 2

# API
curl http://localhost:8080/generate -d '{
  "inputs": "Hello, how are you?",
  "parameters": {"max_new_tokens": 100}
}'
```

Good for HuggingFace models, especially with their Inference Endpoints.

---

## 🌐 OpenAI API Compatibility

Most serving engines expose OpenAI-compatible endpoints:

```python
from openai import OpenAI

# Point to local server
client = OpenAI(
    base_url="http://localhost:8000/v1",
    api_key="not-needed"  # Local servers usually don't require keys
)

response = client.chat.completions.create(
    model="llama-3.1-70b",
    messages=[{"role": "user", "content": "Hello"}]
)
print(response.choices[0].message.content)
```

This means existing code using OpenAI's API works with local models by just changing `base_url`.

---

## ⚖️ Load Balancing Multiple Instances

For high availability and throughput, run multiple instances behind a load balancer.

### Nginx Configuration

```nginx
upstream llm_backend {
    least_conn;  # Route to least busy server
    server 127.0.0.1:8001;
    server 127.0.0.1:8002;
    server 127.0.0.1:8003;
    server 127.0.0.1:8004;
}

server {
    listen 80;

    location /v1/ {
        proxy_pass http://llm_backend;
        proxy_http_version 1.1;
        proxy_set_header Connection "";
        proxy_read_timeout 300s;
        proxy_buffering off;  # Important for streaming
    }
}
```

### Docker Compose Multi-Instance

```yaml
version: '3.8'
services:
  llm-1:
    image: vllm/vllm-openai:latest
    deploy:
      resources:
        reservations:
          devices:
            - capabilities: [gpu]
              device_ids: ['0']
    command: --model meta-llama/Llama-3.1-8B-Instruct --port 8000

  llm-2:
    image: vllm/vllm-openai:latest
    deploy:
      resources:
        reservations:
          devices:
            - capabilities: [gpu]
              device_ids: ['1']
    command: --model meta-llama/Llama-3.1-8B-Instruct --port 8000

  nginx:
    image: nginx:alpine
    ports:
      - "8080:80"
    volumes:
      - ./nginx.conf:/etc/nginx/nginx.conf
    depends_on:
      - llm-1
      - llm-2
```

---

## 🔐 Authentication & Rate Limiting

### API Key Authentication (Nginx)

```nginx
map $http_authorization $api_key_valid {
    default 0;
    "Bearer sk-your-secret-key-1" 1;
    "Bearer sk-your-secret-key-2" 1;
}

server {
    location /v1/ {
        if ($api_key_valid = 0) {
            return 401 '{"error": "Invalid API key"}';
        }
        proxy_pass http://llm_backend;
    }
}
```

### Rate Limiting (Nginx)

```nginx
limit_req_zone $binary_remote_addr zone=llm_limit:10m rate=10r/s;

server {
    location /v1/ {
        limit_req zone=llm_limit burst=20 nodelay;
        proxy_pass http://llm_backend;
    }
}
```

### LiteLLM Proxy (Full-Featured)

For production API management, use [[LiteLLM]]:

```bash
pip install litellm[proxy]

# config.yaml
model_list:
  - model_name: gpt-4
    litellm_params:
      model: openai/meta-llama/Llama-3.1-70B-Instruct
      api_base: http://localhost:8000/v1

litellm_settings:
  drop_params: true

general_settings:
  master_key: sk-your-master-key
  database_url: postgresql://user:pass@localhost/litellm

# Run
litellm --config config.yaml
```

LiteLLM provides: API keys, usage tracking, rate limits, cost tracking, multiple model routing.

---

## 📊 Scaling Strategies

### Vertical Scaling (Bigger GPUs)

| Hardware | Max Concurrent (8B) | Max Concurrent (70B) |
|----------|---------------------|----------------------|
| RTX 4090 | ~32 | ~8 |
| RTX A6000 | ~48 | ~12 |
| A100 80GB | ~64 | ~16 |
| H100 80GB | ~128 | ~32 |

### Horizontal Scaling (More Instances)

| Setup | Total Throughput (8B) | Cost |
|-------|----------------------|------|
| 1x RTX 4090 | 800 t/s | $2K |
| 4x RTX 4090 | 3,000 t/s | $8K |
| 1x H100 | 2,500 t/s | $30K |
| 4x RTX 4090 + LB | 3,200 t/s | $10K |

For cost efficiency, multiple consumer GPUs often beat fewer datacenter GPUs.

### Model Sharding vs. Replication

| Strategy | Use Case | Setup |
|----------|----------|-------|
| **Tensor Parallel** | Large model (70B+) that doesn't fit one GPU | `--tensor-parallel-size 4` |
| **Replication** | Smaller model, need more throughput | Multiple instances + load balancer |
| **Hybrid** | Large model + high throughput | TP within node, replicate across nodes |

---

## 🐳 Docker Deployment

### vLLM Docker

```dockerfile
FROM vllm/vllm-openai:latest

ENV MODEL_NAME=meta-llama/Llama-3.1-8B-Instruct
ENV HF_TOKEN=your_token_here

CMD ["--model", "${MODEL_NAME}", "--port", "8000", "--host", "0.0.0.0"]
```

```bash
docker run --gpus all -p 8000:8000 \
  -e HF_TOKEN=your_token \
  vllm/vllm-openai:latest \
  --model meta-llama/Llama-3.1-8B-Instruct
```

### Ollama Docker

```bash
docker run -d --gpus all \
  -v ollama:/root/.ollama \
  -p 11434:11434 \
  --name ollama \
  ollama/ollama

# Pull model
docker exec ollama ollama pull llama3.1:8b
```

---

## 📈 Monitoring

### Prometheus Metrics

Most servers expose Prometheus metrics:

```bash
# llama.cpp
./llama-server --metrics

# vLLM (automatic)
# Metrics at /metrics endpoint

# Key metrics to watch:
# - request_latency_seconds
# - tokens_per_second
# - gpu_memory_usage
# - queue_depth
# - active_requests
```

### Grafana Dashboard

Key panels:
- Requests/second
- Token throughput
- P50/P95/P99 latency
- GPU utilization
- VRAM usage
- Queue depth
- Error rate

---

## 🎯 Quick Reference

| Users | Model | Recommended Setup |
|-------|-------|-------------------|
| 1-5 | 8B | Ollama, single GPU |
| 1-5 | 70B | llama.cpp, multi-GPU |
| 5-20 | 8B | vLLM, single GPU |
| 5-20 | 70B | vLLM, 2-4 GPUs |
| 20-100 | 8B | vLLM, 2+ instances |
| 20-100 | 70B | vLLM, 4+ GPUs, maybe replicate |
| 100+ | 8B | vLLM/TRT-LLM, many instances |
| 100+ | 70B | TensorRT-LLM, serious hardware |

---

## 🔗 Related Concepts

- [[LLM Inference Engines]] - Single-user inference comparison
- [[LLM Inference Hardware]] - Hardware requirements
- [[LLM Model Guide]] - Model selection
- [[vLLM]] - High-throughput serving
- [[Ollama]] - Simple serving
- [[llama.cpp]] - Flexible serving
- [[TensorRT-LLM]] - Maximum performance
- [[LiteLLM]] - API proxy and management
- [[Docker]] - Container deployment
- [[Nginx]] - Load balancing
- [[Prometheus]] - Monitoring
- [[PagedAttention]] - vLLM's memory optimization
- [[Continuous Batching]] - Throughput optimization

---

## 📚 External Resources

- [vLLM Documentation](https://docs.vllm.ai/)
- [Ollama API Documentation](https://github.com/ollama/ollama/blob/main/docs/api.md)
- [llama.cpp Server](https://github.com/ggerganov/llama.cpp/tree/master/examples/server)
- [TensorRT-LLM](https://github.com/NVIDIA/TensorRT-LLM)
- [Text Generation Inference](https://huggingface.co/docs/text-generation-inference)
- [LiteLLM Proxy](https://docs.litellm.ai/docs/proxy/quick_start)
- [vLLM Performance Tips](https://docs.vllm.ai/en/latest/serving/performance.html)

