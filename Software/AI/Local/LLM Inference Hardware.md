# LLM Inference Hardware

A comprehensive guide to hardware selection for running large language models locally. This document maps model sizes to hardware requirements, provides real-world token/second benchmarks, and covers everything from single consumer GPUs to multi-GPU datacenter configurations. The goal: know exactly what hardware can run what models at what speeds.

For the broader context on running unrestricted models, power requirements, and model selection, see [[LLM Under Your Floorboards]]. For model benchmarks, specializations, and which model to choose, see [[LLM Model Guide]].

---

## 🎯 The Core Equation

LLM inference is fundamentally **memory-bandwidth bound**, not compute-bound. The speed at which you generate tokens depends primarily on:

1. **VRAM capacity** - Can you fit the model?
2. **Memory bandwidth** - How fast can you feed the GPU?
3. **Multi-GPU interconnect** - How fast can GPUs talk to each other?

A model that doesn't fit in VRAM either can't run, runs partially on CPU (10-100x slower), or requires multi-GPU splitting with interconnect overhead.

---

## 📊 Model Size Reference

Before diving into hardware, understand what you're trying to run:

| Model Size | FP16 Weight Size | Q8 Size | Q4 Size | Context Overhead |
|------------|------------------|---------|---------|------------------|
| **7-8B** | ~16 GB | ~8 GB | ~4-5 GB | ~0.5 GB/4K ctx |
| **13-14B** | ~28 GB | ~14 GB | ~7-8 GB | ~0.8 GB/4K ctx |
| **30-34B** | ~68 GB | ~34 GB | ~17-20 GB | ~1.5 GB/4K ctx |
| **70B** | ~140 GB | ~70 GB | ~35-40 GB | ~2.5 GB/4K ctx |
| **110B** | ~220 GB | ~110 GB | ~55-60 GB | ~4 GB/4K ctx |
| **405B** | ~810 GB | ~405 GB | ~200 GB | ~8 GB/4K ctx |
| **671B (DeepSeek R1)** | ~1.3 TB | ~670 GB | ~330 GB | Varies (MoE) |

**MoE models** (Mixtral, DeepSeek) are special - they have more total parameters but only activate a fraction per token:
- [[Mixtral]] 8x7B: 46.7B total, ~12.9B active
- [[Mixtral]] 8x22B: 141B total, ~39B active
- [[DeepSeek]] V2.5: 236B total, ~21B active
- [[DeepSeek]] R1: 671B total, ~37B active per token

---

## 🖥️ Consumer GPU Performance

### Single GPU Benchmarks

Real-world tokens/second using [[llama.cpp]] and similar engines:

| GPU | VRAM | Bandwidth | 7-8B Q4 | 13B Q4 | 34B Q4 | 70B Q4 | Notes |
|-----|------|-----------|---------|--------|--------|--------|-------|
| [[RTX 3060]] 12GB | 12 GB | 360 GB/s | ~40 t/s | ~25 t/s | OOM | OOM | Entry level |
| [[RTX 3080]] 10GB | 10 GB | 760 GB/s | ~55 t/s | OOM | OOM | OOM | VRAM limited |
| [[RTX 3090]] | 24 GB | 936 GB/s | ~65 t/s | ~45 t/s | ~20 t/s | OOM | Still excellent |
| [[RTX 4060 Ti]] 16GB | 16 GB | 288 GB/s | ~38 t/s | ~22 t/s | OOM | OOM | Bandwidth limited |
| [[RTX 4070 Ti Super]] | 16 GB | 672 GB/s | ~58 t/s | ~35 t/s | OOM | OOM | Good mid-range |
| [[RTX 4080 Super]] | 16 GB | 736 GB/s | ~78 t/s | ~45 t/s | OOM | OOM | Still VRAM limited |
| **[[RTX 4090]]** | 24 GB | 1008 GB/s | ~150 t/s | ~110 t/s | ~35 t/s | OOM* | King of consumer |
| **[[RTX 5090]]** | 32 GB | 1792 GB/s | ~213 t/s | ~140 t/s | ~61 t/s | OOM | New champion |

*OOM = Out of Memory. 70B Q4 needs ~35-40 GB VRAM minimum.

### RTX 5090 Deep Dive

The [[RTX 5090]] deserves special attention as the new consumer king:

| Model | Quantization | VRAM Used | Tokens/Second | Context Limit |
|-------|--------------|-----------|---------------|---------------|
| Qwen3 8B | Q4 | ~6 GB | 10,400 t/s (prefill) | 128K+ |
| Llama 3 8B | Q4 | ~5 GB | ~213 t/s | 128K |
| Qwen3 32B | Q4 | ~19 GB | ~61 t/s | 32K+ |
| Qwen3 32B | Q4 | ~19 GB | 2,950 t/s (prefill) | 4K |
| Qwen3moe 30B | Q4 | 31 GB | ~52 t/s | 147K (!) |
| gpt-oss 120B | Extreme Q | 31 GB | ~112 t/s | 131K |

The 32 GB VRAM and 1.8 TB/s bandwidth make the [[RTX 5090]] the first consumer GPU that can comfortably handle 32B models with long context.

### RTX 4090 vs 5090 Comparison

| Metric | RTX 4090 | RTX 5090 | Improvement |
|--------|----------|----------|-------------|
| VRAM | 24 GB | 32 GB | +33% |
| Bandwidth | 1008 GB/s | 1792 GB/s | +78% |
| 8B Q4 t/s | ~150 | ~213 | +42% |
| 32B Q4 t/s | ~35 (tight) | ~61 | +74% |
| Max model (single) | 30-34B | 40-50B | ~50% larger |
| Price | $1,600-2,000 | $2,000-3,800 | +25-90% |

---

## 🔧 Dual Consumer GPU Configurations

Two GPUs can handle larger models, but interconnect matters:

| Configuration | Total VRAM | Interconnect | 70B Q4 | Notes |
|---------------|------------|--------------|--------|-------|
| 2x [[RTX 3090]] | 48 GB | PCIe | ~22 t/s | Popular budget option |
| 2x [[RTX 4090]] | 48 GB | NVLink Bridge | ~35-40 t/s | Best consumer dual |
| 2x [[RTX 4090]] | 48 GB | PCIe only | ~25-30 t/s | No NVLink penalty |
| 2x [[RTX 5090]] | 64 GB | NVLink Bridge | ~50-55 t/s | New dual champion |
| 2x [[RTX 5090]] | 64 GB | PCIe only | ~35-40 t/s | Still very good |

**Key insight:** Consumer [[NVLink]] bridges provide ~112 GB/s, much less than datacenter NVLink (600-900 GB/s), but still 3-4x faster than PCIe for tensor parallelism.

---

## 🏢 Workstation/Prosumer GPUs

For those who need more VRAM without going full datacenter:

| GPU | VRAM | Bandwidth | 34B Q4 | 70B Q4 | 70B Q8 | Price |
|-----|------|-----------|--------|--------|--------|-------|
| [[RTX A5000]] | 24 GB | 768 GB/s | ~25 t/s | OOM | OOM | ~$2,500 |
| **[[RTX A6000]]** | 48 GB | 768 GB/s | ~30 t/s | ~18 t/s | OOM | ~$4,500 |
| [[RTX 5000 Ada]] | 32 GB | 576 GB/s | ~40 t/s | OOM | OOM | ~$4,000 |
| **[[RTX 6000 Ada]]** | 48 GB | 960 GB/s | ~50 t/s | ~25 t/s | OOM | ~$7,000 |
| [[L40S]] | 48 GB | 864 GB/s | ~45 t/s | ~22 t/s | OOM | ~$8,000 |

The [[RTX A6000]] at 48 GB is the sweet spot for running 70B models quantized on a single card without splitting.

---

## 🏭 Datacenter GPUs

Where serious inference happens:

| GPU | VRAM | Bandwidth | TDP | 70B FP16 | 70B Q4 | Price Range |
|-----|------|-----------|-----|----------|--------|-------------|
| [[A100]] 40GB | 40 GB | 1555 GB/s | 250W | OOM | ~22 t/s | $8-12K used |
| **[[A100]] 80GB** | 80 GB | 2039 GB/s | 300W | ~15 t/s | ~35 t/s | $12-18K used |
| [[H100]] PCIe | 80 GB | 2039 GB/s | 350W | ~20 t/s | ~40 t/s | $20-30K |
| **[[H100]] SXM** | 80 GB | 3350 GB/s | 700W | ~25 t/s | ~50 t/s | $25-40K |
| **[[H200]]** | 141 GB | 4800 GB/s | 700W | ~30 t/s | ~60 t/s | $30-40K |
| **[[B200]]** | 192 GB | 8000 GB/s | 1000W | ~50 t/s | ~100 t/s | $45-50K |
| **[[MI300X]]** | 192 GB | 5200 GB/s | 750W | ~25 t/s* | ~55 t/s | $15-25K |

*[[MI300X]] can run 70B on a single GPU due to 192 GB VRAM.

**Power reality check:** A single [[B200]] pulls 1000W. An 8x B200 system pulls 8-10 kW for GPUs alone, plus CPU, memory, cooling. These are not home-friendly devices.

### H100 vs A100 Comparison

| Metric | A100 80GB | H100 SXM |
|--------|-----------|----------|
| Memory Bandwidth | 2039 GB/s | 3350 GB/s |
| FP16 TFLOPS | 77.9 | 267 |
| FP8 TFLOPS | - | 1979 |
| NVLink Bandwidth | 600 GB/s | 900 GB/s |
| 70B Q4 tokens/s | ~22 t/s | ~50 t/s |
| TensorRT-LLM speedup | - | 4.6x vs A100 |

The [[H100]] is 2-2.5x faster than [[A100]] for inference despite being only ~1.5x the memory bandwidth, due to FP8 Transformer Engine optimizations.

---

## 🍎 Apple Silicon

Unified memory architecture allows running models impossible on equivalent VRAM:

| Chip | Unified Memory | Bandwidth | 7-8B Q4 | 13B Q4 | 34B Q4 | 70B Q4 |
|------|----------------|-----------|---------|--------|--------|--------|
| M2 Max | 32-96 GB | 400 GB/s | ~60 t/s | ~40 t/s | ~18 t/s | ~8 t/s* |
| M2 Ultra | 64-192 GB | 800 GB/s | ~90 t/s | ~55 t/s | ~25 t/s | ~12 t/s |
| M3 Max | 36-128 GB | 400 GB/s | ~65 t/s | ~42 t/s | ~20 t/s | ~10 t/s* |
| **M3 Ultra** | 64-192 GB | 800 GB/s | ~115 t/s | ~70 t/s | ~30 t/s | ~15 t/s |
| M4 Max | 36-128 GB | 546 GB/s | ~80 t/s | ~50 t/s | ~25 t/s | ~12 t/s* |
| **M4 Ultra** | 64-256 GB | ~1 TB/s | Est. 140+ | Est. 85+ | Est. 40+ | Est. 20+ |

*Requires 96+ GB configuration.

**Pros:** Silent operation, 40-80W power draw, can run 70B on consumer hardware
**Cons:** Slower than equivalent-VRAM NVIDIA cards, limited MLX ecosystem, expensive

### M5 Preview

Apple's M5 chips show 19-27% improvement over M4 due to higher bandwidth (153 GB/s vs 120 GB/s base). The new GPU Neural Accelerators provide 4x speedup for time-to-first-token.

---

## 📦 Pre-Built Systems

### tinybox Family ([[tinygrad]])

Current v2 lineup (as of 2025):

| Model | GPUs | GPU VRAM | Largest Model | Est. 70B t/s | Price |
|-------|------|----------|---------------|--------------|-------|
| **tinybox red v2** | 4x [[RX 9070 XT]] | 64 GB | 70B Q4 | ~15-20 t/s | $10,000 |
| **tinybox green v2** | 4x [[RTX PRO 6000]] Blackwell | 384 GB | 405B FP16, 671B Q4 | ~50-60 t/s | $50,000 |
| **tinybox pro v2** | 8x [[RTX 5090]] | 256 GB | 405B Q4-Q8 | ~40-50 t/s* | $50,000 |

*The pro v2 has a critical caveat - see below.

---

**tinybox red v2** ($10,000) - Entry AMD:
- 64 GB total → fits 70B Q4 (~35-40GB) with room for context
- 34B at Q8 or FP16, very comfortable
- Expect ~15-20 t/s on 70B Q4, ~40 t/s on 34B
- [[ROCm]] ecosystem, 4x GPU over PCIe
- **In stock, ships within 1 week**

---

**tinybox green v2 Blackwell** ($50,000) - The VRAM king:
- 384 GB total across 4x 96GB RTX PRO 6000 Blackwell
- **This is insane VRAM** - runs 405B at FP16 (~200GB) with 180GB headroom
- Can run 671B (DeepSeek R1) quantized
- 96GB per GPU means single-GPU 70B FP16 - no splitting needed
- Each PRO 6000: 600W, $8,500 standalone
- Expect ~50-60 t/s on 70B FP16 (no quantization needed!)
- Expect ~15-25 t/s on 405B FP16
- **Made to order, 4-12 weeks**

---

**tinybox pro v2** ($50,000) - 8x 5090, but read carefully:
- 256 GB total across 8x 32GB RTX 5090
- **CRITICAL: RTX 5090 has NO NVLink** - all 8 GPUs communicate over PCIe only
- This limits tensor parallelism scaling significantly

**The NVLink problem:**
- 2x 5090 over PCIe: ~27 t/s on 70B - near-linear scaling, great
- 4x 5090 over PCIe: Only 40% faster than 2x - bottleneck appearing
- 8x 5090 over PCIe: Diminishing returns, PCIe becomes the wall
- Datacenter NVLink GPUs are 3-4x faster for 8-way tensor parallelism

**What this means for pro v2:**
- 70B models: Maybe ~40-50 t/s (not 8x single-GPU speed)
- 110B models: Can fit, but scaling limited
- 405B Q4: Fits (~200GB), but 8-way split over PCIe hurts speed
- Best use case: Running multiple smaller models in parallel (batch inference)
- Less ideal: Single large model inference where 8-way TP is needed

**Why buy pro v2 over green v2?**
- More raw compute (2984 vs 3086 TFLOPS, similar)
- 256 cores of CPU for preprocessing
- 2x PCIe 5.0 slots for expansion
- Better for multi-model serving or batch workloads
- Worse for single massive model (405B) due to NVLink absence

---

**Which tinybox v2?**

| Use Case | Best Choice | Why |
|----------|-------------|-----|
| Budget entry, AMD | red v2 ($10K) | 70B Q4 for $10K is great value |
| Single huge model (405B+) | green v2 ($50K) | 384GB VRAM, fewer splits needed |
| Fast 70B-110B inference | green v2 ($50K) | 96GB/GPU = 70B on 1-2 GPUs, less PCIe overhead |
| Multi-model serving | pro v2 ($50K) | 8 GPUs for parallel workloads |
| Batch inference at scale | pro v2 ($50K) | More GPUs = more concurrent requests |
| Training small models | red v2 or pro v2 | TFLOPS matter more for training |

### BIZON Systems

| Model | GPUs | Config | Price Range | Best For |
|-------|------|--------|-------------|----------|
| BIZON G3000 | 2-4x | 4090/5090/6000 Ada | $3,000-15,000 | Researchers, startups |
| BIZON ZX4000 | 2x | A100/H100/4090/5090 | $13,000-50,000 | Workstation use |
| BIZON ZX9000 | 8-10x | H100/H200/A100/4090 | $50,000-200,000+ | Datacenter-lite |
| BIZON G9000 | 8x | HGX H100/H200/A100 | $150,000-400,000+ | Full datacenter |

These come pre-configured with [[PyTorch]], [[TensorFlow]], [[CUDA]] stack, water cooling, and support.

### Lambda Labs

**Note:** Lambda discontinued on-premise Hyperplane servers as of August 2025. Access is now cloud-only.

Cloud pricing (H100): $1.89/GPU/hour
1-Click Clusters: 16-2000+ interconnected B200/H100 GPUs

### NVIDIA DGX

| System | GPUs | VRAM | System Power | Voltage | Price |
|--------|------|------|--------------|---------|-------|
| DGX A100 | 8x A100 80GB | 640 GB | 6.5 kW | 200-240V 40A | ~$200K |
| DGX H100 | 8x H100 SXM | 640 GB | 10.2 kW | 200-240V 60A | ~$300K+ |
| **DGX B200** | 8x B200 | 1536 GB | 14.3 kW | 200-240V 100A | ~$400-500K |
| DGX SuperPOD | 32+ DGX | Varies | 100+ kW | 480V 3-phase | $10M+ |

**Infrastructure notes:**
- DGX H100 and B200 require **dedicated 200-240V circuits** - standard 120V won't work
- DGX B200 pulls 14.3 kW peak - that's a 60A 240V circuit just for one box
- DGX SuperPOD installations require **480V 3-phase power** and industrial cooling
- Liquid cooling options available for H100/B200 to reduce thermal footprint

The [[NVIDIA DGX]] B200 can run:
- Llama 405B at FP16 natively (single system)
- DeepSeek R1 671B quantized
- 15x inference performance vs H100

---

## 📊 What Hardware for What Model?

### Minimum Hardware by Model Size

| Model | Minimum (Functional) | Recommended (Comfortable) | Optimal |
|-------|----------------------|---------------------------|---------|
| **7-8B** | RTX 3060 12GB | RTX 4060 Ti 16GB | RTX 4090 |
| **13-14B** | RTX 4060 Ti 16GB | RTX 4090 | 2x RTX 4090 |
| **30-34B** | RTX 4090 (Q4) | RTX 5090 | RTX A6000 / 2x 4090 |
| **70B** | 2x RTX 4090 (Q4) | 2x RTX 5090 / A100 80GB | H100 / MI300X |
| **110B+** | 4x RTX 4090 | 2x H100 | 4x H100 / 2x MI300X |
| **405B** | 4x H100 (FP8) | 8x H100 | 8x H200 / B200 |
| **671B** | 8x H100 (Q4) | 8x H200 | 8x B200 / 16x H100 |

### Speed Tiers by Experience

| Experience | Tokens/Second | Feels Like |
|------------|---------------|------------|
| Painful | <5 t/s | Watching paint dry |
| Usable | 5-15 t/s | Slow but acceptable |
| Comfortable | 15-30 t/s | Normal conversation |
| Fast | 30-60 t/s | Snappy responses |
| Instant | 60+ t/s | Real-time feel |

Human silent reading speed is ~5-8 words/second (~7-11 tokens/second). Anything above 15 t/s feels responsive.

---

## ⚡ Power Requirements by Tier

What your electrical panel needs to know:

| Setup | GPU Power | System Total | Circuit Required | Voltage | Monthly Cost* |
|-------|-----------|--------------|------------------|---------|---------------|
| Single RTX 4090 | 450W | 600-700W | 15A 120V | Standard | ~$30-50 |
| Single RTX 5090 | 575W | 750-900W | 15A 120V | Standard | ~$40-60 |
| 2x RTX 4090 | 900W | 1200-1400W | 20A 120V | Standard | ~$60-90 |
| 2x RTX 5090 | 1150W | 1500-1800W | 20A 240V | Dedicated | ~$80-120 |
| 4x RTX 4090 | 1800W | 2400-2800W | 30A 240V | Dedicated | ~$120-180 |
| tinybox red v2 (4x 9070 XT) | ~1000W | 1400-1800W | 20A 240V | Dedicated | ~$70-110 |
| tinybox green v2 (4x PRO 6000) | ~2400W | 3000-3500W | 30A 240V | Dedicated | ~$150-220 |
| tinybox pro v2 (8x 5090) | ~4600W | 5500-6500W | 50A 240V | Dedicated | ~$280-400 |
| 2x A100 80GB | 600W | 1000-1200W | 20A 120V | Standard | ~$50-80 |
| 8x A100 80GB | 2400W | 4000-5000W | 30A 240V | Dedicated | ~$200-300 |
| 8x H100 SXM | 5600W | 8000-10000W | 60A 240V | Dedicated | ~$400-600 |
| 8x B200 | 8000W | 12000-14000W | 100A 240V | Dedicated | ~$600-900 |
| DGX SuperPOD | 100+ kW | 150+ kW | 480V 3-phase | Industrial | $5,000+ |

*Assumes $0.12/kWh, 8 hours/day usage. Your rates vary.

**Voltage reality:**
- **120V (US standard):** Max ~1800W per circuit. Fine for 1-2 consumer GPUs.
- **240V (dryer/EV outlet):** Required for serious multi-GPU. Most homes can add a 240V 30A circuit for ~$300-500.
- **480V 3-phase:** Industrial/datacenter only. If you need this, you're not running from home.

**Practical limits:**
- Most US homes have 100-200A service total
- tinybox red v2 (20A 240V) is very home-friendly, ~10-15% of capacity
- tinybox green v2 (30A 240V) is doable, ~15-25% of capacity
- tinybox pro v2 (50A 240V) is serious - 25-50% of a typical home's capacity
- Running 8x H100 at home is technically possible but requires electrical upgrades ($5-20K)
- Anything beyond that: you're building a datacenter, not a home lab

---

## 🔧 Inference Engine Impact

The software stack matters as much as hardware:

| Engine | 8B Performance | 70B Performance | Multi-GPU | Best For |
|--------|----------------|-----------------|-----------|----------|
| [[llama.cpp]] | ~150 t/s (4090) | ~25 t/s (dual) | Good | Flexibility, CPU+GPU |
| [[Ollama]] | ~140 t/s | ~22 t/s | Limited | Easy setup |
| [[vLLM]] | ~2,500 t/s (batched) | ~100+ t/s | Excellent | High throughput |
| [[exllamav2]] | ~180 t/s | ~30 t/s | Good | Fast single-user |
| [[TensorRT-LLM]] | ~250 t/s | ~50 t/s (single H100) | Excellent | Production NVIDIA |

[[TensorRT-LLM]] is often 70% faster than [[llama.cpp]] on the same hardware due to architecture-specific optimizations.

---

## 🎓 Fine-Tuning Hardware Requirements

Different from inference - fine-tuning needs more memory for gradients and optimizer states:

### Full Fine-Tuning

| Model Size | VRAM Required | Hardware |
|------------|---------------|----------|
| 7B | 100-120 GB | 2x A100 80GB |
| 13B | 200+ GB | 4x A100 80GB |
| 70B | 1+ TB | 8x H100 or more |

### LoRA/QLoRA Fine-Tuning

| Model Size | LoRA VRAM | QLoRA VRAM | Minimum Hardware |
|------------|-----------|------------|------------------|
| 7B | 16-24 GB | 8-12 GB | RTX 4090 |
| 13B | 24-32 GB | 12-16 GB | RTX 4090 / A6000 |
| 34B | 48-64 GB | 24-32 GB | 2x RTX 4090 / A100 |
| 70B | 80-100 GB | 40-48 GB | 2x A100 80GB |

[[QLoRA]] reduces fine-tuning memory by ~90% compared to full fine-tuning, making consumer GPUs viable for models up to 70B.

---

## 💰 Cost-Performance Analysis

### Tokens/Second per Dollar (Inference)

| Hardware | Price | 8B Q4 t/s | $/t/s | 70B Q4 t/s | $/t/s |
|----------|-------|-----------|-------|------------|-------|
| RTX 3090 (used) | $700 | ~65 | $10.77 | OOM | - |
| RTX 4090 | $1,800 | ~150 | $12.00 | OOM | - |
| RTX 5090 | $2,500 | ~213 | $11.74 | OOM | - |
| 2x RTX 4090 | $3,600 | ~280 | $12.86 | ~35 | $102.86 |
| RTX A6000 | $4,500 | ~80 | $56.25 | ~18 | $250.00 |
| A100 80GB (used) | $15,000 | ~138 | $108.70 | ~35 | $428.57 |
| H100 SXM | $35,000 | ~200 | $175.00 | ~50 | $700.00 |

**Best value for 8B models:** Used RTX 3090 or RTX 4090
**Best value for 70B models:** 2x RTX 4090 or used A100 80GB

---

## 🔮 Upcoming Hardware (2025-2026)

| Hardware | Expected | Key Improvement |
|----------|----------|-----------------|
| [[RTX 5090]] Super | 2026? | More VRAM? |
| [[B200]] systems | Available now | 192 GB, 8 TB/s bandwidth |
| [[MI350]] | 2025 | AMD's B200 competitor |
| GB200 NVL72 | 2025 | 72 Blackwell GPUs linked |
| [[RTX 6090]] | 2027? | Unknown |

The trend: memory bandwidth increasing faster than compute. FP4/FP8 quantization becoming standard with minimal quality loss.

---

## 📋 Quick Reference: What Should I Buy?

| Budget | Recommendation | Largest Model | Speed (70B) |
|--------|----------------|---------------|-------------|
| $700 | Used RTX 3090 24GB | 13B FP16, 34B Q4 | N/A (OOM) |
| $1,800 | RTX 4090 24GB | 34B Q4, 70B Q2 (bad) | ~20 t/s Q2 |
| $2,500 | RTX 5090 32GB | 40B Q4, 70B Q3 | ~25-30 t/s Q3 |
| $4,500 | RTX A6000 48GB | 70B Q4 | ~18-22 t/s |
| $5,000 | 2x RTX 4090 48GB | 70B Q8 | ~35-40 t/s |
| $6,000 | 2x RTX 5090 64GB | 70B Q8-FP16 | ~50-55 t/s |
| $10,000 | tinybox red v2 (4x 9070 XT) | 70B Q4 | ~15-20 t/s |
| $15,000 | Used A100 80GB | 70B FP16 | ~35 t/s |
| $50,000 | tinybox green v2 (4x PRO 6000) | **405B FP16**, 671B Q4 | ~50-60 t/s |
| $50,000 | tinybox pro v2 (8x 5090)* | 405B Q4 | ~40-50 t/s |
| $50,000 | 2x H100 (NVLink) | 110B FP16 | ~80-100 t/s |
| $150,000+ | 8x H100 DGX | 405B FP16, 671B | ~18-25 t/s (405B) |

*pro v2 has no NVLink - 8-way PCIe limits scaling for large models. Green v2 often better for single huge models despite fewer GPUs.

---

## 🔗 Related Concepts

- [[Off-Grid LLM Power Systems]] - Solar, battery, and generator sizing
- [[LLM Under Your Floorboards]] - Comprehensive local LLM guide
- [[LLM Model Guide]] - Model benchmarks and recommendations
- [[LLM Inference Engines]] - llama.cpp vs vLLM vs Ollama comparison
- [[Local LLMs]] - Overview of local LLM ecosystem
- [[Quantization]] - Reducing model size/memory
- [[GGUF]] - Common quantized format
- [[llama.cpp]] - CPU+GPU inference
- [[vLLM]] - High-throughput serving
- [[TensorRT-LLM]] - NVIDIA optimized inference
- [[NVLink]] - GPU interconnect
- [[CUDA]] - NVIDIA compute platform
- [[ROCm]] - AMD compute platform
- [[RTX 4090]] - Consumer GPU king
- [[RTX 5090]] - New consumer champion
- [[H100]] - Datacenter standard
- [[H200]] - More memory variant
- [[B200]] - Next-gen Blackwell
- [[MI300X]] - AMD datacenter GPU
- [[tinygrad]] - ML framework / tinybox creator
- [[tinybox]] - Pre-built multi-GPU systems
- [[Llama 3]] - Meta's open models
- [[Mixtral]] - MoE models
- [[DeepSeek]] - Chinese MoE models
- [[LoRA]] - Efficient fine-tuning
- [[QLoRA]] - Quantized fine-tuning

---

## 📚 External Resources

- [GPU Benchmarks on LLM Inference (GitHub)](https://github.com/XiongjieDai/GPU-Benchmarks-on-LLM-Inference)
- [LocalLLM.in - Best GPUs for LLM Inference 2025](https://localllm.in/blog/best-gpus-llm-inference-2025)
- [Hardware Corner - GPU Benchmark LLMs](https://www.hardware-corner.net/guides/gpu-benchmark-large-language-models/)
- [NVIDIA TensorRT-LLM Benchmarks](https://nvidia.github.io/TensorRT-LLM/blogs/H100vsA100.html)
- [vLLM Serving on AMD MI300X](https://blog.vllm.ai/2024/10/23/vllm-serving-amd.html)
- [tinygrad / tinybox Documentation](https://docs.tinygrad.org/tinybox/)
- [BIZON AI Workstations](https://bizon-tech.com/)
- [RunPod RTX 5090 Benchmarks](https://www.runpod.io/blog/rtx-5090-llm-benchmarks)
- [Modal - VRAM for Fine-Tuning Guide](https://modal.com/blog/how-much-vram-need-fine-tuning)
- [Oracle - LLM Inference on MI300X](https://blogs.oracle.com/cloud-infrastructure/llm-performance-results-amd-instinct-mi300x-gpus)

