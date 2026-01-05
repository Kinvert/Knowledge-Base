# A100 Builds

A comprehensive guide to building systems around NVIDIA's A100 Tensor Core GPU—the previous-generation datacenter AI workhorse that remains an excellent value in 2025. Covers variants, specifications, what models fit on different configurations, complete build specs, PCIe lane allocation, cooling, power, physical requirements, and pricing.

For consumer GPU builds see [[LLM Under Your Floorboards]]. For comparisons with other datacenter GPUs see [[LLM Inference Hardware]]. For next-generation builds see [[H100 Builds]].

*Pricing last verified: January 2025. GPU prices fluctuate significantly based on supply, demand, and new releases.*

---

## 📍 Where A100 Fits

The A100 is NVIDIA's Ampere architecture datacenter GPU, released in 2020. It was the AI training workhorse for years and remains highly capable for inference workloads in 2025.

| Generation | Architecture | Flagship GPU | VRAM | Release | Status |
|------------|--------------|--------------|------|---------|--------|
| Volta | GV100 | [[V100]] | 16/32GB HBM2 | 2017 | Legacy |
| **Ampere** | **GA100** | **[[A100]]** | **40/80GB HBM2e** | **2020** | **Value tier** |
| Hopper | GH100 | [[H100]] | 80GB HBM3 | 2022 | Current mainstream |
| Hopper Refresh | GH200 | [[H200]] | 141GB HBM3e | 2024 | Premium tier |
| Blackwell | GB100/GB200 | [[B200]] | 192GB HBM3e | 2024-25 | Next generation |

### When to Choose A100

| Scenario | Best Choice | Why |
|----------|-------------|-----|
| Maximum performance, budget flexible | H100 or H200 | 2-3x faster inference |
| Best price/performance for inference | **A100** | 40-60% cheaper than H100, still capable |
| Budget is primary constraint | **A100 (used)** | Used A100 80GB ~$2-3K |
| Training large models | H100 | FP8 support, higher bandwidth |
| Need it deployed this week | **A100** | Abundant availability, mature ecosystem |
| Running 70B models on budget | **A100** | 2x80GB handles 70B FP16 |

### A100 vs V100 vs H100 vs H200

*Sources: [NVIDIA A100 Datasheet](https://www.nvidia.com/content/dam/en-zz/Solutions/Data-Center/a100/pdf/nvidia-a100-datasheet-nvidia-us-2188504-web.pdf), [Hyperstack Benchmark](https://www.hyperstack.cloud/technical-resources/performance-benchmarks/llm-inference-benchmark-comparing-nvidia-a100-nvlink-vs-nvidia-h100-sxm)*

| Spec | V100 32GB | A100 80GB | H100 80GB | H200 141GB |
|------|-----------|-----------|-----------|------------|
| Architecture | Volta | Ampere | Hopper | Hopper |
| VRAM | 32GB HBM2 | 80GB HBM2e | 80GB HBM3 | 141GB HBM3e |
| Memory BW | 900 GB/s | 2.0 TB/s | 3.35 TB/s | 4.8 TB/s |
| FP8 Tensor | N/A | N/A | 3,958 TFLOPS | 3,958 TFLOPS |
| FP16 Tensor | 125 TFLOPS | 312 TFLOPS | 1,979 TFLOPS | 1,979 TFLOPS |
| TDP (SXM) | 300W | 400W | 700W | 700W |
| NVLink | 300 GB/s | 600 GB/s | 900 GB/s | 900 GB/s |
| PCIe Price (new) | ~$3-5K | ~$10-15K | ~$25-35K | ~$35-45K |
| **LLM Inference** | 0.4x A100 | **1x baseline** | ~2x A100 | ~2.5x A100 |

**Bottom line:**
- **V100 → A100**: 2.5x inference performance, 2.2x memory bandwidth, 2.5x VRAM—massive upgrade
- **A100 → H100**: 2x inference performance, 1.7x bandwidth, FP8 support—worth it for production
- **A100 in 2025**: Still excellent for inference, fine-tuning, and budget-conscious training

---

## 🔧 Software Requirements

*Sources: [CUDA 11 Features](https://developer.nvidia.com/blog/cuda-11-features-revealed/), [NVIDIA Driver Documentation](https://docs.nvidia.com/datacenter/tesla/hgx-software-guide/index.html)*

A100 was the first GPU to support CUDA 11 and the Ampere architecture. It has mature software support.

### Minimum Versions

| Component | Minimum | Recommended | Notes |
|-----------|---------|-------------|-------|
| CUDA Toolkit | 11.0 | 12.4+ | First Ampere support |
| NVIDIA Driver | 450.x | 550.x+ | R450 minimum for Ampere |
| cuDNN | 8.0 | 9.x | For deep learning frameworks |
| TensorRT | 7.1 | 10.x | For optimized inference |
| PyTorch | 1.7 | 2.2+ | Native A100 support |
| TensorFlow | 2.4 | 2.15+ | XLA compilation support |

### Framework Considerations

| Framework | A100 Support | Notes |
|-----------|--------------|-------|
| [[PyTorch]] | Excellent (1.7+) | Mature, well-optimized |
| [[TensorFlow]] | Excellent (2.4+) | XLA recommended |
| [[JAX]] | Excellent | Great for research |
| [[vLLM]] | Excellent | Recommended for inference |
| [[TensorRT-LLM]] | Excellent | Best raw performance |
| [[llama.cpp]] | Good | INT8/INT4 quantization |

### Key Difference from H100

A100 does **not** support FP8 precision—only FP16/BF16/TF32/INT8. This means:
- No Transformer Engine FP8 acceleration
- Slightly lower inference throughput for quantized models
- Still excellent for FP16 and INT8 workloads

### Container Images

Pre-built containers with A100 optimization:
- [NGC PyTorch](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/pytorch) - `nvcr.io/nvidia/pytorch:24.01-py3`
- [NGC TensorRT-LLM](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/tensorrt) - For production inference
- [vLLM Docker](https://docs.vllm.ai/en/latest/serving/deploying_with_docker.html) - Easy inference serving

---

## 📋 Quick Reference

| Tier | GPUs | VRAM | Best Models | Tokens/s (70B) | Budget | Form Factor | Location |
|------|------|------|-------------|----------------|--------|-------------|----------|
| 1 | 1x A100 PCIe 80GB | 80GB | 32B FP16, 70B INT8 | ~12 tok/s | $12-18K | Tower/4U | Closet/basement |
| 2 | 2x A100 PCIe 80GB | 160GB | 70B FP16 | ~20 tok/s | $25-35K | 4U | Dedicated room |
| 3 | 4x A100 SXM 80GB | 320GB | 140B FP16, 200B INT8 | ~45 tok/s | $80-120K | 5U | Server room |
| 4 | 8x A100 SXM 80GB | 640GB | 405B INT8 | ~80 tok/s | $150-200K | 6U | Server room |

---

## 🎯 A100 Variants

*Sources: [NVIDIA A100 Datasheet](https://www.nvidia.com/content/dam/en-zz/Solutions/Data-Center/a100/pdf/nvidia-a100-datasheet-nvidia-us-2188504-web.pdf), [A100 80GB Datasheet](https://www.nvidia.com/content/dam/en-zz/Solutions/Data-Center/a100/pdf/a100-80gb-datasheet-update-nvidia-us-1521051-r2-web.pdf)*

| Variant | VRAM | Memory BW | TDP | NVLink BW | Interface | Cooling |
|---------|------|-----------|-----|-----------|-----------|---------|
| **A100 PCIe 40GB** | 40GB HBM2 | 1,555 GB/s | 250W | Optional (bridge) | PCIe 4.0 x16 | Air |
| **A100 PCIe 80GB** | 80GB HBM2e | 2,039 GB/s | 300W | Optional (bridge) | PCIe 4.0 x16 | Air |
| **A100 SXM4 40GB** | 40GB HBM2 | 1,555 GB/s | 400W | 600 GB/s | SXM4 socket | Liquid |
| **A100 SXM4 80GB** | 80GB HBM2e | 2,039 GB/s | 400W | 600 GB/s | SXM4 socket | Liquid |

**Note:** Unlike H100, A100 has no "NVL" PCIe variant with extra VRAM. The 80GB is the maximum.

### Which Variant to Choose

| Use Case | Recommended Variant | Why |
|----------|---------------------|-----|
| Single GPU inference | A100 PCIe 80GB | Easiest integration, sufficient for most models |
| Budget single GPU | A100 PCIe 40GB | 70B at INT4/INT8 fits |
| Paired inference (70B FP16) | 2x A100 PCIe 80GB + NVLink | 160GB total, PCIe bridge available |
| Multi-GPU training | A100 SXM4 80GB | Full NVLink mesh, higher bandwidth |
| Maximum single-node | 8x A100 SXM4 80GB | 640GB, DGX A100 or HGX |

### 40GB vs 80GB Decision

| Model Size | 40GB Sufficient? | 80GB Sufficient? |
|------------|------------------|------------------|
| 7-8B FP16 | Yes | Yes |
| 13B FP16 | Yes | Yes |
| 32B FP16 | Tight | Yes |
| 70B FP16 | No (needs 2x) | No (needs 2x) |
| 70B INT8 | Yes (just fits) | Yes (comfortable) |
| 70B INT4 | Yes | Yes |

**Recommendation:** Always prefer 80GB variants unless budget is extremely tight. The 40GB models are increasingly limiting for modern LLMs.

---

## 🖥️ NVIDIA Platform Overview

*Sources: [ServeTheHome DGX vs HGX](https://www.servethehome.com/nvidia-dgx-versus-nvidia-hgx-what-is-the-difference/), [NVIDIA HGX Platform](https://www.nvidia.com/en-us/data-center/hgx/)*

NVIDIA offers multiple platform tiers for datacenter AI. Understanding these is critical for A100 SXM builds.

| Platform | What It Is | Customization | Target Buyer | A100 Relevance |
|----------|------------|---------------|--------------|----------------|
| **[[DGX]]** | Complete turnkey system | Fixed | Enterprise (plug-and-play) | DGX A100 is the reference 8-GPU system |
| **[[HGX]]** | GPU baseboard + NVSwitch | High (OEM builds around it) | OEMs, cloud providers | Required for SXM builds |
| **[[MGX]]** | Modular server spec | Very high (multi-gen) | System builders | Not applicable (A100 predates MGX) |

### When to Choose Each

| Scenario | Best Platform | Why |
|----------|---------------|-----|
| "I want it working tomorrow" | DGX A100 | Pre-configured, NVIDIA support |
| "I need AMD EPYC CPUs" | HGX + OEM | DGX A100 already uses EPYC |
| "I need specific storage/networking" | HGX + OEM | Customize around the baseboard |
| "Budget is critical" | HGX + self-build or used DGX | Skip new DGX premium |

---

## 🔲 HGX A100 Platform

*Sources: [NVIDIA HGX A100 Blog](https://developer.nvidia.com/blog/introducing-hgx-a100-most-powerful-accelerated-server-platform-for-ai-hpc/), [HGX A100 Datasheet](https://www.nvidia.com/content/dam/en-zz/Solutions/Data-Center/HGX/a100-80gb-hgx-a100-datasheet-us-nvidia-1485640-r6-web.pdf)*

HGX A100 is the GPU baseboard that sits at the heart of all A100 SXM-based systems.

### What's on an HGX Baseboard

| Component | HGX A100 4-GPU | HGX A100 8-GPU |
|-----------|----------------|----------------|
| GPUs | 4x A100 SXM4 | 8x A100 SXM4 |
| VRAM Options | 160GB or 320GB | 320GB or 640GB |
| NVSwitch | None (direct NVLink) | 6x NVSwitch (2nd gen) |
| NVLink per GPU | 600 GB/s | 600 GB/s |
| GPU-GPU Topology | Ring (peer-to-peer) | Full mesh (any-to-any) |
| Fabric Manager | Not required | Required |

### 4-GPU vs 8-GPU: Key Differences

**4-GPU Configuration:**
- GPUs connect directly via NVLink in a ring topology
- Simpler: no NVSwitch, no Fabric Manager software
- Lower cost (~$60-80K for baseboard with 80GB GPUs)
- Good for: inference, fine-tuning, 70B-140B models

**8-GPU Configuration:**
- GPUs connect through 6x NVSwitch chips
- Full mesh: any GPU can talk to any other at 600 GB/s simultaneously
- Higher cost (~$120-150K for baseboard with 80GB GPUs)
- Required for: 200B+ models, large-scale training

### HGX A100 Pricing (2024-2025)

*Sources: [DirectMacro A100 Guide](https://directmacro.com/blog/post/nvidia-a100-in-2025), [Modal A100 Pricing](https://modal.com/blog/nvidia-a100-price-article)*

| Configuration | Baseboard Only | With Typical System | Notes |
|---------------|----------------|---------------------|-------|
| HGX A100 4-GPU 40GB | $40-50K | $70-90K | Limited VRAM |
| HGX A100 4-GPU 80GB | $60-80K | $100-130K | Recommended |
| HGX A100 8-GPU 40GB | $80-100K | $130-160K | DGX Station territory |
| HGX A100 8-GPU 80GB | $120-150K | $180-220K | Approaching used DGX |

*Used HGX baseboards available for 40-60% less*

### Where to Buy HGX A100 Baseboards

**Used/Refurbished (recommended for A100):**
- eBay (search "HGX A100" - many available)
- [ServerMonkey](https://www.servermonkey.com/)
- [VipheraTech](https://viperatech.com/) - New and refurbished
- [Eton Technology](https://etontechnology.com/)

**New (limited availability):**
- [Supermicro](https://store.supermicro.com/)
- Direct from NVIDIA partners

---

## 🔷 DGX A100 Platform

*Sources: [NVIDIA DGX A100 Datasheet](https://www.sifytechnologies.com/wp-content/uploads/2024/09/NVIDIA-DGX-A100-Datasheet.pdf), [Wikipedia DGX](https://en.wikipedia.org/wiki/Nvidia_DGX)*

DGX A100 is NVIDIA's turnkey 8-GPU system—the first DGX to use AMD EPYC processors.

### DGX A100 Specifications

| Spec | DGX A100 |
|------|----------|
| GPUs | 8x A100 SXM4 80GB |
| GPU Memory | 640GB HBM2e |
| NVLink | 600 GB/s per GPU, 6x NVSwitch |
| CPUs | Dual AMD EPYC 7742 (64c each, 128c total) |
| System RAM | 2TB DDR4 |
| Storage | 15TB NVMe (2x 1.92TB M.2 + 8x 3.84TB U.2) |
| Networking | 8x Mellanox ConnectX-6 HDR 200Gb |
| Power | 6.5 kW typical |
| Dimensions | 6U rackmount |
| Weight | 271 lbs (123 kg) |
| **Original MSRP** | $199,000 |
| **Current Price (new)** | $150,000 - $200,000 |
| **Current Price (used)** | $80,000 - $120,000 |

### DGX Station A100 (Desktop)

| Spec | DGX Station A100 320GB | DGX Station A100 160GB |
|------|------------------------|------------------------|
| GPUs | 4x A100 80GB | 4x A100 40GB |
| GPU Memory | 320GB | 160GB |
| CPUs | AMD Threadripper Pro 3975WX (32c) | Same |
| System RAM | 512GB DDR4 | Same |
| Power | 1.5 kW typical | Same |
| **Original MSRP** | $149,000 | $99,000 |
| **Current Price** | $80,000 - $100,000 | $50,000 - $70,000 |

### DGX A100 Pricing Reality (2025)

| What You Get | New | Used/Refurb |
|--------------|-----|-------------|
| DGX A100 640GB (8-GPU) | $150-200K | $80-120K |
| DGX Station A100 320GB (4-GPU) | $100-130K | $60-80K |
| DGX Station A100 160GB (4-GPU) | $70-90K | $40-60K |

**The A100 advantage:** Used DGX A100 systems are abundant and deeply discounted. A used DGX A100 at $100K offers similar VRAM to a new DGX H100 at $400K, at 1/4 the price (though ~1/2 the performance).

---

## 📊 Model-to-Hardware Mapping

*Sources: [Hyperstack VRAM Guide](https://www.hyperstack.cloud/blog/case-study/how-much-vram-do-you-need-for-llms), [dlewis.io Llama Benchmark](https://dlewis.io/evaluating-llama-33-70b-inference-h100-a100/)*

### VRAM Requirements

| Model | FP16 | INT8 | INT4 | Notes |
|-------|------|------|------|----|
| Llama 3.1 8B | 16GB | 8GB | 4GB | Single GPU easy |
| Qwen2.5 32B | 64GB | 32GB | 18GB | Fits 1x A100 80GB |
| Llama 3.1 70B | 140GB | 70GB | 35GB | 2x A100 80GB at FP16 |
| Llama 3.1 70B + 128K ctx | 180GB | 90GB | 55GB | 3-4x A100 80GB |
| Mixtral 8x22B | 176GB | 88GB | 44GB | 3x A100 80GB |
| Llama 3.1 405B | 810GB | 405GB | 200GB | 8x A100 tight at INT8 |
| DeepSeek R1 671B | 1.3TB | 670GB | 335GB | Beyond single DGX A100 |

*Add ~20% overhead for activations and KV cache*

### Minimum A100 Count by Model

| Model | FP16 | INT8 | INT4 | Optimal Config |
|-------|------|------|------|----------------|
| 8B | 1 | 1 | 1 | 1x any variant |
| 32B | 1 | 1 | 1 | 1x 80GB |
| 70B | 2 | 1 | 1 | 2x PCIe 80GB or 4x SXM |
| 70B + long context | 4 | 2 | 1 | 4x SXM |
| 140B | 4 | 2 | 2 | 4x SXM 80GB |
| 200B | 8 | 4 | 2 | 8x SXM (DGX) |
| 405B | 16+ | 8 | 4 | 8x SXM + aggressive quant |

### A100 LLM Performance

*Sources: [Ori Benchmarks](https://www.ori.co/blog/benchmarking-llama-3.1-8b-instruct-on-nvidia-h100-and-a100-chips-with-the-vllm-inferencing-engine), [Hyperstack Benchmark](https://www.hyperstack.cloud/technical-resources/performance-benchmarks/llm-inference-benchmark-comparing-nvidia-a100-nvlink-vs-nvidia-h100-sxm)*

| Model | Config | Precision | Tok/s (batch 1) | Tok/s (batch 32) |
|-------|--------|-----------|-----------------|------------------|
| Llama 3.1 8B | 1x A100 80GB | FP16 | ~60 | ~700 |
| Llama 3.1 70B | 2x A100 80GB | FP16 | ~15 | ~150 |
| Llama 3.1 70B | 4x A100 80GB | FP16 | ~25 | ~300 |
| Llama 3.1 70B | 4x A100 80GB | INT8 | ~35 | ~400 |
| Llama 3.1 405B | 8x A100 80GB | INT8 | ~8 | ~80 |

**Comparison:** H100 delivers ~2-3x these token rates for the same configurations.

---

## 🔧 Tier 1: Single A100 PCIe (~$12-18K)

### Overview
Entry point for datacenter-class inference. Runs 32B models at full precision, 70B at INT8/INT4. Excellent value in 2025 with used prices under $3K.

### Bill of Materials

| Component | Recommendation | Price | Link |
|-----------|----------------|-------|------|
| GPU | 1x A100 PCIe 80GB | $10-15K new / $2-3K used | [NVIDIA A100](https://www.nvidia.com/en-us/data-center/a100/) |
| CPU | AMD EPYC 7313 (16c/32t) | $800 | [[AMD EPYC]] |
| Motherboard | Supermicro H12SSL-i | $600 | [[Supermicro]] |
| RAM | 256GB DDR4-3200 ECC (8x32GB) | $800 | |
| Storage | 2TB NVMe Gen4 | $200 | |
| PSU | 1200W 80+ Platinum | $250 | |
| Case | 4U rackmount or tower | $300-500 | |
| **Total (new GPU)** | | **$13-18K** | |
| **Total (used GPU)** | | **$5-7K** | |

### PCIe Lane Allocation

**CPU:** AMD EPYC 7003 provides **128 PCIe 4.0 lanes**

| Component | Lanes | Bandwidth | Remaining |
|-----------|-------|-----------|-----------|
| A100 GPU | x16 | 64 GB/s | 112 |
| Boot NVMe | x4 | 8 GB/s | 108 |
| Data NVMe | x4 | 8 GB/s | 104 |
| 25GbE NIC | x8 | 32 GB/s | 96 |
| **Available** | | | **96 lanes** |

### Model Performance

| Model | Precision | VRAM Used | Tok/s (batch 1) | Tok/s (batch 16) |
|-------|-----------|-----------|-----------------|------------------|
| Llama 3.1 8B | FP16 | 16GB | ~60 | ~700 |
| Qwen2.5 32B | FP16 | 64GB | ~20 | ~200 |
| Llama 3.1 70B | INT8 | 70GB | ~12 | ~100 |
| Llama 3.1 70B | INT4 | 35GB | ~15 | ~130 |

**Sweet spot:** 32B at FP16 or 70B at INT8/INT4
**Maximum:** 70B at INT8 with limited context

### Physical Specifications

| Spec | Value |
|------|-------|
| Form factor | Tower workstation or 4U rackmount |
| GPU TDP | 300W |
| System power | 500-700W typical |
| Noise | 40-50 dB |
| Heat output | 1,700-2,400 BTU/hr |
| **Location** | Large closet, basement, home office |

### Power Requirements

| State | Power Draw | Circuit |
|-------|------------|---------|
| Idle | 150W | Standard 15A/120V OK |
| Typical | 400W | Standard 15A/120V OK |
| Peak | 600-700W | 15A/120V OK |

Recommended: Dedicated 15A circuit, 1000VA UPS

### Where to Buy

**New:**
- [Supermicro GPU Workstations](https://www.supermicro.com/en/products/gpu)
- [Thinkmate GPU Workstations](https://www.thinkmate.com/systems/workstations/gpx)
- [Exxact Workstations](https://www.exxactcorp.com/gpu-workstations)

**Used (recommended for A100):**
- eBay (search "A100 PCIe 80GB" - many under $3K)
- [ServerMonkey](https://www.servermonkey.com/)

---

## 🔧 Tier 2: Dual A100 PCIe (~$25-35K)

### Overview
The sweet spot for 70B models at full precision. NVLink bridge provides 600 GB/s GPU-to-GPU bandwidth. Excellent for production inference and fine-tuning.

### Bill of Materials

| Component | Recommendation | Price | Link |
|-----------|----------------|-------|------|
| GPUs | 2x A100 PCIe 80GB + NVLink bridge | $20-30K new / $5-7K used | [NVIDIA A100](https://www.nvidia.com/en-us/data-center/a100/) |
| CPU | AMD EPYC 7343 (16c/32t) | $1,200 | [AMD EPYC 7003](https://www.amd.com/en/products/processors/server/epyc/7003-series.html) |
| Motherboard | Supermicro H12DSG-O-CPU | $1,200 | [Supermicro](https://store.supermicro.com/) |
| RAM | 512GB DDR4-3200 ECC | $1,600 | |
| Storage | 4TB NVMe Gen4 RAID | $400 | |
| PSU | 1600W redundant | $600 | |
| Case | 4U rackmount | $400 | |
| **Total (new GPUs)** | | **$26-36K** | |
| **Total (used GPUs)** | | **$11-14K** | |

### PCIe Lane Allocation

**CPU:** AMD EPYC 7343 provides **128 PCIe 4.0 lanes**

| Component | Lanes | Bandwidth | Remaining |
|-----------|-------|-----------|-----------|
| A100 GPU #1 | x16 | 64 GB/s | 112 |
| A100 GPU #2 | x16 | 64 GB/s | 96 |
| NVMe RAID (2 drives) | x8 | 16 GB/s | 88 |
| 100GbE NIC | x16 | 64 GB/s | 72 |
| **Available** | | | **72 lanes** |

### Model Performance

| Model | Precision | VRAM Used | Tok/s (batch 1) | Tok/s (batch 32) |
|-------|-----------|-----------|-----------------|------------------|
| Llama 3.1 70B | FP16 | 140GB | ~15 | ~150 |
| Llama 3.1 70B | INT8 | 70GB | ~25 | ~250 |
| Llama 3.1 70B + 64K | FP16 | 155GB | ~12 | ~120 |
| Mixtral 8x22B | INT8 | 88GB | ~18 | ~180 |

**Sweet spot:** 70B at FP16 with 32K context
**Maximum:** Mixtral 8x22B at INT8

### Physical Specifications

| Spec | Value |
|------|-------|
| Form factor | 4U rackmount |
| GPU TDP | 600W (2x 300W) |
| System power | 900-1,200W typical |
| Noise | 50-60 dB |
| Heat output | 3,000-4,000 BTU/hr |
| **Location** | Dedicated room, basement server area |

### Power Requirements

| State | Power Draw | Circuit |
|-------|------------|---------|
| Idle | 300W | 20A/120V |
| Typical | 800W | 20A/120V |
| Peak | 1,100-1,300W | 20A/120V |

Recommended: Dedicated 20A/120V circuit, 1500VA UPS

---

## 🔧 Tier 3: 4x A100 SXM / HGX 4-GPU (~$80-120K)

### Overview
Four SXM GPUs with direct NVLink connectivity. Good for 140B models at FP16 or 405B at aggressive quantization. This is DGX Station A100 territory.

### Bill of Materials

| Component | Recommendation | Price | Link |
|-----------|----------------|-------|------|
| GPUs | 4x A100 SXM4 80GB (HGX baseboard) | $60-80K | [NVIDIA HGX](https://www.nvidia.com/en-us/data-center/hgx/) |
| CPUs | Dual AMD EPYC 7543 (32c each) | $6,000 | [AMD EPYC 7003](https://www.amd.com/en/products/processors/server/epyc/7003-series.html) |
| Motherboard | HGX-compatible | incl. | |
| RAM | 1TB DDR4-3200 ECC | $3,200 | |
| Storage | 8TB NVMe RAID | $1,000 | |
| PSU | 3000W+ | $1,500 | |
| Networking | ConnectX-6 HDR 200GbE | $2,000 | |
| Cooling | **Liquid cooling required** | $8-12K | |
| **Total** | | **$85-115K** | |

**Alternative:** Used DGX Station A100 320GB for $60-80K

### PCIe Lane Allocation

**HGX Configuration:** GPUs connect via NVLink, not PCIe. CPU lanes for peripherals only.

| Component | Lanes | Bandwidth | Remaining |
|-----------|-------|-----------|-----------|
| NVMe RAID | x16 | 32 GB/s | 112 |
| 200GbE NIC #1 | x16 | 64 GB/s | 96 |
| 200GbE NIC #2 | x16 | 64 GB/s | 80 |
| **Available** | | | **80 lanes** |

GPUs use NVLink (600 GB/s per GPU), not PCIe lanes.

### Model Performance

| Model | Precision | VRAM Used | Tok/s (batch 1) | Tok/s (batch 64) |
|-------|-----------|-----------|-----------------|------------------|
| Llama 3.1 70B | FP16 | 140GB | ~35 | ~400 |
| Llama 3.1 70B | INT8 | 70GB | ~50 | ~550 |
| Mixtral 8x22B | FP16 | 176GB | ~25 | ~280 |
| Llama 3.1 200B* | INT8 | 200GB | ~15 | ~150 |
| Llama 3.1 405B | INT4 | 200GB | ~8 | ~70 |

*Hypothetical model size

**Sweet spot:** 70B with maximum throughput, 140B development
**Maximum:** 200B at INT8 or 405B at INT4

### Physical Specifications

| Spec | Value |
|------|-------|
| Form factor | 5U rackmount or DGX Station tower |
| GPU TDP | 1,600W (4x 400W) |
| System power | 2,500-3,000W typical |
| Noise | 55-70 dB (rack) / 45-55 dB (station) |
| Heat output | 8,500-10,000 BTU/hr |
| Cooling | **Liquid cooling required for rack** |
| **Location** | Server room or dedicated space |

### Power Requirements

| State | Power Draw | Circuit |
|-------|------------|---------|
| Idle | 600W | 30A/240V |
| Typical | 2,200W | 30A/240V |
| Peak | 2,800-3,200W | 30A/240V |

Recommended: 30A/240V circuit, 4000VA UPS

---

## 🔧 Tier 4: 8x A100 SXM / DGX A100 (~$150-200K)

### Overview
Full HGX baseboard with 8 GPUs and NVSwitch fabric. This is the configuration for 405B models at INT8 and large-scale training. DGX A100 is the turnkey option.

### Bill of Materials (DIY)

| Component | Recommendation | Price | Link |
|-----------|----------------|-------|------|
| GPUs | 8x A100 SXM4 80GB (HGX baseboard + 6x NVSwitch) | $120-150K | [NVIDIA HGX A100](https://www.nvidia.com/en-us/data-center/hgx/) |
| CPUs | Dual AMD EPYC 7742 (64c each) | $10,000 | [AMD EPYC 7003](https://www.amd.com/en/products/processors/server/epyc/7003-series.html) |
| RAM | 2TB DDR4 ECC | $6,400 | |
| Storage | 15TB NVMe RAID | $3,000 | |
| PSU | 4.5kW+ redundant | $2,500 | |
| Networking | 8x ConnectX-6 HDR 200Gb | $8,000 | |
| Cooling | **Liquid cooling (CDU + manifolds)** | $15-20K | |
| **Total** | | **$170-210K** | |

### DGX A100 (Turnkey Alternative)

| Spec | DGX A100 |
|------|----------|
| GPUs | 8x A100 SXM4 80GB |
| GPU Memory | 640GB total |
| NVLink | 600 GB/s per GPU, 6x NVSwitch |
| CPUs | Dual AMD EPYC 7742 (64c each, 128c total) |
| System RAM | 2TB DDR4 |
| Storage | 15TB NVMe |
| Networking | 8x ConnectX-6 HDR 200Gb |
| Power | 6.5kW typical |
| Price (new) | **$150,000 - $200,000** |
| Price (used) | **$80,000 - $120,000** |
| Link | [NVIDIA DGX](https://www.nvidia.com/en-us/data-center/dgx-a100/) |

### PCIe Lane Allocation

GPUs use NVSwitch, not PCIe. All 128 lanes available for peripherals.

### Model Performance

| Model | Precision | VRAM Used | Tok/s (batch 1) | Tok/s (batch 64) |
|-------|-----------|-----------|-----------------|------------------|
| Llama 3.1 70B | FP16 | 140GB | ~60 | ~700 |
| Llama 3.1 70B | INT8 | 70GB | ~80 | ~900 |
| Mixtral 8x22B | FP16 | 176GB | ~45 | ~500 |
| Llama 3.1 405B | INT8 | 405GB | ~8 | ~80 |
| Llama 3.1 405B | INT4 | 200GB | ~12 | ~120 |
| DeepSeek R1 671B | INT4 | 335GB | ~5 | ~50 |

**Sweet spot:** 405B at INT8 with 16-32K context
**Maximum:** 671B at INT4 (tight fit)

### Physical Specifications

| Spec | Value |
|------|-------|
| Form factor | 6U rackmount |
| Dimensions | 19" W × 10.5" H × 35" D |
| Weight | 271 lbs (123 kg) |
| Noise | 70-80 dB |
| Heat output | 18,000-22,000 BTU/hr |
| Rack depth | Requires 42"+ deep rack |
| Cooling | **Liquid cooling (DGX has integrated)** |
| **Location** | Server room / datacenter |

### Power Requirements

| State | Power Draw | Circuit |
|-------|------------|---------|
| Idle | 1,000W | Multiple 30A/240V |
| Typical | 5,000W | Multiple 30A/240V |
| Peak | 6,500-7,000W | 2x 30A/240V |

**Infrastructure requirements:**
- 3-phase power recommended
- Industrial PDU with C19/C20
- 10kVA+ UPS
- Coolant Distribution Unit (CDU) rated for 8kW+

### Where to Buy

**New:**
- Direct from NVIDIA
- [Dell](https://www.dell.com/) - OEM systems
- [HPE](https://www.hpe.com/) - OEM systems
- [Supermicro](https://www.supermicro.com/en/products/system/gpu/)

**Used (excellent value):**
- eBay (search "DGX A100" - often $80-120K)
- [ServerMonkey](https://www.servermonkey.com/)
- Enterprise lease returns

---

## ⚡ Power Infrastructure

### Power by Configuration

| Config | GPU Power | System Total | Heat (BTU/hr) | Circuit |
|--------|-----------|--------------|---------------|---------|
| 1x PCIe 80GB | 300W | 500-700W | 1,700-2,400 | 15A/120V |
| 2x PCIe 80GB | 600W | 900-1,200W | 3,000-4,000 | 20A/120V |
| 4x SXM 80GB | 1,600W | 2,500-3,000W | 8,500-10,000 | 30A/240V |
| 8x SXM 80GB | 3,200W | 5,000-6,500W | 17,000-22,000 | 2x 30A/240V |

### UPS Sizing

| System Power | UPS VA Rating | Runtime (15 min) |
|--------------|---------------|------------------|
| 700W | 1,000 VA | Standard |
| 1,200W | 1,800 VA | Rackmount |
| 3,000W | 4,500 VA | Rackmount |
| 6,500W | 10,000 VA | 3-phase |

---

## 🌡️ Cooling Requirements

### Cooling Method by Configuration

| Config | Cooling | Heat Load | Notes |
|--------|---------|-----------|-------|
| 1x PCIe | Air | 500-700W | Standard case fans |
| 2x PCIe | Air | 900-1,200W | High-airflow case |
| 4x SXM | **Liquid** | 2.5-3kW | Direct-to-chip cooling |
| 8x SXM | **Liquid** | 5-6.5kW | CDU infrastructure |

### A100 vs H100 Cooling Advantage

A100 SXM runs at 400W vs H100's 700W. This means:
- 43% less heat per GPU
- Easier cooling infrastructure
- Lower power costs
- DGX A100 draws ~6.5kW vs DGX H100's ~10.2kW

---

## 💰 Total Cost of Ownership (3-Year)

### Electricity Costs

*Assuming $0.12/kWh, 24/7 operation*

| Config | Power | Monthly | 3-Year |
|--------|-------|---------|--------|
| 1x PCIe | 500W avg | $43 | $1,550 |
| 2x PCIe | 900W avg | $78 | $2,800 |
| 4x SXM | 2.5kW avg | $216 | $7,780 |
| 8x SXM | 5.5kW avg | $475 | $17,100 |

### Total 3-Year TCO

| Config | Hardware (new) | Hardware (used) | Electricity | Cooling | Support | **Total (new)** | **Total (used)** |
|--------|----------------|-----------------|-------------|---------|---------|-----------------|------------------|
| 1x PCIe | $15K | $5K | $1.6K | $0.5K | $2K | **$19K** | **$9K** |
| 2x PCIe | $30K | $12K | $2.8K | $0.9K | $3K | **$37K** | **$19K** |
| 4x SXM | $100K | $60K | $7.8K | $2.5K | $8K | **$118K** | **$78K** |
| 8x SXM | $180K | $100K | $17.1K | $5K | $15K | **$217K** | **$137K** |

### Cloud Break-Even

| Provider | $/hr/A100 | Break-even vs Tier 2 (new) | Break-even vs Tier 4 (used) |
|----------|-----------|----------------------------|------------------------------|
| Thunder Compute | $0.66 | 26 months | 18 months |
| Lambda | $1.10 | 16 months | 11 months |
| AWS | $3.00 | 6 months | 4 months |

**A100 value proposition:** With used A100s at 20-30% of original price, break-even against cloud is very fast. A used DGX A100 at $100K breaks even against AWS in ~4 months at full utilization.

---

## 🔗 Related Concepts

**GPUs:**
- [[A100]] - The GPU itself (specs, architecture)
- [[V100]] - Previous generation (Volta)
- [[H100]] - Next generation (Hopper)
- [[H200]] - Latest Hopper with 141GB

**NVIDIA Platforms:**
- [[DGX]] - Turnkey AI supercomputer systems
- [[HGX]] - GPU baseboard for OEM builds
- [[NVLink]] - GPU interconnect technology
- [[NVSwitch]] - Multi-GPU fabric switch

**System Components:**
- [[AMD EPYC]] - Server CPU option (DGX A100 uses 7742)
- [[ConnectX-6]] - InfiniBand/Ethernet networking
- [[InfiniBand]] - Low-latency interconnect

**Related Guides:**
- [[H100 Builds]] - Next-generation builds
- [[LLM Inference Hardware]] - GPU comparison overview
- [[LLM Under Your Floorboards]] - Consumer GPU builds
- [[Quantization]] - Reducing VRAM requirements
- [[vLLM]] - Inference engine
- [[TensorRT-LLM]] - NVIDIA's inference engine

---

## 📚 External Resources

### Official Documentation
- [NVIDIA A100 Product Page](https://www.nvidia.com/en-us/data-center/a100/)
- [A100 Datasheet (40GB)](https://www.nvidia.com/content/dam/en-zz/Solutions/Data-Center/a100/pdf/nvidia-a100-datasheet-nvidia-us-2188504-web.pdf)
- [A100 80GB Datasheet](https://www.nvidia.com/content/dam/en-zz/Solutions/Data-Center/a100/pdf/a100-80gb-datasheet-update-nvidia-us-1521051-r2-web.pdf)
- [HGX A100 Datasheet](https://www.nvidia.com/content/dam/en-zz/Solutions/Data-Center/HGX/a100-80gb-hgx-a100-datasheet-us-nvidia-1485640-r6-web.pdf)
- [DGX A100 Datasheet](https://www.sifytechnologies.com/wp-content/uploads/2024/09/NVIDIA-DGX-A100-Datasheet.pdf)

### Pricing and Availability
- [Modal A100 Pricing Guide](https://modal.com/blog/nvidia-a100-price-article)
- [DirectMacro A100 in 2025](https://directmacro.com/blog/post/nvidia-a100-in-2025)
- [Clarifai A100 Guide](https://www.clarifai.com/blog/nvidia-a100)

### Benchmarks
- [Hyperstack A100 vs H100 Benchmark](https://www.hyperstack.cloud/technical-resources/performance-benchmarks/llm-inference-benchmark-comparing-nvidia-a100-nvlink-vs-nvidia-h100-sxm)
- [Ori Llama Benchmarks](https://www.ori.co/blog/benchmarking-llama-3.1-8b-instruct-on-nvidia-h100-and-a100-chips-with-the-vllm-inferencing-engine)
- [dlewis.io Llama 70B Evaluation](https://dlewis.io/evaluating-llama-33-70b-inference-h100-a100/)

### System Vendors
- [Supermicro GPU Servers](https://www.supermicro.com/en/accelerators/)
- [Supermicro Store](https://store.supermicro.com/)
- [Thinkmate GPU Servers](https://www.thinkmate.com/systems/servers/gpx)
- [Exxact GPU Servers](https://www.exxactcorp.com/gpu-servers)

### Used Market
- [ServerMonkey](https://www.servermonkey.com/)
- [VipheraTech](https://viperatech.com/)
- [eBay Enterprise Servers](https://www.ebay.com/b/Enterprise-Servers/11211/)

<!--
================================================================================
TEMPLATE NOTES: A100 BUILDS
================================================================================

This document follows the H100 Builds template with A100-specific adaptations.

KEY DIFFERENCES FROM H100 BUILDS:
- 4 tiers instead of 5 (no NVL variant)
- PCIe 4.0 instead of 5.0
- NVLink 3 (600 GB/s) instead of NVLink 4 (900 GB/s)
- NVSwitch 2nd gen (6 per 8-GPU) instead of 3rd gen (4 per 8-GPU)
- No FP8 support (FP16/INT8/INT4 only)
- Lower TDP: 300W PCIe, 400W SXM (vs 350W/700W for H100)
- Strong emphasis on used market value

PRICING NOTES:
- Used A100 80GB PCIe: $2-3K (incredible value)
- Used DGX A100: $80-120K (vs $400K+ for new DGX H100)
- A100 is now the "budget datacenter GPU" sweet spot

================================================================================
-->
