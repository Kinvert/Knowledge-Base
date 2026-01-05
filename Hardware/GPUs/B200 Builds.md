# B200 Builds

A comprehensive guide to building systems around NVIDIA's Blackwell B200 Tensor Core GPU—the next-generation datacenter AI accelerator with 192GB HBM3e, 8 TB/s bandwidth, and NVLink 5. Covers variants, specifications, what models fit on different configurations, complete build specs, cooling, power, physical requirements, and pricing.

For consumer GPU builds see [[LLM Under Your Floorboards]]. For comparisons with other datacenter GPUs see [[LLM Inference Hardware]].

*Pricing last verified: January 2025. B200 is newly shipping with significant supply constraints. Prices and availability are volatile.*

---

## 📍 Where B200 Fits

The B200 is NVIDIA's Blackwell architecture datacenter GPU, announced in March 2024 and shipping Q1 2025. It represents a generational leap over Hopper with 2x compute, 2.4x VRAM, and 2x NVLink bandwidth.

| Generation | Architecture | Flagship GPU | VRAM | Release | Status |
|------------|--------------|--------------|------|---------|--------|
| Ampere | GA100 | [[A100]] | 40/80GB HBM2e | 2020 | Previous gen, discounted |
| Hopper | GH100 | [[H100]] | 80GB HBM3 | 2022 | Mainstream datacenter |
| Hopper Refresh | GH100 | [[H200]] | 141GB HBM3e | 2024 | Premium Hopper |
| **Blackwell** | **GB100/GB200** | **[[B200]]** | **192GB HBM3e** | **2025** | **Current flagship** |
| Blackwell Ultra | GB300 | [[B300]] | 288GB HBM3e | Late 2025 | Next refresh |

### When to Choose B200

| Scenario | Best Choice | Why |
|----------|-------------|-----|
| Maximum single-GPU performance | **B200** | 2x H200 compute, 8 TB/s bandwidth |
| Frontier model training (1T+ params) | **B200** | NVLink 5, larger VRAM, better scaling |
| Latency-critical inference | **B200** | Sub-3ms latency achievable |
| Budget-constrained, need now | H200 or H100 | B200 supply constrained, 12+ month waits |
| Existing infrastructure reuse | H200 | B200 requires new cooling/power infrastructure |
| Planning for 2026+ | **B200** | Future-proofing, will become mainstream |

### B200 vs H200 vs H100 vs A100

| Spec | A100 80GB | H100 80GB | H200 141GB | B200 192GB |
|------|-----------|-----------|------------|------------|
| Architecture | Ampere | Hopper | Hopper | Blackwell |
| VRAM | 80GB HBM2e | 80GB HBM3 | 141GB HBM3e | 192GB HBM3e |
| Memory BW | 2.0 TB/s | 3.35 TB/s | 4.8 TB/s | 8.0 TB/s |
| FP8 Tensor | N/A | 3,958 TFLOPS | 3,958 TFLOPS | ~9,000 TFLOPS |
| FP16 Tensor | 312 TFLOPS | 1,979 TFLOPS | 1,979 TFLOPS | ~4,500 TFLOPS |
| FP4 Tensor | N/A | N/A | N/A | ~20,000 TFLOPS |
| TDP (SXM) | 400W | 700W | 700W | 1,000W |
| NVLink | 600 GB/s | 900 GB/s | 900 GB/s | 1,800 GB/s |
| NVSwitch Gen | 2nd | 3rd | 3rd | 4th |
| Est. Price | ~$12-18K | ~$25-35K | ~$35-45K | ~$45-55K |
| **LLM Inference** | 1x baseline | ~2x A100 | ~2.5x A100 | **~5-6x A100** |

**Bottom line:**
- **H200 → B200**: 2.3x compute, 36% more VRAM, 67% more bandwidth—generational leap
- **B200 is ~2x H200** for most workloads, ~15x H100 for some inference tasks
- **B200 requires new infrastructure**: 1000W TDP, NVLink 5, different cooling

---

## 🔧 Software Requirements

*Sources: [NVIDIA CUDA Toolkit](https://developer.nvidia.com/cuda-toolkit), [Blackwell Architecture Guide](https://resources.nvidia.com/en-us-dgx-systems/dgx-b200-datasheet)*

B200 requires CUDA 12.4+ and updated drivers. FP4 support requires the latest frameworks.

### Minimum Versions

| Component | Minimum | Recommended | Notes |
|-----------|---------|-------------|-------|
| CUDA Toolkit | 12.4 | 12.6+ | FP4 requires CUDA 12.6 |
| NVIDIA Driver | 550.x | 560.x+ | Blackwell support |
| cuDNN | 9.0 | 9.x | For deep learning frameworks |
| TensorRT | 10.0 | 10.3+ | FP4/FP8 optimizations |
| PyTorch | 2.3 | 2.4+ | Native Blackwell support |
| TensorFlow | 2.16 | 2.17+ | XLA compilation support |

### Framework Considerations

| Framework | B200 Support | FP4 Support | FP8 Support | Notes |
|-----------|--------------|-------------|-------------|-------|
| [[PyTorch]] | Native (2.3+) | Via Transformer Engine | Yes | Best flexibility |
| [[TensorFlow]] | Native (2.16+) | Limited | Yes | XLA required |
| [[JAX]] | Native | Via Transformer Engine | Yes | Good for research |
| [[vLLM]] | Excellent | Coming | Yes | Recommended for inference |
| [[TensorRT-LLM]] | Excellent | Yes | Yes | Best raw performance |
| [[llama.cpp]] | Good | No | No (INT8/INT4 only) | CPU offload capable |

### Container Images

Pre-built containers with B200 optimization:
- [NGC PyTorch](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/pytorch) - `nvcr.io/nvidia/pytorch:25.01-py3`
- [NGC TensorRT-LLM](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/tensorrt) - For production inference
- [vLLM Docker](https://docs.vllm.ai/en/latest/serving/deploying_with_docker.html) - Easy inference serving

---

## 📋 Quick Reference

| Tier | GPUs | VRAM | Best Models | Tokens/s (70B) | Budget | Form Factor | Location |
|------|------|------|-------------|----------------|--------|-------------|----------|
| 1 | 2x B200 (GB200) | 384GB | 200B FP16, 400B FP8 | ~200 tok/s | $150-200K | 2U-4U | Server room |
| 2 | 4x B200 SXM | 768GB | 405B FP16, 600B FP8 | ~350 tok/s | $280-350K | 5U | Server room |
| 3 | 8x B200 SXM (air) | 1.5TB | 1T FP8, 671B FP16 | ~600 tok/s | $500-600K | 8U | Datacenter |
| 4 | 8x B200 SXM (DGX) | 1.5TB | 1T FP8, 671B FP16 | ~650 tok/s | $550-650K | 8U | Datacenter |
| 5 | 72x B200 (NVL72) | 13.8TB | Multi-trillion | ~5,000+ tok/s | $2.5-3.5M | Rack | Datacenter |

**Note:** B200 is SXM-only (no PCIe variant). Entry point is GB200 Superchip (2x B200 + Grace CPU) or HGX systems.

---

## 🎯 B200 Variants

*Sources: [NVIDIA B200 Datasheet](https://www.primeline-solutions.com/media/categories/server/nach-gpu/nvidia-hgx-h200/nvidia-blackwell-b200-datasheet.pdf), [Lenovo B200 Product Guide](https://lenovopress.lenovo.com/lp2226-thinksystem-nvidia-b200-180gb-1000w-gpu), [NVIDIA DGX B200](https://www.nvidia.com/en-us/data-center/dgx-b200/)*

| Variant | VRAM | Memory BW | TDP | NVLink BW | Form Factor | Cooling |
|---------|------|-----------|-----|-----------|-------------|---------|
| **B200 SXM6** | 192GB HBM3e | 8,000 GB/s | 1,000W | 1,800 GB/s | SXM socket | Air or Liquid |
| **GB200 Superchip** | 384GB (2x B200) | 16,000 GB/s | 2,700W | 1,800 GB/s each | Compute tray | Liquid |
| **GB200 NVL72** | 13.8TB (72x B200) | 576 TB/s | ~120kW | 130 TB/s total | Full rack | Liquid |

**Key architectural changes from Hopper:**
- **Dual-die design**: 208 billion transistors across two dies connected at 10 TB/s
- **NVLink 5**: 1.8 TB/s per GPU (2x Hopper)
- **Reduced NVSwitch count**: 2 NVSwitch ASICs per HGX (vs 4 in Hopper)
- **FP4 Tensor Cores**: New precision for inference workloads
- **On-package NVSwitch**: For GB200 Superchip configurations

### Which Configuration to Choose

| Use Case | Recommended Config | Why |
|----------|---------------------|-----|
| Production inference (405B) | 8x B200 (HGX/DGX) | 1.5TB fits 405B FP16 with headroom |
| Maximum single-system perf | DGX B200 | Turnkey, validated, NVIDIA support |
| Training at scale | GB200 NVL72 | 72-GPU NVLink domain, 1.4 exaFLOPS |
| Entry to Blackwell | GB200 Superchip | 2x B200 + Grace, integrated |
| Custom OEM build | HGX B200 | Build around baseboard |

---

## 🖥️ NVIDIA Platform Overview

*Sources: [NVIDIA HGX Platform](https://www.nvidia.com/en-us/data-center/hgx/), [NVIDIA DGX B200](https://www.nvidia.com/en-us/data-center/dgx-b200/), [NVIDIA GB200 NVL72](https://www.nvidia.com/en-us/data-center/gb200-nvl72/)*

| Platform | What It Is | GPUs | Target Buyer | B200 Relevance |
|----------|------------|------|--------------|----------------|
| **[[DGX]]** | Complete turnkey system | 8x B200 | Enterprise (plug-and-play) | DGX B200 is the reference 8-GPU system |
| **[[HGX]]** | GPU baseboard + NVSwitch | 8x B200 | OEMs, cloud providers | Required for custom SXM builds |
| **GB200 Superchip** | 2x B200 + Grace CPU | 2x B200 | Entry Blackwell | Compute tray building block |
| **GB200 NVL72** | Rack-scale system | 72x B200 | Hyperscale, frontier AI | Maximum single-domain performance |
| **[[MGX]]** | Modular server spec | Various | System builders | Future-proofing platform |

### When to Choose Each

| Scenario | Best Platform | Why |
|----------|---------------|-----|
| "I want it working this quarter" | DGX B200 | Pre-configured, NVIDIA support |
| "I need custom CPU/storage" | HGX B200 | Build around baseboard |
| "Entry to Blackwell, limited budget" | GB200 Superchip | 2x B200 starting point |
| "Training frontier models" | GB200 NVL72 | 72-GPU unified memory domain |
| "Cloud infrastructure" | HGX B200 | Standard for cloud providers |

---

## 🔲 HGX B200 Platform Deep Dive

*Sources: [NVIDIA HGX Platform](https://www.nvidia.com/en-us/data-center/hgx/), [Supermicro HGX B200](https://www.supermicro.com/en/accelerators/nvidia), [ServeTheHome NVSwitch Analysis](https://www.servethehome.com/ingrasys-shows-big-nvidia-nvlink-switch-chips-change-to-the-hgx-b200-b100/)*

HGX B200 is the GPU baseboard for all 8-GPU Blackwell systems. Major architectural changes from HGX H100/H200.

### What's on an HGX B200 Baseboard

| Component | HGX B200 8-GPU |
|-----------|----------------|
| GPUs | 8x B200 SXM6 192GB |
| Total VRAM | 1,536GB (1.5TB) HBM3e |
| Total Memory BW | 64 TB/s aggregate |
| NVSwitch | 2x NVSwitch (4th gen) - reduced from 4 |
| NVLink per GPU | 1,800 GB/s (9 links × 50 GB/s × 2 directions) |
| Total NVLink BW | 14.4 TB/s total |
| GPU-GPU Topology | Full mesh via NVSwitch |
| Fabric Manager | Required |
| Power (GPUs only) | 8,000W |
| Cooling interface | Air or Liquid |

### HGX B200 Architecture Changes

**NVSwitch reduction (4 → 2):**
- Fourth-gen NVSwitch chips are larger (72 ports each)
- Moved to center of baseboard for shorter traces
- Each GPU connects 9 NVLinks to 2 NVSwitch chips
- Same full-mesh topology, fewer chips

**NVLink 5 improvements:**
- 50 GB/s per link (vs 25 GB/s in NVLink 4)
- 18 links per GPU (vs 18 in H100/H200)
- 1.8 TB/s total per GPU (vs 900 GB/s)

### HGX B200 Performance Specifications

| Metric | HGX B200 8-GPU |
|--------|----------------|
| FP4 Tensor | 144 PFLOPS (with sparsity) |
| FP8 Tensor | 72 PFLOPS (with sparsity) |
| FP16 Tensor | 36 PFLOPS (with sparsity) |
| Memory capacity | 1.5TB HBM3e |
| Memory bandwidth | 64 TB/s aggregate |
| NVLink bandwidth | 14.4 TB/s total |
| TDP (GPUs only) | 8,000W |
| System power (typical) | 10-12 kW |

### HGX B200 Pricing (2025)

| Configuration | Baseboard Only | With Typical System | Notes |
|---------------|----------------|---------------------|-------|
| HGX B200 8-GPU (air) | $380-420K | $500-600K | Higher cost, limited cooling |
| HGX B200 8-GPU (liquid) | $400-450K | $550-650K | Better performance |

*Prices volatile due to supply constraints. 12+ month lead times common.*

### Where to Buy HGX B200

**NVIDIA Partners (OEM):**
- [Supermicro](https://www.supermicro.com/en/accelerators/nvidia) - Full production as of Feb 2025
- [Lenovo](https://lenovopress.lenovo.com/lp2226-thinksystem-nvidia-b200-180gb-1000w-gpu) - ThinkSystem SR780a V3, SR680a V3
- [Dell](https://www.dell.com/) - PowerEdge XE series
- [HPE](https://www.hpe.com/) - ProLiant DL380a Gen12

**Resellers:**
- [Broadberry](https://www.broadberry.com/xeon-scalable-processor-gen4-rackmount-servers/nvidia-dgx-b200)
- [Viperatech](https://viperatech.com/)
- [Arc Compute](https://www.arccompute.io/)

**Lead times:** 12+ months as of early 2025. Blackwell allocation fully committed through mid-2026.

### Building Around HGX B200

| Component | Options | Considerations |
|-----------|---------|----------------|
| **CPU host board** | Supermicro, Lenovo, Dell | Must support HGX B200 connector |
| **CPUs** | Intel Xeon 8500 series, AMD EPYC 9004 | Higher power CPUs for balanced system |
| **RAM** | 2TB+ DDR5 ECC | Match CPU memory channels |
| **Storage** | NVMe RAID, 16-32TB | High-speed for checkpoints |
| **Networking** | ConnectX-7/8, InfiniBand NDR | 1-2 NICs per GPU for multi-node |
| **Chassis** | 8U rackmount | Must fit HGX + cooling |
| **Cooling** | Air or Liquid (CDU) | Liquid recommended for sustained loads |
| **Power** | 12-15 kW capacity | 3-phase required |

---

## 🔷 DGX B200 Platform Deep Dive

*Sources: [NVIDIA DGX B200](https://www.nvidia.com/en-us/data-center/dgx-b200/), [DGX B200 Datasheet](https://resources.nvidia.com/en-us-dgx-systems/dgx-b200-datasheet), [RunPod DGX B200 Guide](https://www.runpod.io/articles/guides/nvidia-dgx-b200)*

### DGX B200 Specifications

| Spec | DGX B200 |
|------|----------|
| GPUs | 8x B200 SXM6 192GB |
| GPU Memory | 1,536GB (1.5TB) HBM3e |
| GPU Memory BW | 64 TB/s aggregate |
| Peak FP4 | 144 PFLOPS (with sparsity) |
| Peak FP8 | 72 PFLOPS (with sparsity) |
| NVLink | 1,800 GB/s per GPU, 2x NVSwitch |
| NVLink Aggregate | 14.4 TB/s total |
| CPUs | Dual Intel Xeon Platinum 8570 (56c each, 112c total) |
| System RAM | 2TB DDR5 |
| Storage | 30TB NVMe |
| Networking | 8x ConnectX-7 400Gb |
| Power | 10.2 kW max |
| Cooling | Air or Liquid |
| Dimensions | 8U |
| **List Price** | $500,000 - $550,000 |

### Performance vs Previous DGX

| Metric | DGX H100 | DGX H200 | DGX B200 |
|--------|----------|----------|----------|
| GPU Memory | 640GB | 1,128GB | 1,536GB |
| FP8 Performance | 32 PFLOPS | 32 PFLOPS | 72 PFLOPS |
| Training (relative) | 1x | 1.4x | 3x |
| Inference (relative) | 1x | 1.8x | 15x |
| NVLink BW | 7.2 TB/s | 7.2 TB/s | 14.4 TB/s |

### What DGX B200 Includes

| Component | DGX | HGX Baseboard |
|-----------|-----|---------------|
| GPUs + NVSwitch | ✅ | ✅ |
| CPUs | ✅ Intel Xeon 8570 | ❌ Buy separately |
| System RAM | ✅ 2TB | ❌ Buy separately |
| Storage | ✅ 30TB NVMe | ❌ Buy separately |
| Networking | ✅ 8x ConnectX-7 | ❌ Buy separately |
| Chassis | ✅ 8U integrated | ❌ Buy separately |
| Cooling | ✅ Integrated | ❌ Buy separately |
| Validation | ✅ NVIDIA certified | ❌ OEM dependent |
| Support | ✅ NVIDIA Enterprise | ❌ OEM/self |
| Software | ✅ DGX OS, Base Command | ❌ Install yourself |

### DGX B200 Software Stack

- **DGX OS** - Optimized Ubuntu with Blackwell drivers
- **Base Command Manager** - Cluster management
- **NVIDIA AI Enterprise** - Optimized frameworks
- **NGC Containers** - Pre-built AI containers (Blackwell-optimized)
- **Fabric Manager** - NVSwitch orchestration

---

## 🔶 GB200 Superchip

*Sources: [NVIDIA GB200 NVL72](https://www.nvidia.com/en-us/data-center/gb200-nvl72/), [Supermicro GB200](https://www.supermicro.com/datasheet/datasheet_SuperCluster_GB200_NVL72.pdf), [NexGen Cloud GB200 Guide](https://www.nexgencloud.com/blog/case-studies/nvidia-gb200-user-guide-specs-features-and-use-cases)*

The GB200 Grace Blackwell Superchip combines 2x B200 GPUs with an NVIDIA Grace ARM CPU, connected via NVLink-C2C.

### GB200 Superchip Specifications

| Spec | GB200 Superchip |
|------|-----------------|
| GPUs | 2x B200 (connected at 10 TB/s) |
| GPU Memory | 384GB HBM3e total |
| GPU Memory BW | 16 TB/s aggregate |
| CPU | 1x NVIDIA Grace (72 ARM Neoverse cores) |
| CPU Memory | Up to 480GB LPDDR5X |
| CPU-GPU Connection | 900 GB/s NVLink-C2C |
| TDP | 2,700W |
| Process | TSMC 4NP |

### GB200 Superchip Advantages

- **Unified memory**: Grace CPU shares memory coherence with GPUs
- **Lower latency**: CPU-GPU interconnect at 900 GB/s (vs PCIe 5.0 at 128 GB/s)
- **Power efficiency**: ARM-based Grace uses less power than x86
- **Building block**: GB200 NVL72 uses 36 of these

### GB200 Compute Tray

A compute tray contains 2x GB200 Superchips:
- 4x B200 GPUs
- 2x Grace CPUs
- 768GB GPU memory
- 80 PFLOPS FP8
- "Most powerful compute node ever created"

---

## 🔷 GB200 NVL72 (Rack-Scale)

*Sources: [NVIDIA GB200 NVL72](https://www.nvidia.com/en-us/data-center/gb200-nvl72/), [Hyperstack GB200](https://www.hyperstack.cloud/nvidia-blackwell-gb200), [CoreWeave Blackwell](https://www.coreweave.com/products/nvidia-blackwell)*

The GB200 NVL72 is a rack-scale liquid-cooled system where 72 B200 GPUs act as a single unified accelerator.

### GB200 NVL72 Specifications

| Spec | GB200 NVL72 |
|------|-------------|
| GPUs | 72x B200 (in 36 GB200 Superchips) |
| CPUs | 36x NVIDIA Grace |
| GPU Memory | 13.8TB HBM3e total |
| Total Memory BW | 576 TB/s aggregate |
| NVLink Domain | 72-GPU unified (acts as single GPU) |
| NVLink System BW | 130 TB/s total |
| FP4 Performance | 1.44 exaFLOPS |
| FP8 Performance | 720 PFLOPS |
| Power | ~120 kW |
| Cooling | Liquid only |
| Form Factor | Full rack |
| **Price** | ~$2.5-3.5M |

### GB200 NVL72 vs Multiple DGX B200

| Aspect | 9x DGX B200 (72 GPUs) | GB200 NVL72 |
|--------|----------------------|-------------|
| GPU count | 72 | 72 |
| Memory | 13.8TB | 13.8TB |
| GPU interconnect | InfiniBand between nodes | NVLink (all 72 in one domain) |
| Latency | Higher (network hops) | Lower (direct NVLink) |
| Price | ~$4.5M | ~$3M |
| Form factor | 72U (9 racks) | 1 rack |
| Use case | Flexible multi-node | Unified large models |

### When GB200 NVL72 Makes Sense

- Training models >1T parameters where all GPUs need fast communication
- Inference on massive models that span >8 GPUs
- Workloads that benefit from unified memory addressing
- Organizations that can commit to full rack deployment

---

## 📊 Model-to-Hardware Mapping

*Sources: [Hyperstack VRAM Guide](https://www.hyperstack.cloud/blog/case-study/how-much-vram-do-you-need-for-llms), [Northflank B200 vs H200](https://northflank.com/blog/b200-vs-h200)*

### VRAM Requirements

| Model | FP16 | FP8 | FP4 | Q4 | Notes |
|-------|------|-----|-----|----|----|
| Llama 3.1 8B | 16GB | 8GB | 4GB | 4GB | Single GPU easy |
| Qwen2.5 32B | 64GB | 32GB | 16GB | 18GB | Single B200 |
| Llama 3.1 70B | 140GB | 70GB | 35GB | 35GB | Single B200 |
| Llama 3.1 70B + 128K | 180GB | 90GB | 45GB | 55GB | Single B200 |
| Mixtral 8x22B | 176GB | 88GB | 44GB | 44GB | Single B200 |
| Llama 3.1 405B | 810GB | 405GB | 200GB | 200GB | 4-8x B200 |
| DeepSeek R1 671B | 1.3TB | 670GB | 335GB | 335GB | 8x B200 at FP8 |
| GPT-4 class (1.8T) | 3.6TB | 1.8TB | 900GB | 900GB | GB200 NVL72 |

*Add ~20% overhead for activations and KV cache*

### Minimum B200 Count by Model

| Model | FP16 | FP8 | FP4 | Optimal Config |
|-------|------|-----|-----|----------------|
| 8B | 1 | 1 | 1 | Any |
| 32B | 1 | 1 | 1 | Any |
| 70B | 1 | 1 | 1 | **Single B200!** |
| 70B + long context | 1 | 1 | 1 | Single B200 or GB200 |
| 140B | 2 | 1 | 1 | GB200 Superchip |
| 200B | 2 | 1 | 1 | GB200 Superchip |
| 405B | 8 | 4 | 2 | HGX/DGX B200 |
| 671B | 8 | 4-6 | 3 | DGX B200 |
| 1T+ | 16+ | 8+ | 4+ | GB200 NVL72 |

**Key B200 advantage:** 70B at FP16 with 128K context fits on single GPU!

---

## 🔧 Tier 1: GB200 Superchip (~$150-200K)

### Overview
Entry point to Blackwell. Two B200 GPUs connected at 10 TB/s with a Grace ARM CPU. Runs 200B at FP16, 405B at FP8. Ideal for inference serving and fine-tuning.

### Specifications

| Spec | GB200 Superchip System |
|------|------------------------|
| GPUs | 2x B200 (unified via NVLink-C2C) |
| GPU Memory | 384GB HBM3e |
| GPU-GPU BW | 10 TB/s (on-package) |
| CPU | 1x NVIDIA Grace (72 ARM cores) |
| CPU Memory | 480GB LPDDR5X |
| CPU-GPU BW | 900 GB/s NVLink-C2C |
| TDP | 2,700W |
| Form Factor | 2U-4U compute tray |
| **Est. Price** | $150-200K |

### Model Performance

| Model | Precision | VRAM Used | Tok/s (batch 1) | Tok/s (batch 32) |
|-------|-----------|-----------|-----------------|------------------|
| Llama 3.1 70B | FP16 | 140GB | ~120 | ~1,400 |
| Llama 3.1 70B | FP8 | 70GB | ~160 | ~1,900 |
| Mixtral 8x22B | FP16 | 176GB | ~100 | ~1,200 |
| Llama 3.1 200B* | FP8 | 200GB | ~60 | ~700 |
| Llama 3.1 405B | FP8 | 405GB | Won't fit fully | - |

*Hypothetical model size

**Sweet spot:** 140B at FP16, 200B at FP8
**Maximum:** ~350B at FP8

### Physical Specifications

| Spec | Value |
|------|-------|
| Form factor | 2U-4U compute tray |
| Weight | 80-100 lbs |
| Noise | 65-75 dB |
| Heat output | 9,000-11,000 BTU/hr |
| Cooling | Liquid required |
| **Location** | Server room with liquid cooling |

### Power Requirements

| State | Power Draw | Circuit |
|-------|------------|---------|
| Idle | 600W | 30A/240V |
| Typical | 2,200W | 30A/240V |
| Peak | 2,700-3,000W | 30A/240V |

### Where to Buy

GB200 Superchip is primarily available as part of larger systems:
- [Supermicro](https://www.supermicro.com/en/accelerators/nvidia) - MGX-based systems
- [Dell](https://www.dell.com/) - PowerEdge configurations
- System integrators with NVL72 trays repurposed

---

## 🔧 Tier 2: Quad B200 SXM (~$280-350K)

### Overview
Four B200 GPUs on HGX baseboard. 768GB HBM3e runs 405B at FP16 comfortably. Good balance of capability and infrastructure requirements.

### Bill of Materials

| Component | Recommendation | Price |
|-----------|----------------|-------|
| GPUs | 4x B200 SXM6 (HGX 4-GPU baseboard) | $200-220K |
| CPUs | Dual Intel Xeon 8480+ or AMD EPYC 9654 | $15-20K |
| Motherboard | HGX-compatible | incl. |
| RAM | 1TB DDR5-4800 ECC | $5,000 |
| Storage | 16TB NVMe RAID | $2,500 |
| PSU | 6kW+ | $3,000 |
| Networking | ConnectX-7 200GbE × 2 | $4,000 |
| Cooling | Liquid (CDU required) | $15-25K |
| **Total** | | **$260-300K** |

### Model Performance

| Model | Precision | VRAM Used | Tok/s (batch 1) | Tok/s (batch 64) |
|-------|-----------|-----------|-----------------|------------------|
| Llama 3.1 70B | FP16 | 140GB | ~280 | ~4,000 |
| Llama 3.1 70B | FP8 | 70GB | ~350 | ~5,000 |
| Mixtral 8x22B | FP16 | 176GB | ~220 | ~3,200 |
| Llama 3.1 405B | FP16 | 810GB | Won't fit | - |
| Llama 3.1 405B | FP8 | 405GB | ~80 | ~1,000 |
| DeepSeek R1 671B | FP8 | 670GB | ~40 | ~500 |

**Sweet spot:** 405B at FP8 with full context
**Maximum:** 600B at FP8 (tight)

### Physical Specifications

| Spec | Value |
|------|-------|
| Form factor | 5U rackmount |
| Weight | 180-220 lbs |
| Noise | 70-80 dB |
| Heat output | 20,000-24,000 BTU/hr |
| Cooling | **Liquid required** |
| **Location** | Server room with liquid cooling |

### Power Requirements

| State | Power Draw | Circuit |
|-------|------------|---------|
| Idle | 1,200W | 50A/240V |
| Typical | 5,000W | 50A/240V |
| Peak | 6,000-6,500W | 50A/240V or 3-phase |

---

## 🔧 Tier 3: 8x B200 SXM Air-Cooled (~$500-600K)

### Overview
Full HGX B200 baseboard with air cooling. 1.5TB HBM3e runs 671B at FP16. Lower infrastructure requirements than liquid-cooled but may throttle under sustained loads.

### Bill of Materials

| Component | Recommendation | Price |
|-----------|----------------|-------|
| GPUs | 8x B200 SXM6 (HGX baseboard) | $400-440K |
| CPUs | Dual Intel Xeon 8570 or AMD EPYC 9754 | $25-30K |
| Motherboard | HGX-compatible | incl. |
| RAM | 2TB DDR5 ECC | $10,000 |
| Storage | 30TB NVMe RAID | $5,000 |
| PSU | 12kW redundant | $8,000 |
| Networking | ConnectX-7 400Gb × 8 | $16,000 |
| Cooling | Enhanced air (high-CFM) | $5-8K |
| Chassis | 8U with premium airflow | $3,000 |
| **Total** | | **$475-530K** |

### Model Performance

| Model | Precision | VRAM Used | Tok/s (batch 1) | Tok/s (batch 64) |
|-------|-----------|-----------|-----------------|------------------|
| Llama 3.1 70B | FP16 | 140GB | ~500 | ~7,500 |
| Llama 3.1 70B | FP8 | 70GB | ~600 | ~9,000 |
| Llama 3.1 405B | FP16 | 810GB | ~80 | ~1,000 |
| Llama 3.1 405B | FP8 | 405GB | ~120 | ~1,500 |
| DeepSeek R1 671B | FP16 | 1.3TB | ~50 | ~600 |
| DeepSeek R1 671B | FP8 | 670GB | ~80 | ~1,000 |

**Sweet spot:** 405B at FP16, 671B at FP8
**Maximum:** 671B at FP16 (fits with headroom)
**Limitation:** May throttle under sustained 100% load due to air cooling

### Physical Specifications

| Spec | Value |
|------|-------|
| Form factor | 8U rackmount |
| Weight | 280-320 lbs |
| Noise | 80-90 dB (very loud) |
| Heat output | 35,000-40,000 BTU/hr |
| Cooling | High-CFM air (may throttle) |
| **Location** | Datacenter with excellent airflow |

### Power Requirements

| State | Power Draw | Circuit |
|-------|------------|---------|
| Idle | 2,000W | 3-phase required |
| Typical | 9,000W | 3-phase required |
| Peak | 10,500-11,000W | 3x 30A/240V or 2x 50A/240V |

### Where to Buy

- [Supermicro AS-8125GS-TNHR2](https://www.supermicro.com/en/accelerators/nvidia) (air-cooled)
- [Lenovo ThinkSystem SR680a V3](https://lenovopress.lenovo.com/lp2226-thinksystem-nvidia-b200-180gb-1000w-gpu)

---

## 🔧 Tier 4: DGX B200 (~$550-650K)

### Overview
NVIDIA's turnkey 8x B200 system with liquid cooling. Maximum sustained performance, validated configuration, enterprise support. The reference Blackwell platform.

### DGX B200 Specifications

| Spec | DGX B200 |
|------|----------|
| GPUs | 8x B200 SXM6 192GB |
| GPU Memory | 1,536GB (1.5TB) HBM3e |
| Peak FP4 | 144 PFLOPS |
| Peak FP8 | 72 PFLOPS |
| NVLink | 1,800 GB/s per GPU |
| CPUs | Dual Intel Xeon 8570 (56c each) |
| System RAM | 2TB DDR5 |
| Storage | 30TB NVMe |
| Networking | 8x ConnectX-7 400Gb |
| Power | 10.2 kW max |
| Cooling | Liquid |
| Form Factor | 8U |
| **List Price** | ~$515,000 - $550,000 |

### Model Performance

| Model | Precision | VRAM Used | Tok/s (batch 1) | Tok/s (batch 64) |
|-------|-----------|-----------|-----------------|------------------|
| Llama 3.1 70B | FP16 | 140GB | ~550 | ~8,500 |
| Llama 3.1 70B | FP8 | 70GB | ~650 | ~10,000 |
| Llama 3.1 405B | FP16 | 810GB | ~100 | ~1,300 |
| Llama 3.1 405B | FP8 | 405GB | ~150 | ~1,800 |
| DeepSeek R1 671B | FP16 | 1.3TB | ~65 | ~800 |
| DeepSeek R1 671B | FP8 | 670GB | ~100 | ~1,200 |
| GPT-4 class (1.8T) | FP8 | 1.8TB | Won't fit | - |

**Sweet spot:** 405B at FP16 with maximum throughput
**Maximum:** 671B at FP16, or 1T at FP8 (tight)

### Physical Specifications

| Spec | Value |
|------|-------|
| Form factor | 8U rackmount |
| Dimensions | 19" × 14" × 35.3" D |
| Weight | 300+ lbs |
| Noise | 70-80 dB (liquid-cooled) |
| Heat output | 35,000-40,000 BTU/hr |
| Cooling | **Liquid cooling mandatory** |
| **Location** | Datacenter with liquid cooling infrastructure |

### Power Requirements

| State | Power Draw | Circuit |
|-------|------------|---------|
| Idle | 2,000W | 3-phase required |
| Typical | 8,500W | 3-phase required |
| Peak | 10,200W | 3x 30A/240V or 2x 50A/240V |

### Where to Buy

**Direct:**
- [NVIDIA DGX B200](https://www.nvidia.com/en-us/data-center/dgx-b200/) - Direct from NVIDIA
- [Broadberry DGX B200 Configurator](https://www.broadberry.com/xeon-scalable-processor-gen4-rackmount-servers/nvidia-dgx-b200) - ~$515K

**Cloud (if not buying):**
- [Modal](https://modal.com/) - $6.25/hr per B200 (serverless)
- [Hyperstack](https://www.hyperstack.cloud/nvidia-blackwell-b200) - ~$5.87/hr per B200
- [CoreWeave](https://www.coreweave.com/products/nvidia-blackwell) - ~$8/hr per B200
- [AWS/GCP](https://aws.amazon.com/) - ~$10-18/hr per B200 (8-GPU instances only)

---

## 🔧 Tier 5: GB200 NVL72 (~$2.5-3.5M)

### Overview
Rack-scale liquid-cooled system: 72 B200 GPUs acting as a single unified accelerator. For frontier model training and inference at unprecedented scale.

### GB200 NVL72 Specifications

| Spec | GB200 NVL72 |
|------|-------------|
| GPUs | 72x B200 (36x GB200 Superchips) |
| CPUs | 36x NVIDIA Grace (ARM) |
| GPU Memory | 13.8TB HBM3e total |
| CPU Memory | 17TB+ LPDDR5X |
| Memory BW | 576 TB/s aggregate |
| NVLink Domain | 72-GPU unified |
| NVLink BW | 130 TB/s total |
| FP4 Performance | 1.44 exaFLOPS |
| FP8 Performance | 720 PFLOPS |
| Power | ~120 kW |
| Cooling | Liquid only |
| Form Factor | Full rack (42U) |
| **Price** | ~$2.5-3.5M |

### Model Performance

| Model | Precision | VRAM Used | Tok/s (batch 1) | Tok/s (batch 64) |
|-------|-----------|-----------|-----------------|------------------|
| Llama 3.1 405B | FP16 | 810GB | ~800 | ~12,000 |
| Llama 3.1 405B | FP8 | 405GB | ~1,200 | ~18,000 |
| DeepSeek R1 671B | FP16 | 1.3TB | ~500 | ~7,500 |
| DeepSeek R1 671B | FP8 | 670GB | ~750 | ~11,000 |
| GPT-4 class (1.8T) | FP16 | 3.6TB | ~150 | ~2,000 |
| GPT-4 class (1.8T) | FP8 | 1.8TB | ~300 | ~4,500 |
| Frontier (10T) | FP8 | 10TB | ~50 | ~600 |

**Sweet spot:** Training/inference on 1T+ parameter models
**Maximum:** ~10T parameters at FP8
**Advantage:** All 72 GPUs communicate at NVLink speeds (130 TB/s vs InfiniBand)

### Physical Specifications

| Spec | Value |
|------|-------|
| Form factor | Full 42U rack |
| Footprint | ~30 sq ft with cooling |
| Weight | 3,000+ lbs |
| Noise | 70-80 dB |
| Heat output | ~400,000 BTU/hr |
| Cooling | **Liquid cooling mandatory (custom loop)** |
| **Location** | Datacenter with dedicated liquid cooling |

### Power Requirements

| State | Power Draw | Circuit |
|-------|------------|---------|
| Idle | 30kW | 3-phase industrial |
| Typical | 100kW | 3-phase industrial |
| Peak | 120kW+ | Dedicated transformer |

**Infrastructure requirements:**
- Dedicated 150kW+ power allocation
- Industrial 3-phase power
- Custom liquid cooling loop (120kW heat rejection)
- Reinforced flooring (3,000+ lbs)
- 6+ month lead time

### Where to Buy

- [NVIDIA Direct](https://www.nvidia.com/en-us/data-center/gb200-nvl72/)
- [Supermicro SuperCluster](https://www.supermicro.com/datasheet/datasheet_SuperCluster_GB200_NVL72.pdf)
- [HPE](https://www.hpe.com/) - First shipped Feb 2025
- [CoreWeave](https://www.coreweave.com/products/nvidia-blackwell) - Cloud access

---

## ⚡ Power Infrastructure

*Sources: [DGX B200 Datasheet](https://resources.nvidia.com/en-us-dgx-systems/dgx-b200-datasheet), [Northflank B200 Guide](https://northflank.com/blog/how-much-does-an-nvidia-b200-gpu-cost)*

### Power by Configuration

| Config | GPU Power | System Total | Heat (BTU/hr) | Circuit |
|--------|-----------|--------------|---------------|---------|
| 2x B200 (GB200) | 2,000W | 2,700-3,000W | 9,200-10,200 | 30A/240V |
| 4x B200 | 4,000W | 5,500-6,500W | 18,800-22,200 | 50A/240V |
| 8x B200 | 8,000W | 10,000-11,000W | 34,000-37,500 | 3-phase |
| 72x B200 (NVL72) | 72,000W | 100,000-120,000W | 340,000-410,000 | Industrial |

### UPS Sizing

| System Power | UPS VA Rating | Notes |
|--------------|---------------|-------|
| 3,000W | 4,500 VA | GB200 Superchip |
| 6,500W | 10,000 VA | 4x B200 |
| 11,000W | 16,500 VA | 8x B200, 3-phase |
| 120,000W | Generator backup | NVL72 |

### PDU Requirements

| Config | Connector Type | PDU Rating |
|--------|---------------|------------|
| GB200 Superchip | C19/C20 | 30A single phase |
| 4x B200 | C19/C20 | 50A single phase |
| 8x B200 | C19/C20 | 60A 3-phase |
| NVL72 | Industrial | Custom |

---

## 🌡️ Cooling Requirements

*Sources: [Supermicro Liquid Cooling](https://www.supermicro.com/en/solutions/liquid-cooling), [NVIDIA DGX B200](https://www.nvidia.com/en-us/data-center/dgx-b200/)*

### Cooling Method by Configuration

| Config | Cooling | Heat Load | Notes |
|--------|---------|-----------|-------|
| GB200 Superchip | **Liquid** | 2.7-3kW | Integrated cold plates |
| 4x B200 | **Liquid** | 5.5-6.5kW | CDU required |
| 8x B200 (air) | Air (aggressive) | 10-11kW | May throttle |
| 8x B200 (DGX) | **Liquid** | 10-11kW | Sustained performance |
| NVL72 | **Liquid** | 100-120kW | Custom infrastructure |

### Air Cooling Reality for B200

At 1,000W per GPU, air cooling 8x B200 requires:
- 40,000+ BTU/hr heat rejection
- Hurricane-force airflow (8,000+ CFM)
- Will likely throttle under sustained load
- Liquid strongly recommended for production

### Liquid Cooling Infrastructure

| Component | Purpose | Cost (8-GPU) |
|-----------|---------|--------------|
| CDU (Coolant Distribution Unit) | Pump + heat exchanger | $25-40K |
| Manifolds | Per-rack distribution | $3-6K |
| Cold plates | GPU contact | Included with SXM |
| Facility water loop | Heat rejection | Varies |

**Vendors:**
- [Supermicro Liquid Cooling](https://www.supermicro.com/en/solutions/liquid-cooling) - DLC-2 technology
- [CoolIT Systems](https://www.coolitsystems.com/)
- [Asetek](https://www.asetek.com/)

---

## 🏗️ Rack and Facility Requirements

### Rack Specifications

| Config | Form Factor | Weight | Depth Required |
|--------|-------------|--------|----------------|
| GB200 Superchip | 2U-4U | 80-100 lbs | 30" |
| 4x B200 | 5U | 180-220 lbs | 32" |
| 8x B200 | 8U | 280-320 lbs | 36" |
| NVL72 | 42U rack | 3,000+ lbs | Full rack |

**GPU server racks:** 42-48" deep recommended
**Floor loading:** NVL72 requires reinforced flooring

### Noise Levels

| Config | Noise Level | Comparison |
|--------|-------------|------------|
| GB200 Superchip | 65-75 dB | Vacuum cleaner |
| 8x B200 (air) | 80-90 dB | Leaf blower |
| 8x B200 (liquid) | 70-80 dB | Loud conversation |
| NVL72 | 70-80 dB | Datacenter floor |

---

## 💰 Total Cost of Ownership (3-Year)

### Electricity Costs

*Assuming $0.12/kWh, 24/7 operation*

| Config | Power | Monthly | 3-Year |
|--------|-------|---------|--------|
| GB200 Superchip | 2.5kW avg | $216 | $7,780 |
| 4x B200 | 5.5kW avg | $475 | $17,100 |
| 8x B200 | 9.5kW avg | $821 | $29,560 |
| NVL72 | 100kW avg | $8,640 | $311,000 |

### Total 3-Year TCO

| Config | Hardware | Electricity | Cooling | Support | **Total** |
|--------|----------|-------------|---------|---------|-----------|
| GB200 | $175K | $7.8K | $2.5K | $10K | **$195K** |
| 4x B200 | $315K | $17.1K | $6K | $20K | **$358K** |
| 8x B200 (DGX) | $530K | $29.6K | $9K | $35K | **$604K** |
| NVL72 | $3M | $311K | $100K | $150K | **$3.56M** |

### Cloud Break-Even

| Provider | $/hr/B200 | Break-even vs DGX B200 |
|----------|-----------|------------------------|
| Modal | $6.25 | 11 months |
| Hyperstack | $5.87 | 12 months |
| CoreWeave | $8.00 | 9 months |
| AWS/GCP | $12-18 | 5-7 months |

**Rule of thumb:** Buy if >60% utilization for 10+ months

---

## 🚨 Availability and Lead Times

*Sources: [TechPowerUp](https://www.techpowerup.com/327588/nvidia-blackwell-gpus-are-sold-out-for-12-months-customers-ordering-in-100k-gpu-quantities), [Supermicro IR](https://ir.supermicro.com/news/news-details/2025/Supermicro-Ramps-Full-Production-of-NVIDIA-Blackwell-Rack-Scale-Solutions-with-NVIDIA-HGX-B200/default.aspx)*

### Current Status (January 2025)

| Milestone | Date |
|-----------|------|
| Announcement | GTC March 2024 |
| First customer shipments | Q4 2024 |
| HPE first GB200 NVL72 | February 2025 |
| Supermicro full production | February 2025 |
| Widespread availability | Mid-2026 (estimated) |

### Lead Times

- **New orders:** 12+ months
- **Allocation status:** Sold out through mid-2026
- **Price premiums:** 10-30% above list for faster delivery

### Alternatives While Waiting

- **H200**: Available now, 50-60% of B200 performance
- **H100**: Best availability, good used market
- **Cloud**: Modal, Hyperstack offer B200 access today

---

## 🔗 Related Concepts

**GPUs:**
- [[B200]] - The GPU itself (specs, architecture)
- [[H200]] - Previous generation (141GB HBM3e)
- [[H100]] - Mainstream Hopper
- [[A100]] - Previous generation datacenter GPU
- [[B300]] - Blackwell Ultra (coming late 2025)

**NVIDIA Platforms:**
- [[DGX]] - Turnkey AI supercomputer systems
- [[HGX]] - GPU baseboard for OEM builds
- [[MGX]] - Modular server specification
- [[NVLink]] - GPU interconnect technology (5th gen for B200)
- [[NVSwitch]] - Multi-GPU fabric switch (4th gen for B200)

**System Components:**
- [[Intel Xeon]] - Server CPU option
- [[NVIDIA Grace]] - ARM CPU in GB200
- [[ConnectX-7]] - High-speed networking
- [[InfiniBand]] - Low-latency interconnect
- [[Fabric Manager]] - NVSwitch orchestration software

**Related Guides:**
- [[H200 Builds]] - H200-based system builds
- [[H100 Builds]] - H100-based system builds
- [[A100 Builds]] - A100-based system builds
- [[LLM Inference Hardware]] - GPU comparison overview
- [[LLM Under Your Floorboards]] - Consumer GPU builds
- [[Quantization]] - Reducing VRAM requirements
- [[vLLM]] - Inference engine
- [[TensorRT-LLM]] - NVIDIA's inference engine

---

## 📚 External Resources

### Official Documentation
- [NVIDIA B200 Product Page](https://www.nvidia.com/en-us/data-center/dgx-b200/)
- [NVIDIA DGX B200](https://www.nvidia.com/en-us/data-center/dgx-b200/)
- [DGX B200 Datasheet](https://resources.nvidia.com/en-us-dgx-systems/dgx-b200-datasheet)
- [DGX B200 User Guide](https://docs.nvidia.com/dgx/dgxb200-user-guide/dgxb200-user-guide.pdf)
- [NVIDIA GB200 NVL72](https://www.nvidia.com/en-us/data-center/gb200-nvl72/)

### Platform Documentation
- [NVIDIA HGX Platform](https://www.nvidia.com/en-us/data-center/hgx/)
- [NVIDIA MGX Platform](https://www.nvidia.com/en-us/data-center/products/mgx/)

### System Vendors
- [Supermicro Blackwell Systems](https://www.supermicro.com/en/accelerators/nvidia)
- [Lenovo B200 Product Guide](https://lenovopress.lenovo.com/lp2226-thinksystem-nvidia-b200-180gb-1000w-gpu)
- [Broadberry DGX B200](https://www.broadberry.com/xeon-scalable-processor-gen4-rackmount-servers/nvidia-dgx-b200)
- [Viperatech](https://viperatech.com/)

### Comparisons and Benchmarks
- [B200 vs H200 (Northflank)](https://northflank.com/blog/b200-vs-h200)
- [Multi-GPU Benchmark B200 vs H200 vs H100](https://research.aimultiple.com/multi-gpu/)
- [B100 vs B200 (Northflank)](https://northflank.com/blog/b100-vs-b200)
- [Blackwell vs Hopper (Exxact)](https://www.exxactcorp.com/blog/hpc/comparing-nvidia-tensor-core-gpus)

### Pricing and Market
- [B200 Cost Guide (Northflank)](https://northflank.com/blog/how-much-does-an-nvidia-b200-gpu-cost)
- [B200 Pricing (Modal)](https://modal.com/blog/nvidia-b200-pricing)
- [Hyperstack B200](https://www.hyperstack.cloud/nvidia-blackwell-b200)

### Cloud Providers
- [Modal](https://modal.com/) - Serverless B200
- [CoreWeave Blackwell](https://www.coreweave.com/products/nvidia-blackwell)
- [Hyperstack](https://www.hyperstack.cloud/)

<!--
================================================================================
TEMPLATE NOTES FOR FUTURE GPU BUILD DOCUMENTS (B300, Rubin, etc.)
================================================================================

This document follows the H100/H200/A100 Builds template structure with B200-specific adaptations.

B200-SPECIFIC NOTES:
- No PCIe variant exists - all B200 is SXM6 form factor
- GB200 Superchip is unique to Blackwell (2x B200 + Grace CPU)
- GB200 NVL72 is rack-scale (72 GPUs as unified domain)
- NVLink 5 (1.8 TB/s) and NVSwitch 4th gen
- 1000W TDP requires liquid cooling for sustained performance
- FP4 Tensor Cores are new to Blackwell

TIER STRUCTURE CHANGES:
- No Tier 1 "single PCIe" since B200 is SXM-only
- Tier 1 is GB200 Superchip (2x B200 entry point)
- Tier 5 is GB200 NVL72 rack-scale (unique to Blackwell)

NVLink/NVSwitch GENERATIONS:
| GPU  | NVLink Gen | NVSwitch Gen | Bandwidth | NVSwitch Count |
|------|------------|--------------|-----------|----------------|
| A100 | 3rd        | 2nd          | 600 GB/s  | 6              |
| H100 | 4th        | 3rd          | 900 GB/s  | 4              |
| H200 | 4th        | 3rd          | 900 GB/s  | 4              |
| B200 | 5th        | 4th          | 1800 GB/s | 2              |

AVAILABILITY NOTES:
- Blackwell sold out through mid-2026
- 12+ month lead times as of early 2025
- Cloud access available (Modal, Hyperstack, CoreWeave)

================================================================================
-->
