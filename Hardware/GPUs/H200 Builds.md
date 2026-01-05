# H200 Builds

A comprehensive guide to building systems around NVIDIA's H200 Tensor Core GPU—the memory-enhanced Hopper variant with 141GB HBM3e. Covers variants, specifications, what models fit on different configurations, complete build specs, PCIe lane allocation, cooling, power, physical requirements, and pricing.

For consumer GPU builds see [[LLM Under Your Floorboards]]. For comparisons with other datacenter GPUs see [[LLM Inference Hardware]].

*Pricing last verified: January 2025. GPU prices fluctuate significantly based on supply, demand, and new releases.*

---

## 📍 Where H200 Fits

The H200 is NVIDIA's memory-enhanced Hopper GPU, released in 2024. It uses the same GH100 die as the H100 but with HBM3e memory—76% more VRAM and 43% more bandwidth. Think of it as "H100 Pro" rather than a new architecture.

| Generation | Architecture | Flagship GPU | VRAM | Release | Status |
|------------|--------------|--------------|------|---------|--------|
| Ampere | GA100 | [[A100]] | 40/80GB HBM2e | 2020 | Previous gen, discounted |
| Hopper | GH100 | [[H100]] | 80GB HBM3 | 2022 | Mainstream datacenter |
| **Hopper Refresh** | **GH100** | **[[H200]]** | **141GB HBM3e** | **2024** | **Current premium tier** |
| Blackwell | GB100/GB200 | [[B200]] | 192GB HBM3e | 2024-25 | Next generation |

### When to Choose H200

| Scenario | Best Choice | Why |
|----------|-------------|-----|
| Maximum VRAM per GPU, budget allows | **H200** | 141GB > H100's 80GB, fits larger models |
| Best performance for 70B-200B models | **H200** | Single GPU for 70B FP16, memory-bound gains |
| Cost is primary concern | H100 | 20-30% cheaper, same compute |
| Future-proofing for 1T+ models | B200 | 192GB VRAM, NVLink 5 |
| Training, compute-bound workloads | H100 or B200 | Same compute as H100; B200 is 2x |
| Memory-bound inference at scale | **H200** | 1.4-1.9x faster inference vs H100 |

### H200 vs H100 vs A100 vs B200

| Spec | A100 80GB | H100 80GB | H200 141GB | B200 192GB |
|------|-----------|-----------|------------|------------|
| Architecture | Ampere | Hopper | Hopper | Blackwell |
| VRAM | 80GB HBM2e | 80GB HBM3 | 141GB HBM3e | 192GB HBM3e |
| Memory BW | 2.0 TB/s | 3.35 TB/s | 4.8 TB/s | 8.0 TB/s |
| FP8 Tensor | N/A | 3,958 TFLOPS | 3,958 TFLOPS | 9,000 TFLOPS |
| FP16 Tensor | 312 TFLOPS | 1,979 TFLOPS | 1,979 TFLOPS | 4,500 TFLOPS |
| TDP (SXM) | 400W | 700W | 700W | 1,000W |
| NVLink | 600 GB/s | 900 GB/s | 900 GB/s | 1,800 GB/s |
| PCIe/NVL Price | ~$12-18K | ~$25-35K | ~$30-40K | TBD (~$40-50K) |
| **LLM Inference** | 1x baseline | ~2x A100 | **~2.5x A100** | ~4x A100 |

**Bottom line:**
- **H100 → H200**: Same compute, 76% more VRAM, 43% more bandwidth—1.4-1.9x faster inference
- **H200 vs B200**: B200 has 2x compute, 36% more VRAM—wait for pricing/availability
- **H200 advantage**: Runs 70B FP16 on single GPU (H100 can't), excels at memory-bound inference

---

## 🔧 Software Requirements

*Sources: [NVIDIA CUDA Toolkit](https://developer.nvidia.com/cuda-toolkit), [PyTorch Compatibility](https://pytorch.org/get-started/locally/)*

H200 uses the same Hopper architecture as H100—identical software requirements.

### Minimum Versions

| Component | Minimum | Recommended | Notes |
|-----------|---------|-------------|-------|
| CUDA Toolkit | 11.8 | 12.4+ | FP8 requires CUDA 12+ |
| NVIDIA Driver | 520.x | 550.x+ | Match CUDA version |
| cuDNN | 8.6 | 9.x | For deep learning frameworks |
| TensorRT | 8.5 | 10.x | For optimized inference |
| PyTorch | 2.0 | 2.2+ | Native Hopper support |
| TensorFlow | 2.12 | 2.15+ | XLA compilation support |

### Framework Considerations

| Framework | H200 Support | FP8 Support | Notes |
|-----------|--------------|-------------|-------|
| [[PyTorch]] | Native (2.0+) | Via Transformer Engine | Best flexibility |
| [[TensorFlow]] | Native (2.12+) | Limited | XLA required for best perf |
| [[JAX]] | Native | Via Transformer Engine | Good for research |
| [[vLLM]] | Excellent | Yes | Recommended for inference |
| [[TensorRT-LLM]] | Excellent | Yes | Best raw performance |
| [[llama.cpp]] | Good | No (INT8/INT4 only) | CPU offload capable |

### Container Images

Pre-built containers with H200 optimization (same as H100):
- [NGC PyTorch](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/pytorch) - `nvcr.io/nvidia/pytorch:24.01-py3`
- [NGC TensorRT-LLM](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/tensorrt) - For production inference
- [vLLM Docker](https://docs.vllm.ai/en/latest/serving/deploying_with_docker.html) - Easy inference serving

---

## 📋 Quick Reference

| Tier | GPUs | VRAM | Best Models | Tokens/s (70B) | Budget | Form Factor | Location |
|------|------|------|-------------|----------------|--------|-------------|----------|
| 1 | 1x H200 NVL | 141GB | 70B FP16, 140B FP8 | ~45 tok/s | $40-55K | Tower/4U | Closet/basement |
| 2 | 2x H200 NVL | 282GB | 140B FP16, 200B FP8 | ~85 tok/s | $85-110K | 4U | Dedicated room |
| 3 | 4x H200 NVL | 564GB | 405B FP8, 300B FP16 | ~160 tok/s | $180-220K | 4U-5U | Server room |
| 4 | 4x H200 SXM | 564GB | 405B FP8, 300B FP16 | ~190 tok/s | $250-320K | 5U | Server room |
| 5 | 8x H200 SXM | 1.1TB | 671B FP8, 405B FP16 | ~320 tok/s | $420-520K | 8U | Datacenter |

---

## 🎯 H200 Variants

*Sources: [NVIDIA H200 Page](https://www.nvidia.com/en-us/data-center/h200/), [PNY H200 NVL Datasheet](https://www.pny.com/file%20library/company/support/linecards/data-center-gpus/h200-nvl-datasheet.pdf), [Lenovo H200 Product Guide](https://lenovopress.lenovo.com/lp1944-nvidia-h200-141gb-gpu)*

| Variant | VRAM | Memory BW | TDP | NVLink BW | Interface | Cooling |
|---------|------|-----------|-----|-----------|-----------|---------|
| **H200 NVL PCIe** | 141GB HBM3e | 4,800 GB/s | 600W | 900 GB/s (2-4 way bridge) | PCIe 5.0 x16 | Air |
| **H200 SXM5** | 141GB HBM3e | 4,800 GB/s | 700W | 900 GB/s (mesh) | SXM socket | Liquid |

**Note:** Unlike H100, there is no standalone "H200 PCIe 80GB" variant. All H200s have 141GB HBM3e. The only variants are:
- **H200 NVL**: Air-cooled PCIe form factor with NVLink bridge capability (2 or 4 GPUs)
- **H200 SXM5**: Liquid-cooled socket mount for HGX baseboards

### Which Variant to Choose

| Use Case | Recommended Variant | Why |
|----------|---------------------|-----|
| Single GPU inference (70B+) | H200 NVL | 141GB fits 70B FP16, air-cooled |
| Paired inference (140B) | H200 NVL × 2 | 282GB combined, NVLink bridge |
| Quad-GPU production | H200 NVL × 4 | 564GB, air-cooled, NVLink bridge |
| Maximum performance | H200 SXM5 | Full NVLink mesh, NVSwitch support |
| Multi-node training | H200 SXM5 | NVLink-Network for 256+ GPU clusters |

---

## 🖥️ NVIDIA Platform Overview

*Sources: [ServeTheHome DGX vs HGX](https://www.servethehome.com/nvidia-dgx-versus-nvidia-hgx-what-is-the-difference/), [NVIDIA HGX Platform](https://www.nvidia.com/en-us/data-center/hgx/), [NVIDIA MGX](https://www.nvidia.com/en-us/data-center/products/mgx/)*

NVIDIA offers multiple platform tiers for datacenter AI. Understanding these is critical for H200 builds since SXM GPUs require specific baseboards.

| Platform | What It Is | Customization | Target Buyer | H200 Relevance |
|----------|------------|---------------|--------------|----------------|
| **[[DGX]]** | Complete turnkey system | Fixed | Enterprise (plug-and-play) | DGX H200 is the reference 8-GPU system |
| **[[HGX]]** | GPU baseboard + NVSwitch | High (OEM builds around it) | OEMs, cloud providers | Required for SXM builds |
| **[[MGX]]** | Modular server spec | Very high (multi-gen) | System builders | Good for H200→B200 upgrade path |
| **EGX** | Edge AI platform | Moderate | Edge deployments | Not applicable (uses L40/A100) |
| **AGX** | Embedded autonomous | High | Robotics, automotive | Not applicable (Jetson/Orin) |

### When to Choose Each

| Scenario | Best Platform | Why |
|----------|---------------|-----|
| "I want it working next month" | DGX | Pre-configured, NVIDIA support, validated |
| "I need AMD EPYC CPUs" | HGX + OEM | DGX uses Intel; HGX lets you choose |
| "I need specific storage/networking" | HGX + OEM | Customize around the baseboard |
| "Building for cloud customers" | HGX | Cloud providers all use HGX |
| "Future B200 upgrade path" | MGX | Designed for multi-generational reuse |
| "Budget is critical" | HGX + self-build | Skip NVIDIA's DGX premium |

---

## 🔲 HGX Platform Deep Dive

*Sources: [NVIDIA HGX Page](https://www.nvidia.com/en-us/data-center/hgx/), [Exxact HGX H200](https://www.exxactcorp.com/NVIDIA-935-24287-0040-000-E184652691), [SabrePC HGX H200](https://www.sabrepc.com/935-24287-0040-000-NVIDIA-S184652691)*

HGX (Hyperscale Graphics eXtension) is the GPU baseboard that sits at the heart of all SXM-based systems. When you buy a DGX, you're buying an HGX baseboard plus everything else. When you buy HGX directly, you build the rest yourself or through an OEM.

### What's on an HGX Baseboard

| Component | HGX H200 4-GPU | HGX H200 8-GPU |
|-----------|----------------|----------------|
| GPUs | 4x H200 SXM5 141GB | 8x H200 SXM5 141GB |
| Total VRAM | 564GB HBM3e | 1,128GB (1.1TB) HBM3e |
| NVSwitch | None (direct NVLink) | 4x NVSwitch (3rd gen) |
| NVLink per GPU | 900 GB/s | 900 GB/s |
| GPU-GPU Topology | Ring (peer-to-peer) | Full mesh (any-to-any) |
| Fabric Manager | Not required | Required |
| Power connectors | High-current board power | High-current board power |
| Cooling interface | Liquid cooling manifold | Liquid cooling manifold |

### 4-GPU vs 8-GPU: Key Differences

**4-GPU Configuration:**
- GPUs connect directly via NVLink in a ring topology
- Simpler: no NVSwitch, no Fabric Manager software
- Lower cost (~$170-185K for baseboard)
- Good for: inference, smaller training jobs, 200B-400B models

**8-GPU Configuration:**
- GPUs connect through 4x NVSwitch chips
- Full mesh: any GPU can talk to any other at 900 GB/s simultaneously
- Required for: 405B+ FP16 models, large-scale training, collective operations
- Higher cost (~$280-320K for baseboard)
- Supports NVLink-Network for multi-node scaling (up to 256 GPUs)

### HGX H200 Performance Specifications

| Metric | HGX H200 4-GPU | HGX H200 8-GPU |
|--------|----------------|----------------|
| FP8 Tensor | 16 PFLOPS | 32 PFLOPS |
| FP16 Tensor | 8 PFLOPS | 16 PFLOPS |
| Memory bandwidth | 19.2 TB/s aggregate | 38.4 TB/s aggregate |
| NVLink bandwidth | 3.6 TB/s total | 7.2 TB/s total |
| TDP (GPUs only) | 2,800W | 5,600W |
| System power (typical) | 4-5 kW | 8-10 kW |

### HGX Pricing (2024-2025)

| Configuration | Baseboard Only | With Typical System | Notes |
|---------------|----------------|---------------------|-------|
| HGX H200 4-GPU | $170-185K | $250-320K | Includes CPUs, RAM, chassis |
| HGX H200 8-GPU | $280-320K | $420-520K | Requires liquid cooling |

*Prices vary significantly based on market conditions and quantity*

### Where to Buy HGX H200 Baseboards

**Direct from NVIDIA partners:**
- [Supermicro](https://www.supermicro.com/en/accelerators/nvidia/hopper-ada-lovelace) - SYS-821GE-TNHR series
- [Gigabyte](https://www.gigabyte.com/Enterprise/GPU-Server) - G593-SD2-H200
- [Lenovo](https://lenovopress.lenovo.com/lp1944-nvidia-h200-141gb-gpu) - ThinkSystem SR680a V3

**Resellers:**
- [Exxact](https://www.exxactcorp.com/) - HGX H200 baseboards and systems
- [SabrePC](https://www.sabrepc.com/) - Enterprise configurations
- [Viperatech](https://viperatech.com/) - New with warranty (~$285K+ for 8-GPU systems)
- [Arc Compute](https://www.arccompute.io/) - HGX H200 servers

**Note:** Lead times are currently 12-26 weeks depending on configuration due to high demand.

### Building Around HGX H200

When you buy an HGX baseboard, you still need:

| Component | Options | Considerations |
|-----------|---------|----------------|
| **CPU host board** | Supermicro H13, Gigabyte, Lenovo | Must support HGX connector |
| **CPUs** | AMD EPYC 9004, Intel Xeon 4th/5th gen | DGX uses Intel; HGX gives you choice |
| **RAM** | 1-2TB DDR5 ECC | Match CPU memory channels |
| **Storage** | NVMe RAID, often 8-16TB | High-speed for checkpoints |
| **Networking** | ConnectX-7, InfiniBand | 1-2 NICs per GPU for multi-node |
| **Chassis** | 5U-8U rackmount | Must fit HGX + cooling |
| **Liquid cooling** | CDU + manifolds | Mandatory for SXM |
| **Power** | 6-12 kW capacity | 3-phase recommended for 8-GPU |

### HGX vs Building with NVL PCIe Cards

| Factor | HGX (SXM) | NVL PCIe Cards |
|--------|-----------|----------------|
| GPU-GPU bandwidth | 900 GB/s (NVSwitch) | 900 GB/s (NVLink bridge, 4-way max) |
| Cooling | Liquid required | Air possible |
| Flexibility | Locked to HGX ecosystem | Any server chassis |
| Multi-node scaling | NVLink-Network ready | InfiniBand only |
| Cost per GPU | Higher | Lower |
| Training efficiency | Excellent | Good (up to 4 GPUs) |
| Inference efficiency | Excellent | Excellent |

**Rule of thumb:**
- Training large models (200B+): HGX is worth the premium
- Inference or up to 4-GPU training: NVL PCIe often sufficient

---

## 🔷 DGX Platform Deep Dive

*Sources: [NVIDIA DGX H200](https://www.nvidia.com/en-us/data-center/dgx-h200/), [DGX H200 Datasheet](https://resources.nvidia.com/en-us-dgx-systems/dgx-h200-datasheet)*

DGX is NVIDIA's turnkey solution—an HGX baseboard with everything else pre-integrated and validated.

### DGX H200 Specifications

| Spec | DGX H200 |
|------|----------|
| GPUs | 8x H200 SXM5 141GB |
| GPU Memory | 1,128GB (1.1TB) HBM3e |
| GPU Memory BW | 38.4 TB/s aggregate |
| NVLink | 900 GB/s per GPU, 4x NVSwitch |
| NVLink Aggregate | 7.2 TB/s bidirectional |
| CPUs | Dual Intel Xeon 8480C (56c each, 112c total) |
| System RAM | 2TB DDR5 |
| Storage | 30TB NVMe (8x 3.84TB) |
| Networking | 8x ConnectX-7 400Gb (3.2 Tb/s aggregate) |
| Interconnect | 4x OSFP ports for InfiniBand/Ethernet |
| Power | 10.2 kW max |
| Cooling | Liquid (rear-door or direct) |
| Dimensions | 19" × 14" × 35.3" (8U) |
| Weight | 275 lbs (125 kg) |
| **List Price** | $400,000 - $500,000 |

### What DGX Includes That HGX Doesn't

| Component | DGX | HGX Baseboard |
|-----------|-----|---------------|
| GPUs + NVSwitch | ✅ | ✅ |
| CPUs | ✅ Intel Xeon | ❌ Buy separately |
| System RAM | ✅ 2TB | ❌ Buy separately |
| Storage | ✅ 30TB NVMe | ❌ Buy separately |
| Networking | ✅ 8x ConnectX-7 | ❌ Buy separately |
| Chassis | ✅ 8U integrated | ❌ Buy separately |
| Liquid cooling | ✅ Integrated | ❌ Buy separately |
| Cable management | ✅ Done | ❌ DIY |
| Validation | ✅ NVIDIA certified | ❌ OEM dependent |
| Support | ✅ NVIDIA Enterprise | ❌ OEM/self |
| Software | ✅ DGX OS, Base Command | ❌ Install yourself |

### DGX H200 Pricing Reality

| What You Pay For | Value |
|------------------|-------|
| Hardware (HGX equivalent) | ~$320K |
| Intel CPUs + RAM + storage | ~$35K |
| Networking (8x CX-7) | ~$12K |
| Chassis + cooling | ~$25K |
| Integration + validation | ~$30K |
| NVIDIA support (3 yr) | ~$50K |
| **Total DGX cost** | **~$470K** |

**The DGX premium:** ~$50-100K over building equivalent HGX system yourself. Worth it for:
- Guaranteed compatibility
- Single throat to choke (NVIDIA support)
- Faster deployment (weeks vs months)
- Validated performance benchmarks

### DGX Software Stack

DGX includes pre-installed:
- **DGX OS** - Optimized Ubuntu with drivers
- **Base Command Manager** - Cluster management
- **NVIDIA AI Enterprise** - Optimized frameworks
- **NGC Containers** - Pre-built AI containers
- **Fabric Manager** - NVSwitch orchestration

---

## 🔶 MGX Platform Overview

*Sources: [NVIDIA MGX](https://www.nvidia.com/en-us/data-center/products/mgx/), [MGX Technical Blog](https://developer.nvidia.com/blog/building-the-modular-foundation-for-ai-factories-with-nvidia-mgx/)*

MGX (Modular GPU eXtension) is NVIDIA's newest platform specification, designed for flexibility and multi-generational compatibility.

### MGX Key Features

| Feature | Benefit |
|---------|---------|
| Modular architecture | Swap GPU/CPU/networking without full redesign |
| Multi-generational | Same chassis works with H100 → H200 → B200 |
| 1U-6U form factors | Right-size for workload |
| Open standards | PCIe, OCP, EIA rack specs |
| Reduced R&D | $2-4M savings per platform design |

### MGX vs HGX

| Aspect | HGX | MGX |
|--------|-----|-----|
| GPU count | 4 or 8 SXM | Up to 8 (various types) |
| Form factor | 5U-8U | 1U-6U |
| GPU types | SXM only | SXM, PCIe, or both |
| Upgrade path | New baseboard per generation | Modular swap |
| Cooling | Liquid only | Air or liquid |
| Primary use | Maximum GPU density | Flexible configurations |

### When MGX Makes Sense

- Planning multi-year infrastructure with GPU upgrades (H200 → B200)
- Need smaller form factors (1U-2U)
- Want mix of GPU types (H200 NVL + L40S in same chassis)
- Building standardized fleet across different workloads

**For H200 SXM builds specifically:** HGX remains the standard. MGX is more relevant for mixed deployments or when planning B200 upgrade paths.

---

## 📍 EGX & AGX (Edge Platforms)

*For completeness—not directly relevant to H200 datacenter builds*

### EGX (Edge GPU eXtension)

Edge AI platform for deployments outside datacenters:
- Uses A100 PCIe or L40 (not H200)
- Designed for: manufacturing, retail, healthcare, telco
- Smaller form factor, lower power
- Remote fleet management

### AGX (Autonomous GPU eXtension)

Embedded platforms for autonomous systems:
- **Jetson AGX Orin**: 275 TOPS, 60W, for robots/drones
- **Jetson AGX Thor**: 2000+ TOPS, 130W, for humanoids/vehicles
- **IGX Orin/Thor**: Industrial-grade variants

These use different GPU architectures and aren't compatible with H200 datacenter deployments.

---

## 📊 Model-to-Hardware Mapping

*Sources: [Hyperstack VRAM Guide](https://www.hyperstack.cloud/blog/case-study/how-much-vram-do-you-need-for-llms), [NVIDIA TensorRT-LLM Blog](https://developer.nvidia.com/blog/achieving-top-inference-performance-with-the-nvidia-h100-tensor-core-gpu-and-nvidia-tensorrt-llm/)*

### VRAM Requirements

| Model | FP16 | FP8 | Q4 | Notes |
|-------|------|-----|----|----|
| Llama 3.1 8B | 16GB | 8GB | 4GB | Single GPU easy |
| Qwen2.5 32B | 64GB | 32GB | 18GB | Single H200 with headroom |
| Llama 3.1 70B | 140GB | 70GB | 35GB | **Single H200 at FP16!** |
| Llama 3.1 70B + 128K ctx | 180GB | 90GB | 55GB | Single H200 (tight) |
| Mixtral 8x22B | 176GB | 88GB | 44GB | 2x H200 |
| Llama 3.1 405B | 810GB | 405GB | 200GB | 8x H200 at FP16, 4x at Q4 |
| DeepSeek R1 671B | 1.3TB | 670GB | 335GB | 8x H200 at FP8 |

*Add ~20% overhead for activations and KV cache*

### Minimum H200 Count by Model

| Model | FP16 | FP8 | Q4 | Optimal Config |
|-------|------|-----|-----|----------------|
| 8B | 1 | 1 | 1 | 1x NVL |
| 32B | 1 | 1 | 1 | 1x NVL |
| 70B | **1** | 1 | 1 | **1x NVL (H200 advantage!)** |
| 70B + long context | 2 | 1 | 1 | 2x NVL |
| 140B | 2 | 1 | 1 | 2x NVL |
| 200B | 2 | 2 | 1 | 2x NVL or 4x NVL |
| 405B | 8 | 4 | 2 | 4x SXM or 8x SXM |
| 671B | 12+ | 8 | 4 | 8x SXM |

**Key H200 advantage:** 70B at FP16 fits on a single H200 (141GB), while H100 (80GB) requires 2 GPUs.

---

## 🔧 Tier 1: Single H200 NVL (~$40-55K)

### Overview
The sweet spot for 70B models at full precision on a single GPU. H200's 141GB enables what required 2x H100s before. Excellent for development, API serving, and production inference.

### Bill of Materials

| Component | Recommendation | Price | Link |
|-----------|----------------|-------|------|
| GPU | 1x H200 NVL PCIe 141GB | $32-40K | [NVIDIA H200](https://www.nvidia.com/en-us/data-center/h200/) |
| CPU | AMD EPYC 9124 (16c/32t) | $1,100 | [[AMD EPYC]] / [AMD Store](https://www.amd.com/en/products/processors/server/epyc/4th-generation-9004-and-8004-series.html) |
| Motherboard | Supermicro H13SSL-N | $800 | [[Supermicro]] / [Supermicro Store](https://store.supermicro.com/) |
| RAM | 256GB DDR5-4800 ECC (8x32GB) | $1,200 | |
| Storage | 2TB NVMe Gen4 | $200 | |
| PSU | 1600W 80+ Titanium | $400 | |
| Case | 4U rackmount or tower | $300-500 | |
| Cooling | High-airflow fans | $200 | 600W GPU TDP |
| **Total** | | **$36-44K** | |

### PCIe Lane Allocation

**CPU:** AMD EPYC 9124 provides **128 PCIe 5.0 lanes**

| Component | Lanes | Bandwidth | Remaining |
|-----------|-------|-----------|-----------|
| H200 GPU | x16 | 128 GB/s | 112 |
| Boot NVMe | x4 | 8 GB/s | 108 |
| Data NVMe | x4 | 8 GB/s | 104 |
| 25GbE NIC | x8 | 32 GB/s | 96 |
| **Available** | | | **96 lanes** |

Plenty of room for additional storage, 100GbE upgrade, or second GPU later.

### Model Performance

| Model | Precision | VRAM Used | Tok/s (batch 1) | Tok/s (batch 16) |
|-------|-----------|-----------|-----------------|------------------|
| Llama 3.1 8B | FP16 | 16GB | ~160 | ~2,000 |
| Qwen2.5 32B | FP16 | 64GB | ~55 | ~550 |
| Llama 3.1 70B | FP16 | 140GB | ~45 | ~450 |
| Llama 3.1 70B | FP8 | 70GB | ~55 | ~600 |
| Llama 3.1 70B + 128K | FP8 | 90GB | ~40 | ~400 |

**Sweet spot:** 70B at FP16—this is the killer feature of H200 over H100
**Maximum:** 70B at FP16 with moderate context, or 140B at FP8

### Physical Specifications

| Spec | Value |
|------|-------|
| Form factor | Tower workstation or 4U rackmount |
| Dimensions | 17" W × 7" H × 26" D (tower) |
| Weight | 50-65 lbs (23-30 kg) |
| Noise | 50-60 dB (noticeable but tolerable) |
| Heat output | 2,500-3,200 BTU/hr |
| **Location** | Large closet, basement, home office with door |

### Power Requirements

| State | Power Draw | Circuit |
|-------|------------|---------|
| Idle | 250W | Standard 15A/120V OK |
| Typical | 650W | Standard 15A/120V OK |
| Peak | 900-1,000W | 15A/120V OK (with headroom) |

Recommended: Dedicated 15A circuit, 1200VA UPS

### Where to Buy

**New:**
- [Supermicro GPU Workstations](https://www.supermicro.com/en/products/gpu)
- [Exxact Workstations](https://www.exxactcorp.com/gpu-workstations)
- [Thinkmate GPU Workstations](https://www.thinkmate.com/systems/workstations/gpx)

**Used:**
- eBay (search "H200 NVL")
- Limited used market as of early 2025

---

## 🔧 Tier 2: Dual H200 NVL (~$85-110K)

### Overview
Two H200 NVLs connected via NVLink bridge provide 282GB combined VRAM and 900 GB/s GPU-to-GPU bandwidth. Runs 140B models at FP16, 200B at FP8. The workhorse for production inference.

### Bill of Materials

| Component | Recommendation | Price | Link |
|-----------|----------------|-------|------|
| GPUs | 2x H200 NVL PCIe 141GB + NVLink bridge | $70-85K | [NVIDIA H200](https://www.nvidia.com/en-us/data-center/h200/) |
| CPU | AMD EPYC 9354 (32c/64t) | $2,500 | [AMD EPYC 9004](https://www.amd.com/en/products/processors/server/epyc/4th-generation-9004-and-8004-series.html) |
| Motherboard | Supermicro H13DSG-OM (dual socket capable) | $1,500 | [Supermicro H13](https://www.supermicro.com/en/products/motherboards?pro=H13) |
| RAM | 512GB DDR5-4800 ECC | $2,400 | |
| Storage | 4TB NVMe Gen4 RAID | $400 | |
| PSU | 2400W redundant | $1,000 | |
| Case | 4U rackmount | $400 | |
| Cooling | High-airflow fans | $300 | |
| **Total** | | **$78-93K** | |

### PCIe Lane Allocation

**CPU:** AMD EPYC 9354 provides **128 PCIe 5.0 lanes**

| Component | Lanes | Bandwidth | Remaining |
|-----------|-------|-----------|-----------|
| H200 GPU #1 | x16 | 128 GB/s | 112 |
| H200 GPU #2 | x16 | 128 GB/s | 96 |
| NVMe RAID (2 drives) | x8 | 16 GB/s | 88 |
| 100GbE NIC | x16 | 64 GB/s | 72 |
| **Available** | | | **72 lanes** |

Room for: Additional NVMe, InfiniBand HCA, or 2 more GPUs

### Model Performance

| Model | Precision | VRAM Used | Tok/s (batch 1) | Tok/s (batch 32) |
|-------|-----------|-----------|-----------------|------------------|
| Llama 3.1 70B | FP16 | 140GB | ~85 | ~1,000 |
| Llama 3.1 70B | FP8 | 70GB | ~100 | ~1,200 |
| Llama 3.1 70B + 128K | FP16 | 180GB | ~70 | ~800 |
| Mixtral 8x22B | FP16 | 176GB | ~60 | ~700 |
| Llama 3.1 200B* | FP8 | 200GB | ~35 | ~350 |

*Hypothetical model size

**Sweet spot:** 70B at FP16 with 128K context, or 140B at FP8
**Maximum:** 200B at FP8

### Physical Specifications

| Spec | Value |
|------|-------|
| Form factor | 4U rackmount |
| Dimensions | 17.2" W × 7" H × 28" D |
| Weight | 75-90 lbs (34-41 kg) |
| Noise | 60-70 dB (loud, needs isolation) |
| Heat output | 5,000-6,500 BTU/hr |
| **Location** | Dedicated room, basement server area |

### Power Requirements

| State | Power Draw | Circuit |
|-------|------------|---------|
| Idle | 500W | 20A/120V |
| Typical | 1,400W | 20A/120V |
| Peak | 1,800-2,000W | 20A/120V or 15A/240V |

Recommended: Dedicated 20A/120V or 15A/240V circuit, 2500VA UPS

### Where to Buy

**New:**
- [Supermicro AS-4125GS-TNRT](https://store.supermicro.com/) (~$95-115K configured)
- [Thinkmate GPX Servers](https://www.thinkmate.com/systems/servers/gpx)
- [Exxact 4U GPU Server](https://www.exxactcorp.com/gpu-servers)

---

## 🔧 Tier 3: Quad H200 NVL (~$180-220K)

### Overview
Four H200 NVL cards connected via 4-way NVLink bridge. 564GB combined VRAM runs 405B at FP8 comfortably. Air-cooled alternative to HGX for inference-focused deployments.

### Bill of Materials

| Component | Recommendation | Price | Link |
|-----------|----------------|-------|------|
| GPUs | 4x H200 NVL PCIe 141GB + NVLink bridge | $140-165K | [NVIDIA H200](https://www.nvidia.com/en-us/data-center/h200/) |
| CPUs | Dual AMD EPYC 9454 (48c each) | $8,000 | [AMD EPYC 9004](https://www.amd.com/en/products/processors/server/epyc/4th-generation-9004-and-8004-series.html) |
| Motherboard | Supermicro H13DSG-OM | $1,500 | [Supermicro Store](https://store.supermicro.com/) |
| RAM | 1TB DDR5-4800 ECC | $4,800 | |
| Storage | 8TB NVMe RAID | $1,000 | |
| PSU | 3600W redundant | $1,500 | |
| Networking | Mellanox ConnectX-7 100GbE | $1,500 | [[ConnectX-7]] |
| Case | 4U-5U rackmount | $500 | |
| Cooling | High-airflow / hybrid | $700 | |
| **Total** | | **$160-185K** | |

### PCIe Lane Allocation

**CPUs:** Dual AMD EPYC 9454 = **128 usable PCIe 5.0 lanes**
*(Note: 64 lanes per CPU, but 64 go to Infinity Fabric for inter-CPU link)*

| Component | Lanes | Bandwidth | Remaining |
|-----------|-------|-----------|-----------|
| H200 GPU #1 | x16 | 128 GB/s | 112 |
| H200 GPU #2 | x16 | 128 GB/s | 96 |
| H200 GPU #3 | x16 | 128 GB/s | 80 |
| H200 GPU #4 | x16 | 128 GB/s | 64 |
| NVMe RAID (4 drives) | x16 | 32 GB/s | 48 |
| 100GbE NIC | x16 | 64 GB/s | 32 |
| **Available** | | | **32 lanes** |

Room for: InfiniBand HCA for multi-node, additional 100GbE

### Model Performance

| Model | Precision | VRAM Used | Tok/s (batch 1) | Tok/s (batch 64) |
|-------|-----------|-----------|-----------------|------------------|
| Llama 3.1 70B | FP16 | 140GB | ~160 | ~2,000 |
| Llama 3.1 70B | FP8 | 70GB | ~180 | ~2,400 |
| Mixtral 8x22B | FP16 | 176GB | ~120 | ~1,500 |
| Llama 3.1 405B | FP8 | 405GB | ~30 | ~350 |
| Llama 3.1 405B | Q4 | 200GB | ~40 | ~450 |
| DeepSeek R1 671B | Q4 | 335GB | ~15 | ~150 |

**Sweet spot:** 405B at FP8 with good context
**Maximum:** 671B at Q4 (limited context)
**Advantage over Tier 4:** Air-cooled, simpler infrastructure

### Physical Specifications

| Spec | Value |
|------|-------|
| Form factor | 4U-5U rackmount |
| Dimensions | 17.2" W × 7-8.75" H × 30" D |
| Weight | 100-120 lbs (45-54 kg) |
| Noise | 70-80 dB (very loud) |
| Heat output | 10,000-12,500 BTU/hr |
| Rack depth | Requires 42"+ deep rack |
| **Location** | Dedicated server room |

### Power Requirements

| State | Power Draw | Circuit |
|-------|------------|---------|
| Idle | 800W | 30A/240V |
| Typical | 2,600W | 30A/240V |
| Peak | 3,200-3,600W | 30A/240V |

Recommended: 30A/240V circuit, 4000VA UPS, PDU with C19 outlets

### Where to Buy

**New:**
- [Supermicro AS-4125GS-TNRT](https://www.supermicro.com/en/products/system/gpu/4u/as-4125gs-tnrt) (~$190-230K)
- [Thinkmate GPX TS4](https://www.thinkmate.com/systems/servers/gpx)
- [Exxact 4U GPU Server](https://www.exxactcorp.com/gpu-servers)

---

## 🔧 Tier 4: Quad H200 SXM (~$250-320K)

### Overview
Four SXM GPUs on HGX baseboard with full NVLink mesh. Higher bandwidth than NVL configuration, better for training and maximum inference throughput. Requires liquid cooling.

### Bill of Materials

| Component | Recommendation | Price | Link |
|-----------|----------------|-------|------|
| GPUs | 4x H200 SXM5 (HGX baseboard) | $170-185K | [NVIDIA HGX](https://www.nvidia.com/en-us/data-center/hgx/) |
| CPUs | Dual AMD EPYC 9454 or Intel Xeon 8480+ | $10-15K | [AMD EPYC](https://www.amd.com/en/products/processors/server/epyc/4th-generation-9004-and-8004-series.html) / [Intel Xeon](https://www.intel.com/content/www/us/en/products/details/processors/xeon/scalable.html) |
| Motherboard | HGX-compatible (included in chassis) | incl. | |
| RAM | 1TB DDR5-4800 ECC | $4,800 | |
| Storage | 8TB NVMe RAID | $1,000 | |
| PSU | 5000W+ | $2,500 | |
| Networking | ConnectX-7 200GbE or HDR InfiniBand | $3,000 | |
| Cooling | **Liquid cooling (CDU required)** | $10-18K | |
| **Total** | | **$220-250K** | |

### PCIe Lane Allocation

**HGX Configuration:** GPUs connect via NVSwitch, not PCIe. CPU lanes for peripherals only.

| Component | Lanes | Bandwidth | Remaining |
|-----------|-------|-----------|-----------|
| NVMe RAID | x16 | 32 GB/s | 112 |
| 200GbE NIC #1 | x16 | 64 GB/s | 96 |
| 200GbE NIC #2 | x16 | 64 GB/s | 80 |
| InfiniBand HCA | x16 | 64 GB/s | 64 |
| **Available** | | | **64 lanes** |

GPUs use NVLink (900 GB/s per GPU), not PCIe lanes.

### Model Performance

| Model | Precision | VRAM Used | Tok/s (batch 1) | Tok/s (batch 64) |
|-------|-----------|-----------|-----------------|------------------|
| Llama 3.1 70B | FP16 | 140GB | ~190 | ~2,800 |
| Llama 3.1 70B | FP8 | 70GB | ~220 | ~3,200 |
| Mixtral 8x22B | FP16 | 176GB | ~150 | ~2,100 |
| Llama 3.1 405B | FP8 | 405GB | ~40 | ~450 |
| Llama 3.1 405B | Q4 | 200GB | ~50 | ~550 |

**Sweet spot:** 405B at FP8 with excellent throughput
**Maximum:** 405B at FP8 with 64K+ context

### Physical Specifications

| Spec | Value |
|------|-------|
| Form factor | 5U rackmount |
| Dimensions | 17.2" W × 8.75" H × 32" D |
| Weight | 140-170 lbs (64-77 kg) |
| Noise | 70-80 dB (requires isolation) |
| Heat output | 17,000-20,500 BTU/hr |
| Rack depth | Requires 42"+ deep rack |
| Cooling | **Liquid cooling infrastructure required** |
| **Location** | Server room with liquid cooling loop |

### Power Requirements

| State | Power Draw | Circuit |
|-------|------------|---------|
| Idle | 1,000W | 40A/240V |
| Typical | 4,200W | 40A/240V |
| Peak | 5,500-6,000W | 40A/240V |

Recommended: 40A/240V circuit, industrial UPS, CDU for cooling

### Where to Buy

**Pre-built (recommended for SXM):**
- [Supermicro SYS-521GU-TNXR](https://www.supermicro.com/en/products/system/gpu/5u/sys-521gu-tnxr) (~$270-320K)
- [Dell PowerEdge XE8640](https://www.dell.com/en-us/shop/servers-storage-and-networking/poweredge-xe8640-rack-server/spd/poweredge-xe8640/pe_xe8640_16902_vi_vp)

---

## 🔧 Tier 5: 8x H200 SXM / DGX H200 (~$420-520K)

### Overview
Full HGX baseboard with 8 GPUs and NVSwitch fabric. This is the configuration for 405B FP16 models, 671B at FP8, large-scale training, and maximum single-node performance. Over 1TB of HBM3e memory.

### Bill of Materials (DIY)

| Component | Recommendation | Price | Link |
|-----------|----------------|-------|------|
| GPUs | 8x H200 SXM5 (HGX baseboard + 4x NVSwitch) | $280-320K | [NVIDIA HGX H200](https://www.nvidia.com/en-us/data-center/hgx/) |
| CPUs | Dual AMD EPYC 9654 (96c each) or Intel Xeon 8490H | $18-25K | [AMD EPYC](https://www.amd.com/en/products/processors/server/epyc/4th-generation-9004-and-8004-series.html) |
| RAM | 2TB DDR5 ECC | $10,000 | |
| Storage | 16TB NVMe RAID | $3,000 | |
| PSU | 8kW (6x redundant) | $5,000 | |
| Networking | 8x ConnectX-7 (1 per GPU) | $12,000 | |
| InfiniBand | NDR switch + cables | $15,000 | |
| Cooling | **Liquid cooling (CDU + manifolds)** | $25-35K | |
| **Total** | | **$400-460K** | |

### DGX H200 (Turnkey Alternative)

| Spec | DGX H200 |
|------|----------|
| GPUs | 8x H200 SXM5 141GB |
| GPU Memory | 1,128GB (1.1TB) total |
| Memory Bandwidth | 38.4 TB/s aggregate |
| NVLink | 900 GB/s per GPU, NVSwitch fabric |
| NVLink Aggregate | 7.2 TB/s bidirectional |
| CPUs | Dual Intel Xeon 8480C (56c each) |
| System RAM | 2TB DDR5 |
| Storage | 30TB NVMe |
| Networking | 8x ConnectX-7 400Gb |
| Power | 10.2kW max |
| Price | **$400,000 - $500,000** |
| Link | [NVIDIA DGX H200](https://www.nvidia.com/en-us/data-center/dgx-h200/) |

### PCIe Lane Allocation

GPUs use NVSwitch, not PCIe. All 128 lanes available for peripherals:

| Component | Lanes | Bandwidth | Remaining |
|-----------|-------|-----------|-----------|
| NVMe RAID (8 drives) | x32 | 64 GB/s | 96 |
| 8x ConnectX-7 NICs | x64 | 128 GB/s | 32 |
| Management/BMC | x4 | 8 GB/s | 28 |
| **Available** | | | **28 lanes** |

### Model Performance

| Model | Precision | VRAM Used | Tok/s (batch 1) | Tok/s (batch 64) |
|-------|-----------|-----------|-----------------|------------------|
| Llama 3.1 70B | FP16 | 140GB | ~320 | ~4,800 |
| Llama 3.1 70B | FP8 | 70GB | ~380 | ~5,600 |
| Llama 3.1 405B | FP16 | 810GB | ~40 | ~500 |
| Llama 3.1 405B | FP8 | 405GB | ~65 | ~800 |
| Llama 3.1 405B + 128K | FP8 | 550GB | ~50 | ~600 |
| DeepSeek R1 671B | FP8 | 670GB | ~35 | ~400 |
| DeepSeek R1 671B | Q4 | 335GB | ~50 | ~550 |

**Sweet spot:** 405B at FP16, or 671B at FP8
**Maximum:** 671B at FP8 with moderate context
**H200 advantage:** 405B FP16 fits (H100's 640GB wasn't quite enough)

### Physical Specifications

| Spec | Value |
|------|-------|
| Form factor | 8U rackmount |
| Dimensions | 19" × 14" × 35.3" D |
| Weight | 275 lbs (125 kg) |
| Noise | 75-85 dB (jet engine) |
| Heat output | 27,000-35,000 BTU/hr |
| Rack depth | Requires 48"+ deep rack |
| Cooling | **Liquid cooling mandatory** |
| **Location** | Datacenter with liquid cooling infrastructure |

### Power Requirements

| State | Power Draw | Circuit |
|-------|------------|---------|
| Idle | 1,800W | Multiple 30A/240V |
| Typical | 8,000W | Multiple 30A/240V |
| Peak | 10,000-10,200W | 3x 30A/240V or 2x 50A/240V |

**Infrastructure requirements:**
- 3-phase power recommended
- Industrial PDU with C19/C20
- 15kVA+ UPS or generator backup
- Coolant Distribution Unit (CDU) rated for 12kW+

### Where to Buy

**Turnkey:**
- [NVIDIA DGX H200](https://www.nvidia.com/en-us/data-center/dgx-h200/) - Direct from NVIDIA
- [Dell PowerEdge XE9680](https://www.dell.com/en-us/shop/servers-storage-and-networking/poweredge-xe9680-rack-server/spd/poweredge-xe9680/pe_xe9680_16979_vi_vp) - Dell enterprise (~$290K starting)
- [Broadberry DGX H200 Configurator](https://www.broadberry.com/xeon-scalable-processor-gen4-rackmount-servers/nvidia-dgx-h200)

**System Integrators:**
- [Supermicro SYS-821GE-TNHR](https://www.supermicro.com/en/products/system/gpu/8u/sys-821ge-tnhr) (Intel Xeon) - ~$285K
- [Lenovo ThinkSystem SR680a V3](https://lenovopress.lenovo.com/lp1944-nvidia-h200-141gb-gpu)
- [Viperatech H200 Servers](https://viperatech.com/) - Ships in 2 weeks

**Cloud (if not buying):**
- [Lambda Labs](https://lambdalabs.com/) - ~$3.80/hr per H200
- [CoreWeave](https://www.coreweave.com/) - ~$4.50/hr per H200
- [RunPod](https://www.runpod.io/) - ~$3.50/hr per H200

---

## ⚡ Power Infrastructure

*Sources: [NVIDIA DGX H200 Datasheet](https://resources.nvidia.com/en-us-dgx-systems/dgx-h200-datasheet), [TRG Datacenters](https://www.trgdatacenters.com/resource/nvidia-h200-price-guide/)*

### Power by Configuration

| Config | GPU Power | System Total | Heat (BTU/hr) | Circuit |
|--------|-----------|--------------|---------------|---------|
| 1x NVL | 600W | 800-1,000W | 2,700-3,400 | 15A/120V |
| 2x NVL | 1,200W | 1,600-2,000W | 5,500-6,800 | 20A/120V |
| 4x NVL | 2,400W | 3,000-3,600W | 10,000-12,300 | 30A/240V |
| 4x SXM | 2,800W | 4,500-6,000W | 15,000-20,500 | 40A/240V |
| 8x SXM | 5,600W | 8,500-10,200W | 29,000-35,000 | 3x 30A/240V |

### UPS Sizing

| System Power | UPS VA Rating | Runtime (15 min) |
|--------------|---------------|------------------|
| 1,000W | 1,500 VA | Standard |
| 2,000W | 3,000 VA | Rackmount |
| 3,600W | 5,400 VA | Rackmount |
| 6,000W | 9,000 VA | 3-phase |
| 10,000W | 15,000 VA | 3-phase industrial |

*VA = Watts × 1.5 for typical server power factor*

### PDU Requirements

| Config | Connector Type | PDU Rating |
|--------|---------------|------------|
| 1-2x NVL | C13/C14 | 20A single phase |
| 4x NVL | C19/C20 | 30A single phase |
| 4x SXM | C19/C20 | 40A single phase |
| 8x SXM | C19/C20 | 60A 3-phase |

---

## 🌡️ Cooling Requirements

*Sources: [Supermicro Liquid Cooling](https://www.supermicro.com/en/solutions/liquid-cooling), [Introl Cooling Guide](https://www.introl.io/blog/liquid-cooling-gpu-data-centers-50kw-thermal-limits-guide)*

### Cooling Method by Configuration

| Config | Cooling | Heat Load | Notes |
|--------|---------|-----------|-------|
| 1x NVL | Air | 0.8-1kW | Aggressive case fans |
| 2x NVL | Air | 1.6-2kW | High-airflow case |
| 4x NVL | Air (aggressive) | 3-3.6kW | Hot/cold aisle required |
| 4x SXM | **Liquid** | 4.5-6kW | Direct-to-chip cooling |
| 8x SXM | **Liquid** | 8.5-10kW | CDU infrastructure |

### Air Cooling Limits

- Practical max per rack: 20-30kW with excellent airflow
- Above 30kW: Requires 7,850+ CFM (hurricane-force)
- DGX H200 at 10kW: Maximum 3-4 per standard rack

### Liquid Cooling Benefits

| Metric | Air | Liquid |
|--------|-----|--------|
| Cooling capacity | 20-30kW/rack | 80-100kW/rack |
| Power overhead | 40% of IT load | 10% of IT load |
| Noise | 75-85 dB | 45-55 dB |
| GPU temp consistency | ±10°C | ±2°C |

### Liquid Cooling Infrastructure

| Component | Purpose | Cost |
|-----------|---------|------|
| CDU (Coolant Distribution Unit) | Pump + heat exchanger | $15-30K |
| Manifolds | Per-rack distribution | $2-5K |
| Cold plates | GPU contact | Included with SXM |
| Facility water loop | Heat rejection | Varies |

**Vendors:**
- [Supermicro Liquid Cooling](https://www.supermicro.com/en/solutions/liquid-cooling)
- [Asetek](https://www.asetek.com/)
- [CoolIT Systems](https://www.coolitsystems.com/)

---

## 🏗️ Rack and Facility Requirements

### Rack Specifications

| Config | Form Factor | Weight | Depth Required |
|--------|-------------|--------|----------------|
| 1x NVL | Tower/4U | 50-65 lbs | 26" |
| 2x NVL | 4U | 75-90 lbs | 28" |
| 4x NVL | 4U-5U | 100-120 lbs | 30" |
| 4x SXM | 5U | 140-170 lbs | 32" |
| 8x SXM | 8U | 275 lbs | 35" |

**Standard rack depth:** 36"
**GPU server racks:** 42-48" recommended

### Weight Considerations

- Standard rack: 250-500 lbs total capacity
- GPU racks: Need 1,000+ lbs capacity
- Floor loading: May require reinforcement for multiple racks

### Noise Levels

| Config | Noise Level | Comparison |
|--------|-------------|------------|
| 1x NVL | 50-60 dB | Loud conversation |
| 2x NVL | 60-70 dB | Vacuum cleaner |
| 4x NVL | 70-80 dB | Lawn mower |
| 8x SXM | 75-85 dB | Jet engine at distance |

**Isolation requirements:**
- 1-2 GPU: Separate room with door
- 4 GPU: Dedicated server room
- 8 GPU: Datacenter with sound isolation

---

## 💰 Total Cost of Ownership (3-Year)

### Electricity Costs

*Assuming $0.12/kWh, 24/7 operation*

| Config | Power | Monthly | 3-Year |
|--------|-------|---------|--------|
| 1x NVL | 850W avg | $74 | $2,660 |
| 2x NVL | 1.7kW avg | $147 | $5,290 |
| 4x NVL | 3.2kW avg | $276 | $9,950 |
| 4x SXM | 5kW avg | $432 | $15,550 |
| 8x SXM | 9kW avg | $778 | $28,000 |

### Cooling Costs

Rule of thumb: Cooling = 30-50% of compute power cost

| Config | Cooling Power | Monthly | 3-Year |
|--------|---------------|---------|--------|
| 1x NVL (air) | 250W | $22 | $790 |
| 4x NVL (air) | 1kW | $86 | $3,100 |
| 8x SXM (liquid) | 1.8kW | $156 | $5,600 |

### Total 3-Year TCO

| Config | Hardware | Electricity | Cooling | Support | **Total** |
|--------|----------|-------------|---------|---------|-----------|
| 1x NVL | $47K | $2.7K | $0.8K | $3K | **$53.5K** |
| 2x NVL | $97K | $5.3K | $1.7K | $5K | **$109K** |
| 4x NVL | $200K | $10K | $3.1K | $12K | **$225K** |
| 4x SXM | $285K | $15.5K | $5K | $18K | **$324K** |
| 8x SXM | $470K | $28K | $5.6K | $30K | **$534K** |

### Cloud Break-Even

| Provider | $/hr/H200 | Break-even vs Tier 2 | Break-even vs Tier 5 |
|----------|-----------|----------------------|----------------------|
| Lambda | $3.80 | 14 months | 16 months |
| CoreWeave | $4.50 | 12 months | 13 months |
| Azure | $10.60 | 5 months | 6 months |

**Rule of thumb:** Buy if >60% utilization for 12+ months

---

## 🔗 Related Concepts

**GPUs:**
- [[H200]] - The GPU itself (specs, architecture)
- [[H100]] - Previous Hopper variant (80GB)
- [[A100]] - Previous generation datacenter GPU
- [[B200]] - Blackwell architecture

**NVIDIA Platforms:**
- [[DGX]] - Turnkey AI supercomputer systems
- [[HGX]] - GPU baseboard for OEM builds
- [[MGX]] - Modular server specification
- [[NVLink]] - GPU interconnect technology
- [[NVSwitch]] - Multi-GPU fabric switch

**System Components:**
- [[AMD EPYC]] - Server CPU option
- [[Intel Xeon]] - Server CPU option
- [[ConnectX-7]] - High-speed networking
- [[InfiniBand]] - Low-latency interconnect
- [[Fabric Manager]] - NVSwitch orchestration software

**Related Guides:**
- [[H100 Builds]] - H100-based system builds
- [[A100 Builds]] - A100-based system builds
- [[LLM Inference Hardware]] - GPU comparison overview
- [[LLM Under Your Floorboards]] - Consumer GPU builds
- [[Off-Grid LLM Power Systems]] - Solar/battery for these systems
- [[Quantization]] - Reducing VRAM requirements
- [[vLLM]] - Inference engine
- [[TensorRT-LLM]] - NVIDIA's inference engine

---

## 📚 External Resources

### Official Documentation
- [NVIDIA H200 Product Page](https://www.nvidia.com/en-us/data-center/h200/)
- [NVIDIA DGX H200](https://www.nvidia.com/en-us/data-center/dgx-h200/)
- [DGX H200 Datasheet](https://resources.nvidia.com/en-us-dgx-systems/dgx-h200-datasheet)
- [H200 NVL Datasheet (PNY)](https://www.pny.com/file%20library/company/support/linecards/data-center-gpus/h200-nvl-datasheet.pdf)

### Platform Documentation
- [NVIDIA HGX Platform](https://www.nvidia.com/en-us/data-center/hgx/)
- [NVIDIA MGX Platform](https://www.nvidia.com/en-us/data-center/products/mgx/)
- [DGX vs HGX Comparison (ServeTheHome)](https://www.servethehome.com/nvidia-dgx-versus-nvidia-hgx-what-is-the-difference/)

### CPU Resources
- [AMD EPYC 9004 Series](https://www.amd.com/en/products/processors/server/epyc/4th-generation-9004-and-8004-series.html)
- [Intel Xeon Scalable](https://www.intel.com/content/www/us/en/products/details/processors/xeon/scalable.html)

### System Vendors
- [Supermicro H200 Systems](https://www.supermicro.com/en/accelerators/nvidia/hopper-ada-lovelace)
- [Supermicro Store](https://store.supermicro.com/)
- [Lenovo H200 Product Guide](https://lenovopress.lenovo.com/lp1944-nvidia-h200-141gb-gpu)
- [Thinkmate GPU Servers](https://www.thinkmate.com/systems/servers/gpx)
- [Exxact GPU Servers](https://www.exxactcorp.com/gpu-servers)
- [Viperatech](https://viperatech.com/)
- [Arc Compute](https://www.arccompute.io/)

### Comparisons and Benchmarks
- [H200 vs H100 Comparison (RunPod)](https://www.runpod.io/articles/comparison/nvidia-h200-vs-h100-choosing-the-right-gpu-for-massive-llm-inference)
- [H200 vs H100 (TRG Datacenters)](https://www.trgdatacenters.com/resource/nvidia-h200-vs-h100/)
- [H200 Inference Evaluation (Baseten)](https://www.baseten.co/blog/evaluating-nvidia-h200-gpus-for-llm-inference/)
- [Multi-GPU Benchmark B200 vs H200 vs H100](https://research.aimultiple.com/multi-gpu/)

### Pricing and Market
- [H200 Price Guide (JarvisLabs)](https://docs.jarvislabs.ai/blog/h200-price)
- [H200 Price Comparison (ThunderCompute)](https://www.thundercompute.com/blog/nvidia-h200-pricing)
- [H200 Cost Guide (Cerebrium)](https://www.cerebrium.ai/articles/how-much-does-a-h200-cost-2025-guide)

### Used Market
- [Newegg H200](https://www.newegg.com/p/pl?d=h200)
- eBay (search "H200 NVL" - limited supply as of early 2025)

<!--
================================================================================
TEMPLATE NOTES FOR FUTURE GPU BUILD DOCUMENTS (B200, B100, etc.)
================================================================================

This document follows the H100 Builds template structure.

SECTION REUSABILITY (from H100 template):
| Section                    | Reuse | Notes                                    |
|----------------------------|-------|------------------------------------------|
| Intro paragraph            | 20%   | Rewrite for that GPU                     |
| Where [GPU] Fits           | 30%   | Update generation table, decision matrix |
| vs Previous/Next Gen       | 0%    | Rebuild comparison table entirely        |
| Software Requirements      | 70%   | Update version numbers                   |
| Quick Reference            | 0%    | Rebuild for that GPU's tiers             |
| GPU Variants               | 10%   | Different variants per GPU               |
| Platform Overview          | 80%   | Update "[GPU] Relevance" column only     |
| HGX Deep Dive              | 60%   | Update specs tables for that GPU         |
| DGX Deep Dive              | 40%   | Update for that GPU's DGX version        |
| MGX/EGX/AGX                | 95%   | Almost entirely reusable                 |
| Model-to-Hardware          | 0%    | Recalculate for that GPU's VRAM          |
| Tiers 1-5                  | 30%   | Same structure, all new specs            |
| Power Infrastructure       | 70%   | Update TDP values                        |
| Cooling Requirements       | 80%   | Update heat loads                        |
| Rack/Facility              | 90%   | Mostly reusable                          |
| TCO                        | 50%   | Recalculate all numbers                  |
| Related Concepts           | 70%   | Swap GPU links                           |
| External Resources         | 30%   | Most links change per GPU                |

H200-SPECIFIC NOTES:
- H200 only has two variants: SXM5 and NVL PCIe (no standalone PCIe 80GB like H100)
- All H200s have 141GB HBM3e (no 80GB variant)
- H200 NVL is 600W TDP (vs H100 NVL 400W)
- Same compute as H100 (same TFLOPS, same architecture)
- Same NVLink 4th gen, NVSwitch 3rd gen (900 GB/s)
- Key advantage: 70B FP16 fits on single GPU

NVLink/NVSwitch GENERATIONS:
| GPU  | NVLink Gen | NVSwitch Gen | Bandwidth |
|------|------------|--------------|-----------|
| A100 | 3rd        | 2nd          | 600 GB/s  |
| H100 | 4th        | 3rd          | 900 GB/s  |
| H200 | 4th        | 3rd          | 900 GB/s  |
| B200 | 5th        | 4th          | 1800 GB/s |

PRICING NOTES:
- GPU prices change rapidly. Always note verification date.
- H200 is ~15-25% more expensive than H100 for equivalent configs
- Lead times currently 12-26 weeks
- Cloud rental ~$3.50-$4.50/hr for H200 vs ~$2-3/hr for H100

================================================================================
-->
