# H100 Builds

A comprehensive guide to building systems around NVIDIA's H100 Tensor Core GPU—the current datacenter AI workhorse. Covers variants, specifications, what models fit on different configurations, complete build specs, PCIe lane allocation, cooling, power, physical requirements, and pricing.

For consumer GPU builds see [[LLM Under Your Floorboards]]. For comparisons with other datacenter GPUs see [[LLM Inference Hardware]].

*Pricing last verified: January 2025. GPU prices fluctuate significantly based on supply, demand, and new releases.*

---

## 📍 Where H100 Fits

The H100 is NVIDIA's Hopper architecture datacenter GPU, released in 2022. It sits between the previous-generation A100 (Ampere) and the upcoming B200 (Blackwell).

| Generation | Architecture | Flagship GPU | VRAM | Release | Status |
|------------|--------------|--------------|------|---------|--------|
| Ampere | GA100 | [[A100]] | 40/80GB HBM2e | 2020 | Mature, discounted |
| **Hopper** | **GH100** | **[[H100]]** | **80GB HBM3** | **2022** | **Current mainstream** |
| Hopper Refresh | GH200 | [[H200]] | 141GB HBM3e | 2024 | Premium tier |
| Blackwell | GB100/GB200 | [[B200]] | 192GB HBM3e | 2024-25 | Next generation |

### When to Choose H100

| Scenario | Best Choice | Why |
|----------|-------------|-----|
| Maximum performance, budget flexible | H200 or B200 | More VRAM, higher bandwidth |
| Best price/performance today | **H100** | Mature supply chain, competitive pricing |
| Budget-constrained, used OK | A100 | 40-60% cheaper, still capable |
| Future-proofing for 1T+ models | B200 | 192GB VRAM, NVLink 5 |
| Need it deployed this month | **H100** | Best availability |

### H100 vs A100 vs H200 vs B200

| Spec | A100 80GB | H100 80GB | H200 141GB | B200 192GB |
|------|-----------|-----------|------------|------------|
| Architecture | Ampere | Hopper | Hopper | Blackwell |
| VRAM | 80GB HBM2e | 80GB HBM3 | 141GB HBM3e | 192GB HBM3e |
| Memory BW | 2.0 TB/s | 3.35 TB/s | 4.8 TB/s | 8.0 TB/s |
| FP8 Tensor | N/A | 3,958 TFLOPS | 3,958 TFLOPS | 9,000 TFLOPS |
| FP16 Tensor | 312 TFLOPS | 1,979 TFLOPS | 1,979 TFLOPS | 4,500 TFLOPS |
| TDP (SXM) | 400W | 700W | 700W | 1,000W |
| NVLink | 600 GB/s | 900 GB/s | 900 GB/s | 1,800 GB/s |
| PCIe Price | ~$12-18K | ~$25-35K | ~$30-40K | TBD (~$40-50K) |
| **LLM Inference** | 1x baseline | **~2x A100** | ~2.5x A100 | ~4x A100 |

**Bottom line:**
- **A100 → H100**: ~2x inference performance, 1.7x memory bandwidth, worth the upgrade
- **H100 → H200**: Same compute, 76% more VRAM, 43% more bandwidth—worth it for large models
- **H100 → B200**: ~2x compute, 2.4x VRAM—significant but wait for pricing/availability

---

## 🔧 Software Requirements

*Sources: [NVIDIA CUDA Toolkit](https://developer.nvidia.com/cuda-toolkit), [PyTorch Compatibility](https://pytorch.org/get-started/locally/)*

H100 requires newer software stacks than A100. Verify compatibility before purchasing.

### Minimum Versions

| Component | Minimum | Recommended | Notes |
|-----------|---------|-------------|-------|
| CUDA Toolkit | 11.8 | 12.4+ | FP8 requires CUDA 12+ |
| NVIDIA Driver | 520.x | 550.x+ | Match CUDA version |
| cuDNN | 8.6 | 9.x | For deep learning frameworks |
| TensorRT | 8.5 | 10.x | For optimized inference |
| PyTorch | 2.0 | 2.2+ | Native H100 support |
| TensorFlow | 2.12 | 2.15+ | XLA compilation support |

### Framework Considerations

| Framework | H100 Support | FP8 Support | Notes |
|-----------|--------------|-------------|-------|
| [[PyTorch]] | Native (2.0+) | Via Transformer Engine | Best flexibility |
| [[TensorFlow]] | Native (2.12+) | Limited | XLA required for best perf |
| [[JAX]] | Native | Via Transformer Engine | Good for research |
| [[vLLM]] | Excellent | Yes | Recommended for inference |
| [[TensorRT-LLM]] | Excellent | Yes | Best raw performance |
| [[llama.cpp]] | Good | No (INT8/INT4 only) | CPU offload capable |

### Container Images

Pre-built containers with H100 optimization:
- [NGC PyTorch](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/pytorch) - `nvcr.io/nvidia/pytorch:24.01-py3`
- [NGC TensorRT-LLM](https://catalog.ngc.nvidia.com/orgs/nvidia/containers/tensorrt) - For production inference
- [vLLM Docker](https://docs.vllm.ai/en/latest/serving/deploying_with_docker.html) - Easy inference serving

---

## 📋 Quick Reference

| Tier | GPUs | VRAM | Best Models | Tokens/s (70B) | Budget | Form Factor | Location |
|------|------|------|-------------|----------------|--------|-------------|----------|
| 1 | 1x H100 PCIe | 80GB | 32B FP16, 70B FP8 | ~25 tok/s | $35-45K | Tower/4U | Closet/basement |
| 2 | 2x H100 NVL | 188GB | 70B FP16, 140B FP8 | ~50 tok/s | $75-95K | 4U | Dedicated room |
| 3 | 4x H100 PCIe | 320GB | 200B FP8, 405B Q4 | ~90 tok/s | $150-180K | 4U-5U | Server room |
| 4 | 4x H100 SXM | 320GB | 200B FP16, 405B FP8 | ~120 tok/s | $200-250K | 5U | Server room |
| 5 | 8x H100 SXM | 640GB | 405B FP16, 671B FP8 | ~200 tok/s | $350-450K | 8U | Datacenter |

---

## 🎯 H100 Variants

*Sources: [Hyperstack Comparison](https://www.hyperstack.cloud/technical-resources/performance-benchmarks/comparing-nvidia-h100-pcie-vs-sxm-performance-use-cases-and-more), [Verda Blog](https://verda.com/blog/pcie-and-sxm5-comparison), [NVIDIA H100 Page](https://www.nvidia.com/en-us/data-center/h100/)*

| Variant | VRAM | Memory BW | TDP | NVLink BW | Interface | Cooling |
|---------|------|-----------|-----|-----------|-----------|---------|
| **H100 PCIe** | 80GB HBM2e | 2,000 GB/s | 350W | Optional (bridge) | PCIe 5.0 x16 | Air |
| **H100 NVL PCIe** | 94GB HBM3 | 3,900 GB/s | 400W | 600 GB/s (pairs) | PCIe 5.0 x16 | Air |
| **H100 SXM5** | 80GB HBM3 | 3,350 GB/s | 700W | 900 GB/s (mesh) | SXM socket | Liquid |

### Which Variant to Choose

| Use Case | Recommended Variant | Why |
|----------|---------------------|-----|
| Single GPU inference | H100 PCIe 80GB | Lowest cost, easiest integration |
| Paired inference (70B) | H100 NVL 94GB | More VRAM, native NVLink |
| Multi-GPU training | H100 SXM5 | Full NVLink mesh, highest bandwidth |
| Maximum VRAM per card | H100 NVL 94GB | 94GB vs 80GB on SXM |
| Maximum multi-GPU perf | H100 SXM5 | 900 GB/s NVLink, NVSwitch support |

---

## 🖥️ NVIDIA Platform Overview

*Sources: [ServeTheHome DGX vs HGX](https://www.servethehome.com/nvidia-dgx-versus-nvidia-hgx-what-is-the-difference/), [Server-Parts Platform Comparison](https://www.server-parts.eu/post/nvidia-ai-platform-dgx-hgx-egx-agx-comparison), [NVIDIA HGX Platform](https://www.nvidia.com/en-us/data-center/hgx/), [NVIDIA MGX](https://www.nvidia.com/en-us/data-center/products/mgx/)*

NVIDIA offers multiple platform tiers for datacenter AI. Understanding these is critical for H100 builds since SXM GPUs require specific baseboards.

| Platform | What It Is | Customization | Target Buyer | H100 Relevance |
|----------|------------|---------------|--------------|----------------|
| **[[DGX]]** | Complete turnkey system | Fixed | Enterprise (plug-and-play) | DGX H100 is the reference 8-GPU system |
| **[[HGX]]** | GPU baseboard + NVSwitch | High (OEM builds around it) | OEMs, cloud providers | Required for SXM builds |
| **[[MGX]]** | Modular server spec | Very high (multi-gen) | System builders | Emerging standard for flexibility |
| **EGX** | Edge AI platform | Moderate | Edge deployments | Not applicable (uses A100/L40) |
| **AGX** | Embedded autonomous | High | Robotics, automotive | Not applicable (Jetson/Orin) |

### When to Choose Each

| Scenario | Best Platform | Why |
|----------|---------------|-----|
| "I want it working tomorrow" | DGX | Pre-configured, NVIDIA support, validated |
| "I need AMD EPYC CPUs" | HGX + OEM | DGX uses Intel; HGX lets you choose |
| "I need specific storage/networking" | HGX + OEM | Customize around the baseboard |
| "Building for cloud customers" | HGX | Cloud providers all use HGX |
| "Future GPU upgrades matter" | MGX | Designed for multi-generational reuse |
| "Budget is critical" | HGX + self-build | Skip NVIDIA's DGX premium |

---

## 🔲 HGX Platform Deep Dive

*Sources: [NVIDIA HGX Tech Blog](https://developer.nvidia.com/blog/introducing-nvidia-hgx-h100-an-accelerated-server-platform-for-ai-and-high-performance-computing/), [DirectMacro HGX Guide](https://directmacro.com/blog/post/nvidia-hgx-h100-baseboard-for-mining), [NVIDIA HGX Page](https://www.nvidia.com/en-us/data-center/hgx/)*

HGX (Hyperscale Graphics eXtension) is the GPU baseboard that sits at the heart of all SXM-based systems. When you buy a DGX, you're buying an HGX baseboard plus everything else (CPUs, RAM, storage, chassis, support). When you buy HGX directly, you build the rest yourself or through an OEM.

### What's on an HGX Baseboard

| Component | HGX H100 4-GPU | HGX H100 8-GPU |
|-----------|----------------|----------------|
| GPUs | 4x H100 SXM5 80GB | 8x H100 SXM5 80GB |
| Total VRAM | 320GB HBM3 | 640GB HBM3 |
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
- Lower cost (~$140-160K for baseboard)
- Good for: inference, smaller training jobs, 70B-200B models

**8-GPU Configuration:**
- GPUs connect through 4x NVSwitch chips
- Full mesh: any GPU can talk to any other at 900 GB/s simultaneously
- Required for: 405B+ models, large-scale training, collective operations
- Higher cost (~$215-280K for baseboard)
- Supports NVLink-Network for multi-node scaling (up to 256 GPUs)

### HGX Performance Specifications

| Metric | HGX H100 4-GPU | HGX H100 8-GPU |
|--------|----------------|----------------|
| FP8 Tensor | 16 PFLOPS | 32 PFLOPS |
| FP16 Tensor | 8 PFLOPS | 16 PFLOPS |
| Memory bandwidth | 13.2 TB/s aggregate | 26.4 TB/s aggregate |
| NVLink bandwidth | 3.6 TB/s total | 7.2 TB/s total |
| TDP (GPUs only) | 2,800W | 5,600W |
| System power (typical) | 4-5 kW | 8-10 kW |

### HGX Pricing (2024-2025)

| Configuration | Baseboard Only | With Typical System | Notes |
|---------------|----------------|---------------------|-------|
| HGX H100 4-GPU | $140-160K | $200-250K | Includes CPUs, RAM, chassis |
| HGX H100 8-GPU | $215-280K | $350-450K | Requires liquid cooling |
| HGX H200 8-GPU | $280-350K | $450-550K | 141GB HBM3e per GPU |

*Prices vary significantly based on market conditions and quantity*

### Where to Buy HGX Baseboards

**Direct from NVIDIA partners:**
- [Supermicro](https://store.supermicro.com/) - GPU-NVHGX-H100 series
- [ASUS](https://www.asus.com/networking-iot-servers/servers/) - ESC8000A series
- [Gigabyte](https://www.gigabyte.com/Enterprise/GPU-Server) - G593 series

**Resellers:**
- [DirectMacro](https://directmacro.com/) - New baseboards
- [SabrePC](https://www.sabrepc.com/) - Enterprise configurations
- [Eton Technology](https://etontechnology.com/) - New with warranty
- [VipheraTech](https://viperatech.com/) - New and refurbished

**Used market:**
- eBay (search "HGX H100" - verify seller reputation)
- [ServerMonkey](https://www.servermonkey.com/) - Refurbished enterprise

### Building Around HGX

When you buy an HGX baseboard, you still need:

| Component | Options | Considerations |
|-----------|---------|----------------|
| **CPU host board** | Supermicro H13, ASUS, Gigabyte | Must support HGX connector |
| **CPUs** | AMD EPYC 9004, Intel Xeon 4th/5th gen | DGX uses Intel; HGX gives you choice |
| **RAM** | 1-2TB DDR5 ECC | Match CPU memory channels |
| **Storage** | NVMe RAID, often 8-16TB | High-speed for checkpoints |
| **Networking** | ConnectX-7, InfiniBand | 1-2 NICs per GPU for multi-node |
| **Chassis** | 5U-8U rackmount | Must fit HGX + cooling |
| **Liquid cooling** | CDU + manifolds | Mandatory for SXM |
| **Power** | 6-12 kW capacity | 3-phase recommended for 8-GPU |

### HGX vs Building with PCIe Cards

| Factor | HGX (SXM) | PCIe Cards |
|--------|-----------|------------|
| GPU-GPU bandwidth | 900 GB/s (NVLink) | 64 GB/s (PCIe 5.0 x16) |
| Cooling | Liquid required | Air possible |
| Flexibility | Locked to HGX ecosystem | Any server chassis |
| Multi-node scaling | NVLink-Network ready | PCIe/InfiniBand only |
| Cost per GPU | Higher | Lower |
| Training efficiency | Excellent | Good (smaller models) |
| Inference efficiency | Excellent | Excellent |

**Rule of thumb:**
- Training large models (70B+): HGX is worth the premium
- Inference or smaller training: PCIe cards often sufficient

---

## 🔷 DGX Platform Deep Dive

*Sources: [NVIDIA DGX H100](https://www.nvidia.com/en-us/data-center/dgx-h100/), [DGX User Guide](https://docs.nvidia.com/dgx/dgxh100-user-guide/)*

DGX is NVIDIA's turnkey solution—an HGX baseboard with everything else pre-integrated and validated.

### DGX H100 Specifications

| Spec | DGX H100 |
|------|----------|
| GPUs | 8x H100 SXM5 80GB |
| GPU Memory | 640GB HBM3 |
| NVLink | 900 GB/s per GPU, 4x NVSwitch |
| CPUs | Dual Intel Xeon 8480+ (56c each, 112c total) |
| System RAM | 2TB DDR5 |
| Storage | 30TB NVMe (8x 3.84TB) |
| Networking | 8x ConnectX-7 400Gb (3.2 Tb/s aggregate) |
| Interconnect | 4x OSFP ports for InfiniBand/Ethernet |
| Power | 10.2 kW max |
| Cooling | Liquid (rear-door or direct) |
| Dimensions | 19" × 14" × 35.3" (8U) |
| Weight | 275 lbs (125 kg) |
| **List Price** | $300,000 - $450,000 |

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

### DGX Pricing Reality

| What You Pay For | Value |
|------------------|-------|
| Hardware (HGX equivalent) | ~$280K |
| Intel CPUs + RAM + storage | ~$35K |
| Networking (8x CX-7) | ~$12K |
| Chassis + cooling | ~$25K |
| Integration + validation | ~$30K |
| NVIDIA support (3 yr) | ~$50K |
| **Total DGX cost** | **~$430K** |

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

- Planning multi-year infrastructure with GPU upgrades
- Need smaller form factors (1U-2U)
- Want mix of GPU types (H100 PCIe + L40S in same chassis)
- Building standardized fleet across different workloads

**For H100 SXM builds specifically:** HGX remains the standard. MGX is more relevant for mixed deployments or when planning H200/B200 upgrade paths.

### MGX Partners

- [Supermicro MGX Systems](https://www.supermicro.com/en/accelerators/nvidia/mgx)
- [Gigabyte MGX Server](https://www.gigabyte.com/Enterprise/MGX-Server)
- [MSI MGX Servers](https://www.msi.com/Landing/NVIDIA-MGX)
- [ASUS](https://www.asus.com/networking-iot-servers/)

---

## 📍 EGX & AGX (Edge Platforms)

*For completeness—not directly relevant to H100 datacenter builds*

### EGX (Edge GPU eXtension)

Edge AI platform for deployments outside datacenters:
- Uses A100 PCIe or L40 (not H100 SXM)
- Designed for: manufacturing, retail, healthcare, telco
- Smaller form factor, lower power
- Remote fleet management

### AGX (Autonomous GPU eXtension)

Embedded platforms for autonomous systems:
- **Jetson AGX Orin**: 275 TOPS, 60W, for robots/drones
- **Jetson AGX Thor**: 2000+ TOPS, 130W, for humanoids/vehicles
- **IGX Orin/Thor**: Industrial-grade variants

These use different GPU architectures (Ampere/Blackwell embedded) and aren't compatible with H100 datacenter deployments.

---

## 📊 Model-to-Hardware Mapping

*Sources: [Hyperstack VRAM Guide](https://www.hyperstack.cloud/blog/case-study/how-much-vram-do-you-need-for-llms), [NVIDIA TensorRT-LLM Blog](https://developer.nvidia.com/blog/achieving-top-inference-performance-with-the-nvidia-h100-tensor-core-gpu-and-nvidia-tensorrt-llm/)*

### VRAM Requirements

| Model | FP16 | FP8 | Q4 | Notes |
|-------|------|-----|----|----|
| Llama 3.1 8B | 16GB | 8GB | 4GB | Single GPU easy |
| Qwen2.5 32B | 64GB | 32GB | 18GB | Fits 1x H100 |
| Llama 3.1 70B | 140GB | 70GB | 35GB | 1x H100 at FP8 |
| Llama 3.1 70B + 128K ctx | 180GB | 90GB | 55GB | 2x H100 recommended |
| Mixtral 8x22B | 176GB | 88GB | 44GB | 2x H100 |
| Llama 3.1 405B | 810GB | 405GB | 200GB | 8x H100 at FP8 |
| DeepSeek R1 671B | 1.3TB | 670GB | 335GB | 8x+ H100 |

*Add ~20% overhead for activations and KV cache*

### Minimum H100 Count by Model

| Model | FP16 | FP8 | Q4 | Optimal Config |
|-------|------|-----|-----|----------------|
| 8B | 1 | 1 | 1 | 1x any variant |
| 32B | 1 | 1 | 1 | 1x any variant |
| 70B | 2 | 1 | 1 | 2x NVL or 1x SXM |
| 70B + long context | 3 | 2 | 1 | 2x NVL |
| 140B | 4 | 2 | 1 | 4x SXM |
| 200B | 4 | 3 | 2 | 4x SXM |
| 405B | 10+ | 8 | 4 | 8x SXM (DGX) |
| 671B | 16+ | 8-10 | 5 | 8x SXM + aggressive quant |

---

## 🔧 Tier 1: Single H100 PCIe (~$35-45K)

### Overview
Entry point for datacenter-class inference. Runs 32B models at full precision, 70B at FP8. Good for API serving, development, and smaller production workloads.

### Bill of Materials

| Component | Recommendation | Price | Link |
|-----------|----------------|-------|------|
| GPU | 1x H100 PCIe 80GB | $30-38K | [NVIDIA H100](https://www.nvidia.com/en-us/data-center/h100/) |
| CPU | AMD EPYC 9124 (16c/32t) | $1,100 | [[AMD EPYC]] / [AMD Store](https://www.amd.com/en/products/processors/server/epyc/4th-generation-9004-and-8004-series.html) |
| Motherboard | Supermicro H13SSL-N | $800 | [[Supermicro]] / [Supermicro Store](https://store.supermicro.com/) |
| RAM | 256GB DDR5-4800 ECC (8x32GB) | $1,200 | |
| Storage | 2TB NVMe Gen4 | $200 | |
| PSU | 1600W 80+ Titanium | $400 | |
| Case | 4U rackmount or tower | $300-500 | |
| **Total** | | **$34-42K** | |

### PCIe Lane Allocation

**CPU:** AMD EPYC 9124 provides **128 PCIe 5.0 lanes**

| Component | Lanes | Bandwidth | Remaining |
|-----------|-------|-----------|-----------|
| H100 GPU | x16 | 128 GB/s | 112 |
| Boot NVMe | x4 | 8 GB/s | 108 |
| Data NVMe | x4 | 8 GB/s | 104 |
| 25GbE NIC | x8 | 32 GB/s | 96 |
| **Available** | | | **96 lanes** |

Plenty of room for additional storage, 100GbE upgrade, or second GPU later.

### Model Performance

| Model | Precision | VRAM Used | Tok/s (batch 1) | Tok/s (batch 16) |
|-------|-----------|-----------|-----------------|------------------|
| Llama 3.1 8B | FP16 | 16GB | ~120 | ~1,500 |
| Qwen2.5 32B | FP16 | 64GB | ~40 | ~400 |
| Llama 3.1 70B | FP8 | 70GB | ~25 | ~250 |
| Llama 3.1 70B | Q4 | 35GB | ~30 | ~300 |
| Mixtral 8x7B | FP16 | 90GB | Won't fit | - |

**Sweet spot:** 32B at FP16 or 70B at FP8/Q4
**Maximum:** 70B at FP8 with limited context (~32K)

### Physical Specifications

| Spec | Value |
|------|-------|
| Form factor | Tower workstation or 4U rackmount |
| Dimensions | 17" W × 7" H × 26" D (tower) |
| Weight | 45-60 lbs (20-27 kg) |
| Noise | 45-55 dB (noticeable but tolerable) |
| Heat output | 2,000-2,700 BTU/hr |
| **Location** | Large closet, basement, home office with door |

### Power Requirements

| State | Power Draw | Circuit |
|-------|------------|---------|
| Idle | 200W | Standard 15A/120V OK |
| Typical | 500W | Standard 15A/120V OK |
| Peak | 700-800W | 15A/120V OK (with headroom) |

Recommended: Dedicated 15A circuit, 1000VA UPS

### Where to Buy

**New:**
- [Supermicro GPU Workstations](https://www.supermicro.com/en/products/gpu)
- [BIZON ZX4000](https://bizon-tech.com/bizon-zx4000.html) - Custom water-cooled
- [Thinkmate GPU Workstations](https://www.thinkmate.com/systems/workstations/gpx)
- [Exxact Workstations](https://www.exxactcorp.com/gpu-workstations)

**Used:**
- eBay (search "H100 PCIe")
- [ServerMonkey](https://www.servermonkey.com/)

---

## 🔧 Tier 2: Dual H100 NVL (~$75-95K)

### Overview
The sweet spot for 70B models at full precision. NVLink bridge provides 600 GB/s GPU-to-GPU bandwidth. Excellent for production inference and fine-tuning.

### Bill of Materials

| Component | Recommendation | Price | Link |
|-----------|----------------|-------|------|
| GPUs | 2x H100 NVL PCIe 94GB + NVLink bridge | $65-75K | [NVIDIA H100](https://www.nvidia.com/en-us/data-center/h100/) |
| CPU | AMD EPYC 9354 (32c/64t) | $2,500 | [AMD EPYC 9004](https://www.amd.com/en/products/processors/server/epyc/4th-generation-9004-and-8004-series.html) |
| Motherboard | Supermicro H13DSG-OM (dual socket capable) | $1,500 | [Supermicro H13](https://www.supermicro.com/en/products/motherboards?pro=H13) |
| RAM | 512GB DDR5-4800 ECC | $2,400 | |
| Storage | 4TB NVMe Gen4 RAID | $400 | |
| PSU | 2000W redundant | $800 | |
| Case | 4U rackmount | $400 | |
| Cooling | High-airflow fans | $200 | |
| **Total** | | **$73-83K** | |

### PCIe Lane Allocation

**CPU:** AMD EPYC 9354 provides **128 PCIe 5.0 lanes**

| Component | Lanes | Bandwidth | Remaining |
|-----------|-------|-----------|-----------|
| H100 GPU #1 | x16 | 128 GB/s | 112 |
| H100 GPU #2 | x16 | 128 GB/s | 96 |
| NVMe RAID (2 drives) | x8 | 16 GB/s | 88 |
| 100GbE NIC | x16 | 64 GB/s | 72 |
| **Available** | | | **72 lanes** |

Room for: Additional NVMe, InfiniBand HCA, or 2 more GPUs

### Model Performance

| Model | Precision | VRAM Used | Tok/s (batch 1) | Tok/s (batch 32) |
|-------|-----------|-----------|-----------------|------------------|
| Llama 3.1 70B | FP16 | 140GB | ~50 | ~600 |
| Llama 3.1 70B | FP8 | 70GB | ~65 | ~800 |
| Llama 3.1 70B + 128K | FP16 | 180GB | ~45 | ~500 |
| Mixtral 8x22B | FP16 | 176GB | ~35 | ~400 |
| Llama 3.1 405B | Q4 | 200GB | Won't fit well | - |

**Sweet spot:** 70B at FP16 with full 128K context
**Maximum:** 140B at FP8 or Mixtral 8x22B

### Physical Specifications

| Spec | Value |
|------|-------|
| Form factor | 4U rackmount |
| Dimensions | 17.2" W × 7" H × 28" D |
| Weight | 65-80 lbs (30-36 kg) |
| Noise | 55-65 dB (loud, needs isolation) |
| Heat output | 4,000-5,500 BTU/hr |
| **Location** | Dedicated room, basement server area |

### Power Requirements

| State | Power Draw | Circuit |
|-------|------------|---------|
| Idle | 400W | 20A/120V |
| Typical | 1,000W | 20A/120V |
| Peak | 1,400-1,600W | 20A/120V or 15A/240V |

Recommended: Dedicated 20A/120V or 15A/240V circuit, 2000VA UPS

### Where to Buy

**New:**
- [Supermicro AS-4125GS-TNRT](https://store.supermicro.com/) (~$85-100K configured)
- [BIZON ZX5500](https://bizon-tech.com/bizon-zx5500.html) (~$90-110K)
- [Thinkmate GPX Servers](https://www.thinkmate.com/systems/servers/gpx)
- [Silicon Mechanics](https://www.siliconmechanics.com/)

---

## 🔧 Tier 3: Quad H100 PCIe (~$150-180K)

### Overview
Four PCIe cards without full NVLink mesh. Good for inference where tensor parallelism overhead is acceptable. Limited for training due to PCIe bandwidth constraints between GPUs.

### Bill of Materials

| Component | Recommendation | Price | Link |
|-----------|----------------|-------|------|
| GPUs | 4x H100 PCIe 80GB | $120-150K | [NVIDIA H100](https://www.nvidia.com/en-us/data-center/h100/) |
| CPUs | Dual AMD EPYC 9454 (48c each) | $8,000 | [AMD EPYC 9004](https://www.amd.com/en/products/processors/server/epyc/4th-generation-9004-and-8004-series.html) |
| Motherboard | Supermicro H13DSG-OM | $1,500 | [Supermicro Store](https://store.supermicro.com/) |
| RAM | 1TB DDR5-4800 ECC | $4,800 | |
| Storage | 8TB NVMe RAID | $1,000 | |
| PSU | 3000W redundant | $1,200 | |
| Networking | Mellanox ConnectX-7 100GbE | $1,500 | [[ConnectX-7]] |
| Case | 4U-5U rackmount | $500 | |
| Cooling | High-airflow / hybrid | $500 | |
| **Total** | | **$139-169K** | |

### PCIe Lane Allocation

**CPUs:** Dual AMD EPYC 9454 = **128 usable PCIe 5.0 lanes**
*(Note: 64 lanes per CPU, but 64 go to Infinity Fabric for inter-CPU link)*

| Component | Lanes | Bandwidth | Remaining |
|-----------|-------|-----------|-----------|
| H100 GPU #1 | x16 | 128 GB/s | 112 |
| H100 GPU #2 | x16 | 128 GB/s | 96 |
| H100 GPU #3 | x16 | 128 GB/s | 80 |
| H100 GPU #4 | x16 | 128 GB/s | 64 |
| NVMe RAID (4 drives) | x16 | 32 GB/s | 48 |
| 100GbE NIC | x16 | 64 GB/s | 32 |
| **Available** | | | **32 lanes** |

Room for: InfiniBand HCA for multi-node, additional 100GbE

### Model Performance

| Model | Precision | VRAM Used | Tok/s (batch 1) | Tok/s (batch 64) |
|-------|-----------|-----------|-----------------|------------------|
| Llama 3.1 70B | FP16 | 140GB | ~90 | ~1,200 |
| Llama 3.1 70B | FP8 | 70GB | ~110 | ~1,500 |
| Mixtral 8x22B | FP16 | 176GB | ~70 | ~900 |
| Llama 3.1 405B | FP8 | 405GB | Won't fit | - |
| Llama 3.1 405B | Q4 | 200GB | ~15 | ~150 |

**Sweet spot:** 70B with massive batch sizes, or 200B at FP8
**Maximum:** 405B at Q4 (limited context)
**Limitation:** No NVLink mesh = PCIe bandwidth bottleneck for tensor parallelism

### Physical Specifications

| Spec | Value |
|------|-------|
| Form factor | 4U-5U rackmount |
| Dimensions | 17.2" W × 7-8.75" H × 30" D |
| Weight | 85-100 lbs (38-45 kg) |
| Noise | 65-75 dB (very loud) |
| Heat output | 7,500-9,500 BTU/hr |
| Rack depth | Requires 42"+ deep rack |
| **Location** | Dedicated server room |

### Power Requirements

| State | Power Draw | Circuit |
|-------|------------|---------|
| Idle | 600W | 30A/240V |
| Typical | 1,800W | 30A/240V |
| Peak | 2,400-2,800W | 30A/240V |

Recommended: 30A/240V circuit, 3000VA UPS, PDU with C19 outlets

### Where to Buy

**New:**
- [Supermicro AS-4125GS-TNRT](https://www.supermicro.com/en/products/system/gpu/4u/as-4125gs-tnrt) (~$160-190K)
- [Thinkmate GPX TS4](https://www.thinkmate.com/systems/servers/gpx)
- [Exxact 4U GPU Server](https://www.exxactcorp.com/gpu-servers)

---

## 🔧 Tier 4: Quad H100 SXM (~$200-250K)

### Overview
Four SXM GPUs with full NVLink mesh via NVSwitch. Proper multi-GPU performance for training and large model inference. Requires liquid cooling.

### Bill of Materials

| Component | Recommendation | Price | Link |
|-----------|----------------|-------|------|
| GPUs | 4x H100 SXM5 (HGX baseboard) | $140-160K | [NVIDIA HGX](https://www.nvidia.com/en-us/data-center/hgx/) |
| CPUs | Dual AMD EPYC 9454 or Intel Xeon 8480+ | $10-15K | [AMD EPYC](https://www.amd.com/en/products/processors/server/epyc/4th-generation-9004-and-8004-series.html) / [Intel Xeon](https://www.intel.com/content/www/us/en/products/details/processors/xeon/scalable.html) |
| Motherboard | HGX-compatible (included in chassis) | incl. | |
| RAM | 1TB DDR5-4800 ECC | $4,800 | |
| Storage | 8TB NVMe RAID | $1,000 | |
| PSU | 4000W+ | $2,000 | |
| Networking | ConnectX-7 200GbE or HDR InfiniBand | $3,000 | |
| Cooling | **Liquid cooling (CDU required)** | $8-15K | |
| **Total** | | **$185-215K** | |

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
| Llama 3.1 70B | FP16 | 140GB | ~120 | ~1,800 |
| Llama 3.1 70B | FP8 | 70GB | ~150 | ~2,200 |
| Mixtral 8x22B | FP16 | 176GB | ~100 | ~1,400 |
| Llama 3.1 200B* | FP8 | 200GB | ~50 | ~600 |
| Llama 3.1 405B | FP8 | 405GB | Won't fit | - |

*Hypothetical model size

**Sweet spot:** 70B with maximum throughput, 200B development
**Maximum:** 200B at FP8 with good context

### Physical Specifications

| Spec | Value |
|------|-------|
| Form factor | 5U rackmount |
| Dimensions | 17.2" W × 8.75" H × 32" D |
| Weight | 120-150 lbs (54-68 kg) |
| Noise | 70-80 dB (requires isolation) |
| Heat output | 14,000-17,000 BTU/hr |
| Rack depth | Requires 42"+ deep rack |
| Cooling | **Liquid cooling infrastructure required** |
| **Location** | Server room with liquid cooling loop |

### Power Requirements

| State | Power Draw | Circuit |
|-------|------------|---------|
| Idle | 800W | 40A/240V |
| Typical | 3,500W | 40A/240V |
| Peak | 4,500-5,000W | 40A/240V |

Recommended: 40A/240V circuit, industrial UPS, CDU for cooling

### Where to Buy

**Pre-built (recommended for SXM):**
- [Supermicro SYS-421GU-TNXR](https://www.supermicro.com/en/products/system/gpu/4u/sys-421gu-tnxr) (~$220-260K)
- [Supermicro SYS-521GU-TNXR](https://www.supermicro.com/en/products/system/gpu/5u/sys-521gu-tnxr) (~$230-270K)
- [Dell PowerEdge XE8640](https://www.dell.com/en-us/shop/servers-storage-and-networking/poweredge-xe8640-rack-server/spd/poweredge-xe8640/pe_xe8640_16902_vi_vp)

---

## 🔧 Tier 5: 8x H100 SXM / DGX H100 (~$350-450K)

### Overview
Full HGX baseboard with 8 GPUs and NVSwitch fabric. This is the configuration for 405B models, large-scale training, and maximum single-node performance. Essentially a small supercomputer.

### Bill of Materials (DIY)

| Component | Recommendation | Price | Link |
|-----------|----------------|-------|------|
| GPUs | 8x H100 SXM5 (HGX baseboard + 4x NVSwitch) | $280-320K | [NVIDIA HGX H100](https://www.nvidia.com/en-us/data-center/hgx/) |
| CPUs | Dual AMD EPYC 9654 (96c each) or Intel Xeon 8490H | $18-25K | [AMD EPYC](https://www.amd.com/en/products/processors/server/epyc/4th-generation-9004-and-8004-series.html) |
| RAM | 2TB DDR5 ECC | $10,000 | |
| Storage | 16TB NVMe RAID | $3,000 | |
| PSU | 6.4kW (6x redundant) | $4,000 | |
| Networking | 8x ConnectX-7 (1 per GPU) | $12,000 | |
| InfiniBand | NDR switch + cables | $15,000 | |
| Cooling | **Liquid cooling (CDU + manifolds)** | $20-30K | |
| **Total** | | **$370-430K** | |

### DGX H100 (Turnkey Alternative)

| Spec | DGX H100 |
|------|----------|
| GPUs | 8x H100 SXM5 80GB |
| GPU Memory | 640GB total |
| NVLink | 900 GB/s per GPU, NVSwitch fabric |
| CPUs | Dual Intel Xeon 8480+ (56c each) |
| System RAM | 2TB DDR5 |
| Storage | 30TB NVMe |
| Networking | 8x ConnectX-7 400Gb |
| Power | 10.2kW max |
| Price | **$300,000 - $450,000** |
| Link | [NVIDIA DGX H100](https://www.nvidia.com/en-us/data-center/dgx-h100/) |

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
| Llama 3.1 70B | FP16 | 140GB | ~200 | ~3,300 |
| Llama 3.1 70B | FP8 | 70GB | ~250 | ~4,000 |
| Llama 3.1 405B | FP16 | 810GB | Won't fit | - |
| Llama 3.1 405B | FP8 | 405GB | ~40 | ~500 |
| Llama 3.1 405B + 128K | FP8 | 550GB | ~30 | ~350 |
| DeepSeek R1 671B | FP8 | 670GB | Won't fit fully | - |
| DeepSeek R1 671B | Q4 | 335GB | ~20 | ~200 |

**Sweet spot:** 405B at FP8 with 32-64K context
**Maximum:** 671B at Q4 or 405B at FP8 with 128K context

### Physical Specifications

| Spec | Value |
|------|-------|
| Form factor | 8U rackmount |
| Dimensions | 19" W × 14" H × 35.3" D |
| Weight | 275 lbs (125 kg) |
| Noise | 75-85 dB (jet engine) |
| Heat output | 22,000-35,000 BTU/hr |
| Rack depth | Requires 48"+ deep rack |
| Cooling | **Liquid cooling mandatory** |
| **Location** | Datacenter with liquid cooling infrastructure |

### Power Requirements

| State | Power Draw | Circuit |
|-------|------------|---------|
| Idle | 1,500W | Multiple 30A/240V |
| Typical | 7,000W | Multiple 30A/240V |
| Peak | 10,000-10,200W | 3x 30A/240V or 2x 50A/240V |

**Infrastructure requirements:**
- 3-phase power recommended
- Industrial PDU with C19/C20
- 15kVA+ UPS or generator backup
- Coolant Distribution Unit (CDU) rated for 12kW+

### Where to Buy

**Turnkey:**
- [NVIDIA DGX H100](https://www.nvidia.com/en-us/data-center/dgx-h100/) - Direct from NVIDIA
- [Dell PowerEdge XE9680](https://www.dell.com/en-us/shop/servers-storage-and-networking/poweredge-xe9680-rack-server/spd/poweredge-xe9680/pe_xe9680_16979_vi_vp) - Dell enterprise
- [HPE ProLiant DL380a Gen11](https://www.hpe.com/us/en/compute/hpc/supercomputing.html) - HPE enterprise

**System Integrators:**
- [Supermicro AS-8125GS-TNHR](https://www.supermicro.com/en/products/system/gpu/8u/as-8125gs-tnhr) (AMD EPYC)
- [Supermicro SYS-821GE-TNHR](https://www.supermicro.com/en/products/system/gpu/8u/sys-821ge-tnhr) (Intel Xeon)
- [Lambda Labs](https://lambdalabs.com/) - Cloud only now, but reference designs
- [CoreWeave](https://www.coreweave.com/) - Cloud H100 clusters

---

## ⚡ Power Infrastructure

*Sources: [NVIDIA DGX User Guide](https://docs.nvidia.com/dgx/dgxh100-user-guide/introduction-to-dgxh100.html), [TRG Datacenters](https://www.trgdatacenters.com/resource/nvidia-h100-power-consumption/)*

### Power by Configuration

| Config | GPU Power | System Total | Heat (BTU/hr) | Circuit |
|--------|-----------|--------------|---------------|---------|
| 1x PCIe | 350W | 600-800W | 2,000-2,700 | 15A/120V |
| 2x PCIe | 700W | 1,200-1,600W | 4,000-5,500 | 20A/120V |
| 2x SXM | 1,400W | 2,000-2,500W | 6,800-8,500 | 30A/240V |
| 4x PCIe | 1,400W | 2,200-2,800W | 7,500-9,500 | 30A/240V |
| 4x SXM | 2,800W | 4,000-5,000W | 14,000-17,000 | 40A/240V |
| 8x SXM | 5,600W | 8,000-10,200W | 27,000-35,000 | 3x 30A/240V |

### UPS Sizing

| System Power | UPS VA Rating | Runtime (15 min) |
|--------------|---------------|------------------|
| 800W | 1,200 VA | Standard |
| 1,600W | 2,400 VA | Rackmount |
| 2,800W | 4,200 VA | Rackmount |
| 5,000W | 7,500 VA | 3-phase |
| 10,000W | 15,000 VA | 3-phase industrial |

*VA = Watts × 1.5 for typical server power factor*

### PDU Requirements

| Config | Connector Type | PDU Rating |
|--------|---------------|------------|
| 1-2x PCIe | C13/C14 | 20A single phase |
| 4x PCIe | C19/C20 | 30A single phase |
| 4x SXM | C19/C20 | 40A single phase |
| 8x SXM | C19/C20 | 60A 3-phase |

---

## 🌡️ Cooling Requirements

*Sources: [Supermicro Liquid Cooling](https://www.supermicro.com/en/pressreleases/supermicro-launches-industrys-first-nvidia-hgx-h100-8-and-4-gpu-h100-servers-liquid), [Introl Cooling Guide](https://www.introl.io/blog/liquid-cooling-gpu-data-centers-50kw-thermal-limits-guide)*

### Cooling Method by Configuration

| Config | Cooling | Heat Load | Notes |
|--------|---------|-----------|-------|
| 1x PCIe | Air | 600-800W | Standard case fans |
| 2x PCIe | Air | 1.2-1.6kW | High-airflow case |
| 4x PCIe | Air (aggressive) | 2.2-2.8kW | Hot/cold aisle required |
| 4x SXM | **Liquid** | 4-5kW | Direct-to-chip cooling |
| 8x SXM | **Liquid** | 8-10kW | CDU infrastructure |

### Air Cooling Limits

- Practical max per rack: 20-30kW with excellent airflow
- Above 30kW: Requires 7,850+ CFM (hurricane-force)
- DGX H100 at 10kW: Maximum 3-4 per standard rack

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
| 1x PCIe | Tower/4U | 45-60 lbs | 26" |
| 2x PCIe | 4U | 65-80 lbs | 28" |
| 4x PCIe | 4U-5U | 85-100 lbs | 30" |
| 4x SXM | 5U | 120-150 lbs | 32" |
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
| 1x PCIe | 45-55 dB | Loud conversation |
| 2x PCIe | 55-65 dB | Vacuum cleaner |
| 4x PCIe | 65-75 dB | Lawn mower |
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
| 1x PCIe | 600W avg | $52 | $1,870 |
| 2x PCIe | 1.2kW avg | $104 | $3,740 |
| 4x PCIe | 2.2kW avg | $190 | $6,850 |
| 4x SXM | 4kW avg | $346 | $12,460 |
| 8x SXM | 8kW avg | $691 | $24,900 |

### Cooling Costs

Rule of thumb: Cooling = 30-50% of compute power cost

| Config | Cooling Power | Monthly | 3-Year |
|--------|---------------|---------|--------|
| 1x PCIe (air) | 200W | $17 | $620 |
| 4x PCIe (air) | 800W | $69 | $2,500 |
| 8x SXM (liquid) | 1.5kW | $130 | $4,680 |

### Total 3-Year TCO

| Config | Hardware | Electricity | Cooling | Support | **Total** |
|--------|----------|-------------|---------|---------|-----------|
| 1x PCIe | $40K | $1.9K | $0.6K | $3K | **$45.5K** |
| 2x NVL | $85K | $3.7K | $1.2K | $5K | **$95K** |
| 4x PCIe | $165K | $6.9K | $2.5K | $10K | **$184K** |
| 4x SXM | $225K | $12.5K | $4K | $15K | **$256K** |
| 8x SXM | $400K | $24.9K | $4.7K | $25K | **$455K** |

### Cloud Break-Even

| Provider | $/hr/H100 | Break-even vs Tier 2 | Break-even vs Tier 5 |
|----------|-----------|----------------------|----------------------|
| Lambda | $1.89 | 22 months | 26 months |
| CoreWeave | $2.50 | 17 months | 20 months |
| AWS | $3.50 | 12 months | 14 months |

**Rule of thumb:** Buy if >60% utilization for 18+ months

---

## 🔗 Related Concepts

**GPUs:**
- [[H100]] - The GPU itself (specs, architecture)
- [[A100]] - Previous generation datacenter GPU
- [[H200]] - H100 successor with 141GB HBM3e
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
- [[LLM Inference Hardware]] - GPU comparison overview
- [[LLM Under Your Floorboards]] - Consumer GPU builds
- [[Off-Grid LLM Power Systems]] - Solar/battery for these systems
- [[Quantization]] - Reducing VRAM requirements
- [[vLLM]] - Inference engine
- [[TensorRT-LLM]] - NVIDIA's inference engine

---

## 📚 External Resources

### Official Documentation
- [NVIDIA H100 Product Page](https://www.nvidia.com/en-us/data-center/h100/)
- [NVIDIA DGX H100](https://www.nvidia.com/en-us/data-center/dgx-h100/)
- [DGX H100 User Guide](https://docs.nvidia.com/dgx/dgxh100-user-guide/)
- [TensorRT-LLM Performance Blog](https://developer.nvidia.com/blog/achieving-top-inference-performance-with-the-nvidia-h100-tensor-core-gpu-and-nvidia-tensorrt-llm/)

### Platform Documentation
- [NVIDIA HGX Platform](https://www.nvidia.com/en-us/data-center/hgx/)
- [HGX H100 Technical Blog](https://developer.nvidia.com/blog/introducing-nvidia-hgx-h100-an-accelerated-server-platform-for-ai-and-high-performance-computing/)
- [NVIDIA MGX Platform](https://www.nvidia.com/en-us/data-center/products/mgx/)
- [MGX Technical Blog](https://developer.nvidia.com/blog/building-the-modular-foundation-for-ai-factories-with-nvidia-mgx/)
- [DGX vs HGX Comparison (ServeTheHome)](https://www.servethehome.com/nvidia-dgx-versus-nvidia-hgx-what-is-the-difference/)
- [Platform Comparison Guide](https://www.server-parts.eu/post/nvidia-ai-platform-dgx-hgx-egx-agx-comparison)

### CPU Resources
- [AMD EPYC 9004 Series](https://www.amd.com/en/products/processors/server/epyc/4th-generation-9004-and-8004-series.html)
- [Intel Xeon Scalable](https://www.intel.com/content/www/us/en/products/details/processors/xeon/scalable.html)

### System Vendors
- [Supermicro GPU Servers](https://www.supermicro.com/en/accelerators/nvidia/hopper-ada-lovelace)
- [Supermicro Store](https://store.supermicro.com/)
- [Supermicro H13 Motherboards](https://www.supermicro.com/en/products/motherboards?pro=H13)
- [Thinkmate GPU Servers](https://www.thinkmate.com/systems/servers/gpx)
- [Silicon Mechanics](https://www.siliconmechanics.com/)
- [BIZON Workstations](https://bizon-tech.com/)
- [Exxact GPU Servers](https://www.exxactcorp.com/gpu-servers)
- [Puget Systems](https://www.pugetsystems.com/)

### Comparisons and Benchmarks
- [H100 PCIe vs SXM](https://www.hyperstack.cloud/technical-resources/performance-benchmarks/comparing-nvidia-h100-pcie-vs-sxm-performance-use-cases-and-more)
- [VRAM Requirements Guide](https://www.hyperstack.cloud/blog/case-study/how-much-vram-do-you-need-for-llms)
- [Supermicro Liquid Cooling](https://www.supermicro.com/en/pressreleases/supermicro-launches-industrys-first-nvidia-hgx-h100-8-and-4-gpu-h100-servers-liquid)

### Used Market
- [ServerMonkey](https://www.servermonkey.com/)
- [eBay Enterprise Servers](https://www.ebay.com/b/Enterprise-Servers/11211/)

<!--
================================================================================
TEMPLATE NOTES FOR FUTURE GPU BUILD DOCUMENTS (A100, H200, B200, etc.)
================================================================================

This document serves as a template. When creating "A100 Builds" or "B200 Builds":

SECTION REUSABILITY:
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

FIND-REPLACE CHECKLIST:
1. [ ] Replace "H100" with new GPU name throughout
2. [ ] Update all VRAM values (80GB → whatever)
3. [ ] Update all TDP values
4. [ ] Update all bandwidth values (memory BW, NVLink BW)
5. [ ] Update all TFLOPS values
6. [ ] Update pricing throughout
7. [ ] Update generation table in "Where [GPU] Fits"
8. [ ] Rebuild comparison table vs other generations
9. [ ] Update NVLink/NVSwitch generation numbers
10. [ ] Update software version requirements
11. [ ] Recalculate Model-to-Hardware VRAM requirements
12. [ ] Rebuild each Tier's BOM, performance tables
13. [ ] Update PCIe lane allocations if CPU recommendations change
14. [ ] Recalculate TCO section
15. [ ] Update cloud break-even calculations
16. [ ] Verify and update all external links
17. [ ] Update "Last verified" date

GPU-SPECIFIC NOTES:
- A100: No NVL variant exists. Tiers may be 4 instead of 5.
- H200: Same compute as H100, just more VRAM. Tiers similar.
- B200: Different architecture (Blackwell). GB200 superchip changes tier structure.
- B100: Lower TDP variant of Blackwell. May have different tier mapping.

NVLink/NVSwitch GENERATIONS:
| GPU  | NVLink Gen | NVSwitch Gen | Bandwidth |
|------|------------|--------------|-----------|
| A100 | 3rd        | 2nd          | 600 GB/s  |
| H100 | 4th        | 3rd          | 900 GB/s  |
| H200 | 4th        | 3rd          | 900 GB/s  |
| B200 | 5th        | 4th          | 1800 GB/s |

PRICING NOTES:
- GPU prices change rapidly. Always note verification date.
- Used market prices can be 40-60% of new for previous gen.
- HGX baseboard pricing is separate from system pricing.
- DGX includes ~$50-100K premium over equivalent HGX build.

================================================================================
-->
