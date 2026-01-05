# B100 Builds

A comprehensive guide to building systems around NVIDIA's B100 Tensor Core GPU—the power-efficient Blackwell variant designed as a drop-in upgrade for existing H100/H200 infrastructure. Covers variants, specifications, what models fit on different configurations, complete build specs, cooling, power, physical requirements, and pricing.

For consumer GPU builds see [[LLM Under Your Floorboards]]. For comparisons with other datacenter GPUs see [[LLM Inference Hardware]].

*Pricing last verified: January 2025. B100 has limited availability as most vendors focus on B200. Prices are estimates.*

---

## 📍 Where B100 Fits

The B100 is NVIDIA's power-efficient Blackwell GPU, designed as a drop-in replacement for H100/H200 in existing infrastructure. Same 700W TDP as H100, same memory as B200, but ~75-80% of B200's compute. Think of it as "Blackwell for existing datacenters."

| Generation | Architecture | Flagship GPU | VRAM | TDP | Release | Status |
|------------|--------------|--------------|------|-----|---------|--------|
| Hopper | GH100 | [[H100]] | 80GB HBM3 | 700W | 2022 | Mainstream |
| Hopper Refresh | GH100 | [[H200]] | 141GB HBM3e | 700W | 2024 | Premium Hopper |
| **Blackwell (efficient)** | **GB100** | **[[B100]]** | **192GB HBM3e** | **700W** | **2025** | **Drop-in Blackwell** |
| Blackwell (full) | GB200 | [[B200]] | 192GB HBM3e | 1000W | 2025 | Current flagship |
| Blackwell Ultra | GB300 | [[B300]] | 288GB HBM3e | 1200W | Late 2025 | Next refresh |

### When to Choose B100

| Scenario | Best Choice | Why |
|----------|-------------|-----|
| Existing H100/H200 infrastructure | **B100** | Drop-in compatible, same 700W TDP |
| Can't upgrade power/cooling | **B100** | Works in existing 700W-per-GPU systems |
| Budget-conscious Blackwell entry | **B100** | ~$30-35K vs B200's ~$45-55K |
| Maximum absolute performance | B200 | 25-30% more compute |
| New datacenter build | B200 | Build for 1000W from start |
| Availability is priority | H200 or H100 | B100 has limited vendor support |

### B100 vs B200 vs H200 vs H100

| Spec | H100 SXM | H200 SXM | B100 SXM | B200 SXM |
|------|----------|----------|----------|----------|
| Architecture | Hopper | Hopper | Blackwell | Blackwell |
| VRAM | 80GB HBM3 | 141GB HBM3e | 192GB HBM3e | 192GB HBM3e |
| Memory BW | 3.35 TB/s | 4.8 TB/s | 8.0 TB/s | 8.0 TB/s |
| FP8 Tensor | 3,958 TFLOPS | 3,958 TFLOPS | ~7,000 TFLOPS | ~9,000 TFLOPS |
| FP4 Tensor | N/A | N/A | ~14,000 TFLOPS | ~20,000 TFLOPS |
| **TDP** | **700W** | **700W** | **700W** | **1,000W** |
| NVLink | 900 GB/s | 900 GB/s | 1,800 GB/s | 1,800 GB/s |
| Drop-in for H100 | N/A | Yes | **Yes** | No (needs 1kW) |
| Est. Price | ~$25-35K | ~$35-45K | ~$30-35K | ~$45-55K |
| **LLM Inference** | 1x baseline | ~1.4x | **~2x** | ~2.5x |

**Bottom line:**
- **B100 = B200 memory + B200 bandwidth + 75% B200 compute + H100 power envelope**
- **Key advantage:** Slide out H100/H200 HGX, slide in B100 HGX—no infrastructure changes
- **Trade-off:** ~25% less compute than B200, but works in existing systems

---

## 🔧 Software Requirements

*Sources: [NVIDIA CUDA Toolkit](https://developer.nvidia.com/cuda-toolkit), [Exxact Blackwell Comparison](https://www.exxactcorp.com/blog/hpc/comparing-nvidia-tensor-core-gpus)*

B100 uses Blackwell architecture—same software requirements as B200.

### Minimum Versions

| Component | Minimum | Recommended | Notes |
|-----------|---------|-------------|-------|
| CUDA Toolkit | 12.4 | 12.6+ | FP4 requires CUDA 12.6 |
| NVIDIA Driver | 550.x | 560.x+ | Blackwell support |
| cuDNN | 9.0 | 9.x | Required for best performance |
| TensorRT | 10.0 | 10.3+ | FP4/FP8 optimizations |
| PyTorch | 2.3 | 2.4+ | Native Blackwell support |
| TensorFlow | 2.16 | 2.17+ | XLA compilation support |

### Framework Considerations

| Framework | B100 Support | FP4 Support | FP8 Support | Notes |
|-----------|--------------|-------------|-------------|-------|
| [[PyTorch]] | Native (2.3+) | Via Transformer Engine | Yes | Best flexibility |
| [[TensorFlow]] | Native (2.16+) | Limited | Yes | XLA required |
| [[vLLM]] | Excellent | Coming | Yes | Recommended for inference |
| [[TensorRT-LLM]] | Excellent | Yes | Yes | Best raw performance |

---

## 📋 Quick Reference

| Tier | GPUs | VRAM | Best Models | Tokens/s (70B) | Budget | Form Factor |
|------|------|------|-------------|----------------|--------|-------------|
| 1 | 4x B100 SXM | 768GB | 405B FP8, 300B FP16 | ~280 tok/s | $180-220K | 5U |
| 2 | 8x B100 SXM (HGX) | 1.5TB | 671B FP8, 500B FP16 | ~500 tok/s | $320-400K | 8U |
| 3 | 8x B100 (DGX-class) | 1.5TB | 671B FP8, 500B FP16 | ~520 tok/s | $400-480K | 8U |

**Note:** B100 is SXM-only. No PCIe variant exists. Entry point is 4-GPU HGX configuration.

---

## 🎯 B100 Variants

*Sources: [NVIDIA HGX Platform](https://www.nvidia.com/en-us/data-center/hgx/), [Northflank B100 vs B200](https://northflank.com/blog/b100-vs-b200), [Exxact Blackwell](https://www.exxactcorp.com/blog/hpc/comparing-nvidia-tensor-core-gpus)*

| Variant | VRAM | Memory BW | TDP | NVLink BW | Form Factor | H100 Compatible |
|---------|------|-----------|-----|-----------|-------------|-----------------|
| **B100 SXM** | 192GB HBM3e | 8,000 GB/s | 700W | 1,800 GB/s | SXM socket | **Yes (drop-in)** |

**Key differentiator:** The B100 is specifically designed to be a drop-in replacement for H100/H200 HGX baseboards. Same power envelope, same cooling requirements, same physical form factor.

### B100 vs B200 Decision Matrix

| Factor | Choose B100 | Choose B200 |
|--------|-------------|-------------|
| Existing H100/H200 system | ✅ Drop-in upgrade | ❌ Needs new infrastructure |
| Power infrastructure | ✅ 700W/GPU works | ❌ Needs 1000W/GPU |
| Cooling infrastructure | ✅ Existing cooling works | ❌ May need upgrade |
| Maximum performance | ❌ ~25% less than B200 | ✅ Full Blackwell perf |
| Capital investment | ✅ Lower cost | ❌ Higher cost |
| Time to deploy | ✅ Faster (drop-in) | ❌ Infrastructure work |

---

## 🖥️ NVIDIA Platform Overview

*Sources: [NVIDIA HGX Platform](https://www.nvidia.com/en-us/data-center/hgx/), [Exxact Blackwell Deployments](https://www.exxactcorp.com/blog/hpc/nvidia-blackwell-deployments-gb200-nvl72-dgx-hgx-b200-hgx-b100)*

| Platform | What It Is | B100 Support | Notes |
|----------|------------|--------------|-------|
| **[[HGX]] B100** | GPU baseboard | ✅ | Drop-in for HGX H100/H200 |
| **[[DGX]]** | Turnkey system | Limited | DGX focus is B200/B300 |
| **[[MGX]]** | Modular spec | ✅ | B100 compatible |

### HGX B100 Drop-In Compatibility

The HGX B100 is designed as a plug-and-play replacement for existing HGX H100 and HGX H200 systems:

| Aspect | HGX H100 | HGX H200 | HGX B100 |
|--------|----------|----------|----------|
| Per-GPU TDP | 700W | 700W | 700W |
| Total power (8-GPU) | 5,600W | 5,600W | 5,600W |
| Baseboard form factor | Standard | Standard | Standard |
| Cooling requirements | Same | Same | Same |
| **Upgrade path** | - | Swap baseboard | Swap baseboard |

**Upgrade procedure:** Slide out the Hopper HGX baseboard, slide in the Blackwell HGX B100 baseboard. No power, cooling, or chassis modifications required.

---

## 🔲 HGX B100 Platform Deep Dive

*Sources: [NVIDIA HGX Platform](https://www.nvidia.com/en-us/data-center/hgx/), [Symmatrix HGX B100](https://www.symmatrix.com/product/nvidia-hgx-b100/), [smicro.eu B100 Baseboard](https://smicro.eu/nvidia-umbriel-b100-baseboard-1-5tb-hbm3e-935-26287-0000-000-1)*

### HGX B100 Specifications

| Spec | HGX B100 8-GPU |
|------|----------------|
| Codename | Umbriel |
| GPUs | 8x B100 SXM 192GB |
| Total VRAM | 1,536GB (1.5TB) HBM3e |
| Total Memory BW | 64 TB/s aggregate |
| NVSwitch | 4th generation |
| NVLink per GPU | 1,800 GB/s |
| Total NVLink BW | 14.4 TB/s |
| AI Performance | 112 PFLOPS |
| FP4 (dense/sparse) | 56/112 PFLOPS |
| FP8 (dense/sparse) | 28/56 PFLOPS |
| TDP (GPUs only) | 5,600W |
| System power | 8-10 kW |
| **H100/H200 compatible** | **Yes** |

### HGX B100 Pricing (2025 Estimates)

| Configuration | Baseboard Only | With Typical System | Notes |
|---------------|----------------|---------------------|-------|
| HGX B100 8-GPU | $260-300K | $400-480K | Drop-in for existing systems |

*Prices estimated. B100 has limited market availability compared to B200.*

### Where to Buy HGX B100

**NVIDIA Partners:**
- [Supermicro](https://www.supermicro.com/en/accelerators/nvidia) - HGX B100 systems
- [Symmatrix](https://www.symmatrix.com/product/nvidia-hgx-b100/) - HGX B100 8-GPU

**Resellers:**
- [smicro.eu](https://smicro.eu/nvidia-umbriel-b100-baseboard-1-5tb-hbm3e-935-26287-0000-000-1) - B100 baseboard (Umbriel)
- [Viperatech](https://viperatech.com/product/nvidia-b100-blackwell-ai-gpu/) - B100 GPUs

**Cloud Providers:**
- [CUDO Compute](https://www.cudocompute.com/) - B100 available
- Most major clouds skipped B100 for B200

**Availability note:** B100 has limited vendor support. Most cloud providers and system integrators focus on B200. If you need Blackwell and can support 1000W/GPU, B200 is more widely available.

---

## 📊 Model-to-Hardware Mapping

### VRAM Requirements

| Model | FP16 | FP8 | FP4 | Notes |
|-------|------|-----|-----|-------|
| Llama 3.1 8B | 16GB | 8GB | 4GB | Single GPU easy |
| Qwen2.5 32B | 64GB | 32GB | 16GB | Single B100 |
| Llama 3.1 70B | 140GB | 70GB | 35GB | Single B100 |
| Mixtral 8x22B | 176GB | 88GB | 44GB | Single B100 |
| Llama 3.1 405B | 810GB | 405GB | 200GB | 4-8x B100 |
| DeepSeek R1 671B | 1.3TB | 670GB | 335GB | 8x B100 |

### Minimum B100 Count by Model

| Model | FP16 | FP8 | FP4 | Optimal Config |
|-------|------|-----|-----|----------------|
| 70B | 1 | 1 | 1 | Single B100 |
| 140B | 2 | 1 | 1 | 4-GPU HGX |
| 200B | 2 | 1 | 1 | 4-GPU HGX |
| 405B | 8 | 4 | 2 | 8-GPU HGX |
| 671B | 8+ | 6 | 3 | 8-GPU HGX |

---

## 🔧 Tier 1: Quad B100 SXM (~$180-220K)

### Overview
Entry point for B100. Four GPUs on 4-GPU HGX baseboard or half-populated 8-GPU system. 768GB HBM3e runs 405B at FP8 comfortably.

### Specifications

| Spec | 4x B100 System |
|------|----------------|
| GPUs | 4x B100 SXM 192GB |
| GPU Memory | 768GB HBM3e |
| AI Performance | 56 PFLOPS |
| TDP (GPUs) | 2,800W |
| System Power | 4-5 kW |
| NVLink | 1,800 GB/s per GPU |

### Model Performance

| Model | Precision | VRAM Used | Tok/s (batch 1) | Tok/s (batch 32) |
|-------|-----------|-----------|-----------------|------------------|
| Llama 3.1 70B | FP16 | 140GB | ~220 | ~3,200 |
| Llama 3.1 70B | FP8 | 70GB | ~280 | ~4,000 |
| Mixtral 8x22B | FP16 | 176GB | ~180 | ~2,600 |
| Llama 3.1 405B | FP8 | 405GB | ~65 | ~800 |

### Power Requirements

| State | Power Draw | Circuit |
|-------|------------|---------|
| Idle | 800W | 30A/240V |
| Typical | 3,500W | 30A/240V |
| Peak | 4,500W | 40A/240V |

---

## 🔧 Tier 2: 8x B100 SXM HGX (~$320-400K)

### Overview
Full HGX B100 baseboard with 8 GPUs. 1.5TB HBM3e runs 671B at FP8. Drop-in replacement for existing HGX H100/H200 systems—the primary B100 use case.

### HGX B100 Specifications

| Spec | HGX B100 8-GPU |
|------|----------------|
| GPUs | 8x B100 SXM 192GB |
| GPU Memory | 1,536GB (1.5TB) HBM3e |
| Memory Bandwidth | 64 TB/s aggregate |
| AI Performance | 112 PFLOPS |
| NVLink | 14.4 TB/s total |
| TDP (GPUs) | 5,600W |
| System Power | 8-10 kW |

### Bill of Materials (Upgrade Existing)

If upgrading existing HGX H100/H200 system:

| Component | Recommendation | Price |
|-----------|----------------|-------|
| HGX B100 baseboard | 8x B100 SXM | $260-300K |
| Installation labor | Baseboard swap | $5-10K |
| Driver/firmware updates | - | Included |
| **Total upgrade cost** | | **$265-310K** |

Existing infrastructure reused: CPUs, RAM, storage, networking, chassis, cooling, power

### Bill of Materials (New Build)

| Component | Recommendation | Price |
|-----------|----------------|-------|
| GPUs | 8x B100 SXM (HGX baseboard) | $260-300K |
| CPUs | Dual Intel Xeon 8480+ or AMD EPYC 9654 | $18-22K |
| RAM | 2TB DDR5 ECC | $10,000 |
| Storage | 16TB NVMe RAID | $3,000 |
| PSU | 10kW redundant | $5,000 |
| Networking | 8x ConnectX-7 400Gb | $16,000 |
| Cooling | Air or Liquid | $5-15K |
| Chassis | 8U HGX-compatible | $3,000 |
| **Total new build** | | **$320-380K** |

### Model Performance

| Model | Precision | VRAM Used | Tok/s (batch 1) | Tok/s (batch 64) |
|-------|-----------|-----------|-----------------|------------------|
| Llama 3.1 70B | FP16 | 140GB | ~420 | ~6,500 |
| Llama 3.1 70B | FP8 | 70GB | ~500 | ~7,800 |
| Llama 3.1 405B | FP16 | 810GB | ~70 | ~900 |
| Llama 3.1 405B | FP8 | 405GB | ~110 | ~1,400 |
| DeepSeek R1 671B | FP8 | 670GB | ~70 | ~900 |

**Comparison to H100:** ~1.8x faster inference at same power

### Physical Specifications

| Spec | Value |
|------|-------|
| Form factor | 8U rackmount |
| Weight | 280-320 lbs |
| Noise | 75-85 dB |
| Heat output | 27,000-34,000 BTU/hr |
| Cooling | Air or Liquid (existing H100 cooling works) |
| **Location** | Datacenter |

### Power Requirements

**Same as H100/H200 8-GPU systems:**

| State | Power Draw | Circuit |
|-------|------------|---------|
| Idle | 1,800W | 3-phase |
| Typical | 7,500W | 3-phase |
| Peak | 9,000-10,000W | 3x 30A/240V |

---

## 🔧 Tier 3: 8x B100 DGX-Class (~$400-480K)

### Overview
Fully configured system with B100 GPUs, optimized software stack, and vendor support. Similar to DGX but with B100 instead of B200.

**Note:** NVIDIA's official DGX lineup focuses on B200 and B300. "DGX-class" B100 systems are available from OEMs like Supermicro.

### Specifications

| Spec | 8x B100 DGX-Class |
|------|-------------------|
| GPUs | 8x B100 SXM 192GB |
| GPU Memory | 1,536GB (1.5TB) HBM3e |
| AI Performance | 112 PFLOPS |
| CPUs | Dual Intel Xeon 8570 or AMD EPYC |
| System RAM | 2TB DDR5 |
| Storage | 30TB NVMe |
| Networking | 8x ConnectX-7 400Gb |
| Power | 10 kW max |
| **Price** | $400-480K |

### Performance vs DGX H100

| Metric | DGX H100 | B100 DGX-Class |
|--------|----------|----------------|
| GPU Memory | 640GB | 1,536GB |
| Memory BW | 26.4 TB/s | 64 TB/s |
| AI Perf (FP8) | 32 PFLOPS | 56 PFLOPS |
| Power | 10.2 kW | 10 kW |
| **Performance uplift** | 1x | **~1.8x** |

---

## ⚡ Power Infrastructure

### Power by Configuration

| Config | GPU Power | System Total | Heat (BTU/hr) | Circuit |
|--------|-----------|--------------|---------------|---------|
| 4x B100 | 2,800W | 4,000-4,500W | 13,600-15,400 | 40A/240V |
| 8x B100 | 5,600W | 8,500-10,000W | 29,000-34,000 | 3-phase |

**Key advantage:** Same power requirements as H100/H200—existing infrastructure works.

### UPS Sizing

| System Power | UPS VA Rating | Notes |
|--------------|---------------|-------|
| 4,500W | 6,750 VA | 4-GPU system |
| 10,000W | 15,000 VA | 8-GPU system, 3-phase |

---

## 🌡️ Cooling Requirements

### Same as H100/H200

| Config | Cooling | Heat Load | Notes |
|--------|---------|-----------|-------|
| 4x B100 | Air or Liquid | 4-4.5kW | Same as 4x H100 |
| 8x B100 | Air or Liquid | 8.5-10kW | Same as 8x H100 |

**Key advantage:** Existing H100/H200 cooling infrastructure works without modification.

---

## 💰 Total Cost of Ownership (3-Year)

### Upgrade Scenario (Existing H100 → B100)

| Cost Category | Amount |
|---------------|--------|
| HGX B100 baseboard | $280K |
| Installation | $8K |
| Electricity (incremental) | $0 (same power) |
| Cooling (incremental) | $0 (same cooling) |
| **Total upgrade** | **$288K** |
| **Performance gain** | ~1.8x |

### New Build 3-Year TCO

| Config | Hardware | Electricity | Cooling | Support | **Total** |
|--------|----------|-------------|---------|---------|-----------|
| 4x B100 | $200K | $11.8K | $4K | $12K | **$228K** |
| 8x B100 | $350K | $26.3K | $8K | $22K | **$406K** |

### Cloud Pricing (Limited Availability)

| Provider | $/hr/B100 | Notes |
|----------|-----------|-------|
| CUDO Compute | ~$4-5/hr | Limited availability |
| Most clouds | N/A | Skipped B100 for B200 |

---

## 🚨 Availability and Market Position

*Sources: [Modal H100 vs B100](https://modal.com/blog/h100-and-h200-vs-b100-and-b200), [Northflank B100 vs B200](https://northflank.com/blog/b100-vs-b200)*

### Market Reality

The B100 occupies a niche position:

| Factor | Reality |
|--------|---------|
| **Cloud availability** | Most providers skipped B100, went straight to B200 |
| **OEM support** | Limited compared to B200 |
| **Target market** | Existing H100/H200 owners wanting drop-in upgrade |
| **New builds** | B200 generally preferred if infrastructure supports it |

### When B100 Makes Sense

1. **Existing HGX H100/H200 infrastructure** - Slide out, slide in, done
2. **Power-constrained datacenter** - Can't add 300W per GPU
3. **Cooling-constrained facility** - Existing cooling works
4. **Time-to-deploy critical** - No infrastructure modifications
5. **Budget optimization** - Get Blackwell at lower cost

### When to Skip B100

1. **New datacenter build** - Build for B200's 1000W from start
2. **Maximum performance needed** - B200 is 25% faster
3. **Cloud deployment** - B200 more available
4. **Limited vendor support acceptable** - B200 has more options

---

## 🔗 Related Concepts

**GPUs:**
- [[B100]] - The GPU itself (specs, architecture)
- [[B200]] - Full-power Blackwell (1000W)
- [[H100]] - Hopper predecessor (drop-in compatible)
- [[H200]] - Hopper refresh (drop-in compatible)

**NVIDIA Platforms:**
- [[HGX]] - GPU baseboard (B100 HGX drop-in for H100 HGX)
- [[DGX]] - Turnkey systems (focus on B200/B300)
- [[NVLink]] - GPU interconnect (5th gen)
- [[NVSwitch]] - Multi-GPU fabric (4th gen)

**Related Guides:**
- [[B200 Builds]] - Full-power Blackwell builds
- [[H100 Builds]] - Compatible predecessor
- [[H200 Builds]] - Compatible predecessor
- [[LLM Inference Hardware]] - GPU comparison

---

## 📚 External Resources

### Official Documentation
- [NVIDIA HGX Platform](https://www.nvidia.com/en-us/data-center/hgx/)
- [NVIDIA Blackwell Architecture](https://www.nvidia.com/en-us/data-center/technologies/blackwell-architecture/)

### Comparisons
- [B100 vs B200 (Northflank)](https://northflank.com/blog/b100-vs-b200)
- [B100 vs H100 (Northflank)](https://northflank.com/blog/b100-vs-h100)
- [Blackwell vs Hopper (Exxact)](https://www.exxactcorp.com/blog/hpc/comparing-nvidia-tensor-core-gpus)
- [H100 vs H200 vs B100 vs B200 (Modal)](https://modal.com/blog/h100-and-h200-vs-b100-and-b200)

### Vendors
- [Supermicro Blackwell](https://www.supermicro.com/en/accelerators/nvidia)
- [Symmatrix HGX B100](https://www.symmatrix.com/product/nvidia-hgx-b100/)
- [Viperatech B100](https://viperatech.com/product/nvidia-b100-blackwell-ai-gpu/)

<!--
================================================================================
TEMPLATE NOTES
================================================================================

B100-SPECIFIC NOTES:
- SXM-only (no PCIe variant)
- 700W TDP (same as H100/H200) - KEY DIFFERENTIATOR
- Drop-in compatible with HGX H100/H200
- Limited vendor/cloud support vs B200
- ~75-80% of B200 compute performance
- Same memory (192GB) and bandwidth (8 TB/s) as B200
- Target market: existing H100/H200 datacenters

TIER STRUCTURE:
- Fewer tiers than other GPUs (limited market)
- No Tier for single GPU (SXM only)
- Focus on HGX drop-in upgrade scenario

================================================================================
-->
