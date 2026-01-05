# Off-Grid LLM Power Systems

A comprehensive guide to powering local LLM infrastructure with solar, batteries, and backup generators. The goal: run GPU-heavy systems 80-90% of the time on renewable energy, with enough storage for overnight operation and multi-day cloudy periods. This covers sizing calculations, battery chemistry, seasonal considerations, and specific configurations for each LLM hardware tier.

For hardware power requirements see [[LLM Inference Hardware]]. For the broader context on local LLMs see [[LLM Under Your Floorboards]].

---

## 🎯 Design Goals

| Requirement | Target |
|-------------|--------|
| Renewable uptime | 80-90% annually |
| Summer self-sufficiency | Full day + night operation |
| Winter capability | Partial (generator supplement) |
| Battery longevity | 10+ years (proper cycling) |
| Autonomy (no sun) | 2-3 days minimum |
| Generator runtime | <20% of hours annually |

The fundamental challenge: LLM hardware draws constant, high power 24/7, but solar only produces 4-6 hours of peak equivalent per day. You need massive battery banks to bridge the gap.

---

## 🌍 Location Assumptions

**Upper Midwest / Great Lakes Region (~42-45°N latitude)**

| Season | Day Length | Peak Sun Hours | Cloud Factor | Effective PSH |
|--------|------------|----------------|--------------|---------------|
| June (best) | 15.5 hrs | 6.5 hrs | 0.75 | ~4.9 hrs |
| March/Sept | 12 hrs | 4.5 hrs | 0.70 | ~3.2 hrs |
| December (worst) | 9 hrs | 2.5 hrs | 0.55 | ~1.4 hrs |
| **Annual Average** | 12 hrs | 4.2 hrs | 0.68 | ~2.9 hrs |

**Peak Sun Hours (PSH)** = equivalent hours of 1000 W/m² irradiance. A 1kW panel produces 1kWh per PSH.

**Cloud factor** is critical here - the Great Lakes region is notoriously cloudy:
- Annual cloud cover: ~65%
- Winter cloud cover: ~75-80%
- Summer cloud cover: ~55-60%
- Lake effect adds extra winter cloudiness

This is one of the harder US regions for solar self-sufficiency.

---

## 📋 Operational Profiles

Not everyone needs 24/7 operation. Running only during daylight hours—or accepting reduced winter availability—dramatically cuts costs.

### Profile Definitions

| Profile | Summer (May-Aug) | Spring/Fall | Winter (Nov-Feb) | Use Case |
|---------|------------------|-------------|------------------|----------|
| **A: 24/7 Year-Round** | 24 hrs/day | 24 hrs/day | 24 hrs/day + generator | Production server, always-on |
| **B: 24/7 Summer, Reduced Winter** | 24 hrs/day | 16 hrs/day | 8 hrs/day | Seasonal heavy use |
| **C: Daylight Only Year-Round** | 15 hrs/day | 12 hrs/day | 9 hrs/day | Workday operation |
| **D: Daylight Summer, Minimal Winter** | 15 hrs/day | 10 hrs/day | 4 hrs/day | Fair-weather computing |
| **E: Peak Hours Only** | 8 hrs/day | 6 hrs/day | 4 hrs/day | Batch jobs, training runs |

### Why Profiles Matter

The overnight battery requirement is the cost killer:

| Profile | June Night Bridge | Dec Night Bridge | Battery Multiplier |
|---------|-------------------|------------------|-------------------|
| 24/7 (A) | 8.5 hrs × power | 15 hrs × power | 1.0x (baseline) |
| 24/7 summer only (B) | 8.5 hrs × power | 0 hrs (off) | 0.6x |
| Daylight only (C) | ~2 hrs buffer | ~2 hrs buffer | 0.15x |
| Daylight + minimal winter (D) | ~2 hrs buffer | ~1 hr buffer | 0.12x |
| Peak hours (E) | ~1 hr buffer | ~1 hr buffer | 0.08x |

**Translation:** Daylight-only operation needs 85% less battery than 24/7. That's the difference between $250K and $35K in batteries.

---

## 🔋 Profile Calculations (Per 1kW Load)

Reference table for scaling to any system:

### Profile A: 24/7 Year-Round

| Component | Calculation | Size per 1kW |
|-----------|-------------|--------------|
| Daily consumption | 1kW × 24h | 24 kWh |
| Solar array (summer sufficient) | 24 ÷ (4.9 × 0.77) | 6.4 kW |
| Battery (overnight + 2-day buffer) | (8.5 + 48) × 1kW ÷ 0.8 DoD | 70 kWh |
| Winter generator hours | ~2,000 hrs/season | Significant |

### Profile B: 24/7 Summer, Reduced Winter

| Component | Calculation | Size per 1kW |
|-----------|-------------|--------------|
| Summer daily | 1kW × 24h | 24 kWh |
| Winter daily | 1kW × 8h | 8 kWh |
| Solar array | 24 ÷ (4.9 × 0.77) | 6.4 kW |
| Battery (overnight summer) | 8.5 × 1kW ÷ 0.8 DoD | 10.6 kWh |
| Add cloudy buffer (+1 day) | +24 kWh ÷ 0.8 | +30 kWh |
| **Total battery** | | **~41 kWh** |

### Profile C: Daylight Only Year-Round

| Component | Calculation | Size per 1kW |
|-----------|-------------|--------------|
| Summer daily (15 hrs) | 1kW × 15h | 15 kWh |
| Winter daily (9 hrs) | 1kW × 9h | 9 kWh |
| Solar array (summer) | 15 ÷ (4.9 × 0.77) | 4.0 kW |
| Battery (2-hr buffer + clouds) | 2 × 1kW + 15 kWh buffer ÷ 0.8 | ~22 kWh |

### Profile D: Daylight Summer, Minimal Winter

| Component | Calculation | Size per 1kW |
|-----------|-------------|--------------|
| Summer daily (15 hrs) | 1kW × 15h | 15 kWh |
| Winter daily (4 hrs) | 1kW × 4h | 4 kWh |
| Solar array | 15 ÷ (4.9 × 0.77) | 4.0 kW |
| Battery (minimal buffer) | 2 × 1kW + 10 kWh ÷ 0.8 | ~15 kWh |

### Profile E: Peak Hours Only

| Component | Calculation | Size per 1kW |
|-----------|-------------|--------------|
| Summer daily (8 hrs) | 1kW × 8h | 8 kWh |
| Winter daily (4 hrs) | 1kW × 4h | 4 kWh |
| Solar array | 8 ÷ (4.9 × 0.77) | 2.1 kW |
| Battery (1-hr buffer) | 1 × 1kW + 8 kWh ÷ 0.8 | ~11 kWh |

---

## 📊 The Core Math

### Daily Energy Requirement

```
Daily kWh = System Power (kW) × 24 hours
```

### Solar Array Sizing (Summer Self-Sufficiency)

```
Array Size (kW) = Daily kWh ÷ (Effective PSH × System Efficiency)

Where:
- Effective PSH = Peak Sun Hours × Cloud Factor
- System Efficiency = ~0.75-0.80 (inverter, wiring, temperature, MPPT losses)
```

### Battery Sizing

```
Battery Capacity (kWh) = Daily kWh × Days of Autonomy ÷ Max Depth of Discharge

Where:
- Days of Autonomy = 2-3 days (consecutive cloudy days)
- Max DoD = 50% (lead-acid) or 80% (LiFePO4) for longevity
```

### Critical Insight: Summer vs Winter Gap

| Season | 10kW Load Daily Need | Effective PSH | Solar Generation (50kW array) |
|--------|---------------------|---------------|-------------------------------|
| **June** | 240 kWh | 4.9 hrs | ~185 kWh |
| **December** | 240 kWh | 1.4 hrs | ~53 kWh |

Even with aggressive summer oversizing, winter is impossible without:
- Massive battery banks (impractical)
- Grid tie (defeats "off-grid")
- Generator backup (realistic)

**The realistic approach:** Size for summer self-sufficiency, accept generator dependency in winter.

---

## 🔋 Battery Chemistry Deep Dive

### Lithium Iron Phosphate (LiFePO4) - Recommended

| Spec | Value | Notes |
|------|-------|-------|
| Cycle life | 3000-6000 cycles | At 80% DoD |
| Usable capacity | 80-90% | Safe deep discharge |
| Round-trip efficiency | 95-98% | Very efficient |
| Self-discharge | <3%/month | Negligible |
| Temperature range | -20°C to 60°C | Needs heating in winter |
| Calendar life | 10-15 years | |
| Cost | $400-600/kWh | Falling rapidly |
| Safety | Excellent | No thermal runaway |

**For LLM use:** LiFePO4 is the clear winner. High cycle life means daily deep cycling won't destroy the bank.

### Lead-Acid (AGM/Gel) - Budget Option

| Spec | Value | Notes |
|------|-------|-------|
| Cycle life | 500-1500 cycles | At 50% DoD |
| Usable capacity | 50% | Deeper kills lifespan |
| Round-trip efficiency | 80-85% | Significant losses |
| Self-discharge | 3-5%/month | |
| Calendar life | 3-7 years | |
| Cost | $150-250/kWh | Lower upfront |
| Safety | Good | |

**For LLM use:** The 50% DoD limit means you need 2x the capacity, and they'll need replacement in 3-5 years with daily cycling. False economy for high-demand systems.

### Comparison for 500 kWh Usable Storage

| Chemistry | Total Capacity Needed | Cost | Lifespan | 10-Year Cost |
|-----------|----------------------|------|----------|--------------|
| LiFePO4 | 625 kWh | ~$300K | 12+ years | ~$300K |
| Lead-Acid | 1000 kWh | ~$200K | 4 years | ~$500K |

LiFePO4 wins decisively for daily cycling applications.

---

## 🔄 Battery Cycling and Longevity

**The key to 10+ year battery life:** Don't cycle too deep, too often.

| Depth of Discharge | LiFePO4 Cycles | Years at 1 cycle/day |
|-------------------|----------------|----------------------|
| 100% | 2000 | 5.5 years |
| 80% | 4000 | 11 years |
| 70% | 5000 | 13.7 years |
| 50% | 7000+ | 19+ years |

**For LLM systems:** Target 70-80% DoD maximum for good balance of usable capacity and longevity.

### Practical Implications

For a system that needs 200 kWh overnight:
- At 80% DoD: Need 250 kWh battery bank
- At 70% DoD: Need 286 kWh battery bank
- At 50% DoD: Need 400 kWh battery bank

The extra 50-150 kWh of battery capacity to stay at 70% DoD adds ~$20-60K but saves replacing the whole bank in year 8 vs year 12.

---

## ⚡ Reference: Power Per 10kW Continuous Load

This is your baseline calculator. Scale linearly for your actual load.

### Solar Array Sizing (10kW continuous)

| Target | Calculation | Array Size |
|--------|-------------|------------|
| Summer self-sufficient | 240 kWh ÷ (4.9 PSH × 0.77 eff) | **64 kW** |
| Spring/Fall partial | 240 kWh ÷ (3.2 PSH × 0.77 eff) | 97 kW |
| Winter self-sufficient | 240 kWh ÷ (1.4 PSH × 0.77 eff) | 223 kW (impractical) |

**Recommendation for 10kW load: 60-70 kW solar array**

This provides:
- Full summer self-sufficiency
- ~70% self-sufficiency in spring/fall
- ~25% of winter needs (rest from generator)

### Battery Bank Sizing (10kW continuous)

| Component | Calculation | Capacity |
|-----------|-------------|----------|
| Overnight (June) | 10kW × 8.5 hrs darkness | 85 kWh |
| Overnight (Dec) | 10kW × 15 hrs darkness | 150 kWh |
| Cloudy day buffer (2 days) | 10kW × 48 hrs | 480 kWh |
| **Total usable needed** | Overnight + buffer | ~500-550 kWh |
| At 80% DoD | 550 ÷ 0.80 | **690 kWh total** |
| At 70% DoD | 550 ÷ 0.70 | **785 kWh total** |

**Recommendation for 10kW load: 700-800 kWh LiFePO4 battery bank**

### Cost Estimate (10kW continuous load)

| Component | Specification | Cost Range |
|-----------|---------------|------------|
| Solar panels | 65 kW @ $0.30-0.50/W | $20-32K |
| Mounting/racking | Ground or roof mount | $8-15K |
| Inverters | 15-20 kW (multiple) | $8-15K |
| Charge controllers | MPPT, 65kW capacity | $5-10K |
| Battery bank | 750 kWh LiFePO4 | $300-450K |
| BMS and integration | Battery management | $5-15K |
| Wiring, disconnects, panels | Balance of system | $10-20K |
| Installation labor | Electrician + solar | $15-30K |
| **Total** | | **$370-590K** |

The battery bank is 70-80% of system cost. This is the reality of true off-grid for high continuous loads.

---

## 🖥️ LLM System Configurations

### Single RTX 4090 System

| Spec | Value |
|------|-------|
| System power | 650W average |
| Daily consumption | 15.6 kWh |
| Annual consumption | 5,700 kWh |

| Component | Size | Cost |
|-----------|------|------|
| Solar array | 4.5 kW | $1,500-2,500 |
| Battery bank | 50 kWh LiFePO4 | $20-30K |
| Inverter | 2 kW | $1,000-2,000 |
| **Total system** | | **$25-40K** |

**Notes:** This is actually achievable for most homeowners. A PowerWall (13.5 kWh) won't cut it—you need ~4 of them or equivalent.

---

### Single RTX 5090 System

| Spec | Value |
|------|-------|
| System power | 825W average |
| Daily consumption | 19.8 kWh |
| Annual consumption | 7,200 kWh |

| Component | Size | Cost |
|-----------|------|------|
| Solar array | 5.5 kW | $1,800-3,000 |
| Battery bank | 65 kWh LiFePO4 | $26-39K |
| Inverter | 2.5 kW | $1,200-2,500 |
| **Total system** | | **$32-50K** |

---

### Dual RTX 4090 System

| Spec | Value |
|------|-------|
| System power | 1,300W average |
| Daily consumption | 31.2 kWh |
| Annual consumption | 11,400 kWh |

| Component | Size | Cost |
|-----------|------|------|
| Solar array | 9 kW | $3,000-5,000 |
| Battery bank | 100 kWh LiFePO4 | $40-60K |
| Inverter | 4 kW | $2,000-4,000 |
| **Total system** | | **$50-75K** |

---

### Dual RTX 5090 System

| Spec | Value |
|------|-------|
| System power | 1,650W average |
| Daily consumption | 39.6 kWh |
| Annual consumption | 14,450 kWh |

| Component | Size | Cost |
|-----------|------|------|
| Solar array | 11.5 kW | $3,800-6,000 |
| Battery bank | 130 kWh LiFePO4 | $52-78K |
| Inverter | 5 kW | $2,500-5,000 |
| **Total system** | | **$65-95K** |

---

### tinybox red v2 (4x RX 9070 XT)

| Spec | Value |
|------|-------|
| System power | 1,600W average |
| Daily consumption | 38.4 kWh |
| Annual consumption | 14,000 kWh |

| Component | Size | Cost |
|-----------|------|------|
| Solar array | 11 kW | $3,600-5,500 |
| Battery bank | 125 kWh LiFePO4 | $50-75K |
| Inverter | 5 kW | $2,500-5,000 |
| **Total system** | | **$65-95K** |

**Total cost with tinybox:** $75-105K (tinybox $10K + solar $65-95K)

---

### tinybox green v2 (4x RTX PRO 6000 Blackwell)

| Spec | Value |
|------|-------|
| System power | 3,250W average |
| Daily consumption | 78 kWh |
| Annual consumption | 28,500 kWh |

| Component | Size | Cost |
|-----------|------|------|
| Solar array | 22 kW | $7,000-11,000 |
| Battery bank | 250 kWh LiFePO4 | $100-150K |
| Inverter | 8 kW | $4,000-8,000 |
| Charge controllers | ~22kW MPPT | $3,000-6,000 |
| **Total system** | | **$120-185K** |

**Total cost with tinybox:** $170-235K (tinybox $50K + solar $120-185K)

---

### tinybox pro v2 (8x RTX 5090)

| Spec | Value |
|------|-------|
| System power | 6,000W average |
| Daily consumption | 144 kWh |
| Annual consumption | 52,600 kWh |

| Component | Size | Cost |
|-----------|------|------|
| Solar array | 40 kW | $13,000-20,000 |
| Battery bank | 460 kWh LiFePO4 | $185-275K |
| Inverter(s) | 15 kW | $8,000-15,000 |
| Charge controllers | ~40kW MPPT | $5,000-10,000 |
| Electrical infrastructure | 200A+ service | $5,000-15,000 |
| **Total system** | | **$220-350K** |

**Total cost with tinybox:** $270-400K (tinybox $50K + solar $220-350K)

---

### 8x H100 System (DGX-class)

| Spec | Value |
|------|-------|
| System power | 9,000W average |
| Daily consumption | 216 kWh |
| Annual consumption | 78,800 kWh |

| Component | Size | Cost |
|-----------|------|------|
| Solar array | 60 kW | $20,000-30,000 |
| Battery bank | 690 kWh LiFePO4 | $275-415K |
| Inverter(s) | 20+ kW | $12,000-25,000 |
| Charge controllers | ~60kW MPPT | $8,000-15,000 |
| Electrical infrastructure | 400A service | $15,000-30,000 |
| Building/enclosure | Climate control | $20,000-50,000 |
| **Total system** | | **$350-565K** |

**Total cost with DGX:** $650K-865K (DGX ~$300K + solar $350-565K)

At this scale, you're building a small power plant. Grid-tie with battery backup starts making more sense than pure off-grid.

---

## 🎯 Specific Models: What Hardware, What Solar?

Only hardware that runs each model **well**. No PCIe-split garbage. Each table shows the **lowest power option** that doesn't compromise speed.

---

### DeepSeek R1 671B (Best Open Reasoning Model)

**VRAM needed:** ~330 GB (Q4), ~670 GB (Q8), ~1.3 TB (FP16)

No single GPU fits this. Must use multi-GPU with NVLink.

| Hardware | VRAM | NVLink | Quant | Speed | System Power | Hardware Cost |
|----------|------|--------|-------|-------|--------------|---------------|
| 8x A100 80GB | 640 GB | 600 GB/s | Q8 | 18-25 t/s | **3.5 kW** | ~$120K used |
| 8x H100 SXM | 640 GB | 900 GB/s | Q8 | 25-35 t/s | 9 kW | ~$250K |
| 8x H200 | 1128 GB | 900 GB/s | FP16 | 30-40 t/s | 9 kW | ~$320K |
| DGX B200 | 1536 GB | NVSwitch | FP16 | 50-70 t/s | 14 kW | ~$450K |

🔋 **Lowest power:** 8x A100 80GB at 3.5 kW system power

**Solar for R1 (8x A100, 12 hrs/day summer):**
- Daily: 3.5 kW × 12 hrs = 42 kWh
- Solar array: 12 kW (~$4-6K)
- Battery: 55 kWh (~$22-33K)
- **Solar total: $35-55K**
- **Complete system: $155-175K**

---

### Llama 3.1 405B (Largest Open General Model)

**VRAM needed:** ~200 GB (Q4), ~400 GB (Q8), ~810 GB (FP16)

No single GPU fits. Need NVLink multi-GPU.

| Hardware | VRAM | NVLink | Quant | Speed | System Power | Hardware Cost |
|----------|------|--------|-------|-------|--------------|---------------|
| 4x A100 80GB | 320 GB | 600 GB/s | Q4 | 15-22 t/s | **2 kW** | ~$60K used |
| tinybox green v2 (4x PRO 6000) | 384 GB | Yes | Q8 | 25-35 t/s | 3.25 kW | $50K |
| 4x H100 SXM | 320 GB | 900 GB/s | Q4 | 30-40 t/s | 4.5 kW | ~$140K |
| 8x H100 SXM | 640 GB | 900 GB/s | FP16 | 45-60 t/s | 9 kW | ~$250K |

🔋 **Lowest power:** 4x A100 80GB at 2 kW system power

**Solar for 405B (4x A100, 12 hrs/day summer):**
- Daily: 2 kW × 12 hrs = 24 kWh
- Solar array: 6.5 kW (~$2-3.5K)
- Battery: 30 kWh (~$12-18K)
- **Solar total: $20-30K**
- **Complete system: $80-90K**

---

### Llama 3.1 70B / Qwen 2.5 72B (Sweet Spot Models)

**VRAM needed:** ~35-40 GB (Q4), ~70 GB (Q8), ~140 GB (FP16)

**Single-GPU options exist!**

| Hardware | VRAM | Single GPU? | Quant | Speed | System Power | Hardware Cost |
|----------|------|-------------|-------|-------|--------------|---------------|
| **A100 80GB** | 80 GB | ✅ Yes | Q8 | 30-38 t/s | **400W** | ~$15K used |
| H100 80GB | 80 GB | ✅ Yes | FP16 | 50-60 t/s | 800W | ~$30K |
| MI300X | 192 GB | ✅ Yes | FP16 | 45-55 t/s | 850W | ~$15K |
| H200 | 141 GB | ✅ Yes | FP16 | 50-60 t/s | 800W | ~$35K |
| 2x RTX 4090 + NVLink | 48 GB | No (2-way) | Q4 | 35-40 t/s | 1.3 kW | ~$4K |
| 2x RTX 5090 | 64 GB | No (PCIe) | Q8 | 40-50 t/s | 1.65 kW | ~$5K |

🔋 **Lowest power:** Single A100 80GB at 400W system power

**Solar for 70B (A100 80GB, 12 hrs/day summer):**
- Daily: 0.4 kW × 12 hrs = 4.8 kWh
- Solar array: 1.3 kW (~$500-800)
- Battery: 6 kWh (~$2.5-4K)
- **Solar total: $4-7K**
- **Complete system: $19-22K**

---

### Qwen 2.5 Coder 32B / DeepSeek R1-Distill-32B

**VRAM needed:** ~17 GB (Q4), ~32 GB (Q8), ~64 GB (FP16)

**Single-GPU options!**

| Hardware | VRAM | Quant | Speed | System Power | Hardware Cost |
|----------|------|-------|-------|--------------|---------------|
| **A100 40GB** | 40 GB | FP16 | 28-35 t/s | **300W** | ~$8K used |
| RTX 5090 | 32 GB | Q8 | 55-65 t/s | 650W | ~$2.5K |
| A6000 | 48 GB | FP16 | 28-35 t/s | 350W | ~$4.5K |
| A100 80GB | 80 GB | FP16 | 32-40 t/s | 400W | ~$15K |
| RTX 4090 | 24 GB | Q4 | 30-40 t/s | 550W | ~$2K |

🔋 **Lowest power:** A100 40GB at 300W system power
🚀 **Best speed/watt:** RTX 5090 (best perf, reasonable power)

**Solar for Coder 32B (A100 40GB, 12 hrs/day summer):**
- Daily: 0.3 kW × 12 hrs = 3.6 kWh
- Solar array: 1 kW (~$350-500)
- Battery: 5 kWh (~$2-3K)
- **Solar total: $3-5K**
- **Complete system: $11-13K**

---

### Llama 3.1 8B / Qwen 2.5 7B (Consumer Models)

**VRAM needed:** ~4 GB (Q4), ~8 GB (Q8), ~16 GB (FP16)

**Any modern GPU works.**

| Hardware | VRAM | Quant | Speed | System Power | Hardware Cost |
|----------|------|-------|-------|--------------|---------------|
| **RTX 4060 Ti 16GB** | 16 GB | FP16 | 35-45 t/s | **200W** | ~$450 |
| RTX 4070 | 12 GB | Q8 | 50-60 t/s | 250W | ~$550 |
| RTX 4090 | 24 GB | FP16 | 140-160 t/s | 550W | ~$2K |
| RTX 5090 | 32 GB | FP16 | 200+ t/s | 650W | ~$2.5K |

🔋 **Lowest power:** RTX 4060 Ti at 200W system power

**Solar for 8B (RTX 4060 Ti, 12 hrs/day summer):**
- Daily: 0.2 kW × 12 hrs = 2.4 kWh
- Solar array: 0.7 kW (~$250-400)
- Battery: 3 kWh (~$1.2-1.8K)
- **Solar total: $2-3K**
- **Complete system: $2.5-3.5K**

---

### Summary: Model → Lowest Power Hardware → Solar (12 hrs/day summer)

| Model | Lowest Power Hardware | Single GPU? | System Power | Hardware | Solar | **Total** |
|-------|----------------------|-------------|--------------|----------|-------|-----------|
| **R1 671B** | 8x A100 80GB | No (NVLink) | 3.5 kW | $120K | $35-55K | **$155-175K** |
| **405B** | 4x A100 80GB | No (NVLink) | 2 kW | $60K | $20-30K | **$80-90K** |
| **70B** | A100 80GB | ✅ Yes | 400W | $15K | $4-7K | **$19-22K** |
| **Coder 32B** | A100 40GB | ✅ Yes | 300W | $8K | $3-5K | **$11-13K** |
| **8B** | RTX 4060 Ti | ✅ Yes | 200W | $450 | $2-3K | **$2.5-3.5K** |

---

## 📊 System Profile Grids

Here's where operational flexibility meets cost reality. Each grid shows what you need for different usage patterns.

### tinybox pro v2 (6kW) - All Profiles

| Profile | Hours/Day (Summer) | Hours/Day (Winter) | Solar Array | Battery Bank | Solar System Cost | Annual Runtime |
|---------|-------------------|-------------------|-------------|--------------|-------------------|----------------|
| **A: 24/7 Year-Round** | 24 | 24 (+ gen) | 40 kW | 460 kWh | $220-350K | 8,760 hrs |
| **B: 24/7 Summer, 8hr Winter** | 24 | 8 | 40 kW | 250 kWh | $140-210K | 5,840 hrs |
| **C: Daylight Year-Round** | 15 | 9 | 25 kW | 135 kWh | $85-130K | 4,380 hrs |
| **D: Daylight Summer, 4hr Winter** | 15 | 4 | 25 kW | 90 kWh | $65-100K | 3,285 hrs |
| **E: Peak Hours Only** | 8 | 4 | 13 kW | 65 kWh | $45-70K | 2,190 hrs |

**Cost breakdown by profile (solar system only, not including tinybox):**

| Profile | Panels | Battery | Inverter/BOS | Install | **Total** |
|---------|--------|---------|--------------|---------|-----------|
| A (24/7) | $13-20K | $185-275K | $15-30K | $10-25K | **$220-350K** |
| B (24/7 summer) | $13-20K | $100-150K | $15-25K | $10-20K | **$140-210K** |
| C (daylight) | $8-12K | $55-80K | $12-20K | $8-15K | **$85-130K** |
| D (daylight/minimal) | $8-12K | $36-54K | $10-18K | $8-15K | **$65-100K** |
| E (peak only) | $4-7K | $26-39K | $8-15K | $6-12K | **$45-70K** |

**Insight:** Going from 24/7 (Profile A) to daylight-only (Profile C) saves $135-220K—mostly batteries.

---

### tinybox green v2 (3.25kW) - All Profiles

| Profile | Hours/Day (Summer) | Hours/Day (Winter) | Solar Array | Battery Bank | Solar System Cost | Annual Runtime |
|---------|-------------------|-------------------|-------------|--------------|-------------------|----------------|
| **A: 24/7 Year-Round** | 24 | 24 (+ gen) | 22 kW | 250 kWh | $120-185K | 8,760 hrs |
| **B: 24/7 Summer, 8hr Winter** | 24 | 8 | 22 kW | 135 kWh | $80-120K | 5,840 hrs |
| **C: Daylight Year-Round** | 15 | 9 | 14 kW | 72 kWh | $50-75K | 4,380 hrs |
| **D: Daylight Summer, 4hr Winter** | 15 | 4 | 14 kW | 50 kWh | $40-60K | 3,285 hrs |
| **E: Peak Hours Only** | 8 | 4 | 7 kW | 36 kWh | $28-42K | 2,190 hrs |

---

### tinybox red v2 (1.6kW) - All Profiles

| Profile | Hours/Day (Summer) | Hours/Day (Winter) | Solar Array | Battery Bank | Solar System Cost | Annual Runtime |
|---------|-------------------|-------------------|-------------|--------------|-------------------|----------------|
| **A: 24/7 Year-Round** | 24 | 24 (+ gen) | 11 kW | 125 kWh | $65-95K | 8,760 hrs |
| **B: 24/7 Summer, 8hr Winter** | 24 | 8 | 11 kW | 66 kWh | $42-62K | 5,840 hrs |
| **C: Daylight Year-Round** | 15 | 9 | 7 kW | 35 kWh | $28-42K | 4,380 hrs |
| **D: Daylight Summer, 4hr Winter** | 15 | 4 | 7 kW | 24 kWh | $22-34K | 3,285 hrs |
| **E: Peak Hours Only** | 8 | 4 | 3.5 kW | 18 kWh | $15-24K | 2,190 hrs |

---

### 2x RTX 4090 (1.3kW) - All Profiles

| Profile | Hours/Day (Summer) | Hours/Day (Winter) | Solar Array | Battery Bank | Solar System Cost | Annual Runtime |
|---------|-------------------|-------------------|-------------|--------------|-------------------|----------------|
| **A: 24/7 Year-Round** | 24 | 24 (+ gen) | 9 kW | 100 kWh | $50-75K | 8,760 hrs |
| **B: 24/7 Summer, 8hr Winter** | 24 | 8 | 9 kW | 54 kWh | $35-52K | 5,840 hrs |
| **C: Daylight Year-Round** | 15 | 9 | 5.5 kW | 29 kWh | $22-34K | 4,380 hrs |
| **D: Daylight Summer, 4hr Winter** | 15 | 4 | 5.5 kW | 20 kWh | $18-28K | 3,285 hrs |
| **E: Peak Hours Only** | 8 | 4 | 3 kW | 15 kWh | $12-20K | 2,190 hrs |

---

### Single RTX 4090 (650W) - All Profiles

| Profile | Hours/Day (Summer) | Hours/Day (Winter) | Solar Array | Battery Bank | Solar System Cost | Annual Runtime |
|---------|-------------------|-------------------|-------------|--------------|-------------------|----------------|
| **A: 24/7 Year-Round** | 24 | 24 (+ gen) | 4.5 kW | 50 kWh | $25-40K | 8,760 hrs |
| **B: 24/7 Summer, 8hr Winter** | 24 | 8 | 4.5 kW | 27 kWh | $18-28K | 5,840 hrs |
| **C: Daylight Year-Round** | 15 | 9 | 2.8 kW | 14 kWh | $12-18K | 4,380 hrs |
| **D: Daylight Summer, 4hr Winter** | 15 | 4 | 2.8 kW | 10 kWh | $10-15K | 3,285 hrs |
| **E: Peak Hours Only** | 8 | 4 | 1.5 kW | 8 kWh | $7-12K | 2,190 hrs |

---

### 8x H100 / DGX (9kW) - All Profiles

| Profile | Hours/Day (Summer) | Hours/Day (Winter) | Solar Array | Battery Bank | Solar System Cost | Annual Runtime |
|---------|-------------------|-------------------|-------------|--------------|-------------------|----------------|
| **A: 24/7 Year-Round** | 24 | 24 (+ gen) | 60 kW | 690 kWh | $350-565K | 8,760 hrs |
| **B: 24/7 Summer, 8hr Winter** | 24 | 8 | 60 kW | 370 kWh | $210-320K | 5,840 hrs |
| **C: Daylight Year-Round** | 15 | 9 | 38 kW | 200 kWh | $130-200K | 4,380 hrs |
| **D: Daylight Summer, 4hr Winter** | 15 | 4 | 38 kW | 135 kWh | $100-155K | 3,285 hrs |
| **E: Peak Hours Only** | 8 | 4 | 20 kW | 100 kWh | $70-110K | 2,190 hrs |

---

## 📈 Profile Comparison: Total System Cost (Hardware + Solar)

What you actually spend for the complete setup:

### tinybox pro v2 ($50K hardware)

| Profile | Solar Cost | **Total Investment** | $/hour runtime |
|---------|------------|---------------------|----------------|
| A: 24/7 | $220-350K | **$270-400K** | $31-46 |
| B: 24/7 summer | $140-210K | **$190-260K** | $33-45 |
| C: Daylight | $85-130K | **$135-180K** | $31-41 |
| D: Daylight/minimal | $65-100K | **$115-150K** | $35-46 |
| E: Peak only | $45-70K | **$95-120K** | $43-55 |

### tinybox green v2 ($50K hardware)

| Profile | Solar Cost | **Total Investment** | $/hour runtime |
|---------|------------|---------------------|----------------|
| A: 24/7 | $120-185K | **$170-235K** | $19-27 |
| B: 24/7 summer | $80-120K | **$130-170K** | $22-29 |
| C: Daylight | $50-75K | **$100-125K** | $23-29 |
| D: Daylight/minimal | $40-60K | **$90-110K** | $27-33 |
| E: Peak only | $28-42K | **$78-92K** | $36-42 |

### tinybox red v2 ($10K hardware)

| Profile | Solar Cost | **Total Investment** | $/hour runtime |
|---------|------------|---------------------|----------------|
| A: 24/7 | $65-95K | **$75-105K** | $9-12 |
| B: 24/7 summer | $42-62K | **$52-72K** | $9-12 |
| C: Daylight | $28-42K | **$38-52K** | $9-12 |
| D: Daylight/minimal | $22-34K | **$32-44K** | $10-13 |
| E: Peak only | $15-24K | **$25-34K** | $11-16 |

---

## 🗓️ Monthly Availability by Profile

What "reduced winter" actually means for your workflow:

### Profile D: Daylight Summer, 4hr Winter

| Month | Day Length | Usable Hours | Daily kWh (6kW) | Notes |
|-------|------------|--------------|-----------------|-------|
| June | 15.5 hrs | 15 hrs | 90 kWh | Full workday + evening |
| July | 15 hrs | 15 hrs | 90 kWh | Full workday + evening |
| August | 14 hrs | 14 hrs | 84 kWh | Full workday |
| September | 12.5 hrs | 10 hrs | 60 kWh | Workday only |
| October | 11 hrs | 8 hrs | 48 kWh | Core hours |
| November | 9.5 hrs | 6 hrs | 36 kWh | Midday only |
| December | 9 hrs | 4 hrs | 24 kWh | Peak sun only |
| January | 9.5 hrs | 4 hrs | 24 kWh | Peak sun only |
| February | 10.5 hrs | 6 hrs | 36 kWh | Midday expanding |
| March | 12 hrs | 8 hrs | 48 kWh | Core hours |
| April | 13.5 hrs | 10 hrs | 60 kWh | Workday only |
| May | 14.5 hrs | 14 hrs | 84 kWh | Full workday |

**Annual total:** ~3,285 hours = 37.5% of the year
**Summer months (May-Aug):** ~1,770 hours at full capacity
**Winter months (Nov-Feb):** ~600 hours at reduced capacity

---

## 💡 Choosing Your Profile

### Profile A (24/7 Year-Round)
**Choose if:**
- Running production services others depend on
- Need overnight batch training
- Can't accept any downtime
- Have budget for massive battery bank
- Will supplement with generator in winter

**Realistic for:** Funded startups, research labs, serious hobbyists with deep pockets

### Profile B (24/7 Summer, Reduced Winter)
**Choose if:**
- Heavy use is seasonal (research cycles, summer projects)
- Can schedule intensive work for summer months
- Winter is for lighter tasks or maintenance
- Want overnight capability at least part of year

**Realistic for:** Academic researchers, seasonal businesses, flexible schedules

### Profile C (Daylight Year-Round)
**Choose if:**
- Working normal hours anyway
- Can run jobs during the day and review results in evening
- Don't need 3 AM inference
- Want reliable year-round availability

**Realistic for:** Most individual users, home offices, development work

### Profile D (Daylight Summer, Minimal Winter)
**Choose if:**
- Primarily fair-weather computing
- Can accept limited winter access
- Budget-conscious
- Have grid backup for winter if really needed

**Realistic for:** Hobbyists, experimenters, cost-sensitive users

### Profile E (Peak Hours Only)
**Choose if:**
- Running scheduled batch jobs
- Training runs that can queue for good solar
- Minimal interactive use
- Maximum cost savings

**Realistic for:** Training-focused users, batch processing, scheduled workloads

---

## 📊 Summary: All Systems by Profile

### Quick Reference: Profile A (24/7 Year-Round)

| System | Power | Solar Array | Battery | Solar Cost | Total |
|--------|-------|-------------|---------|------------|-------|
| 1x RTX 4090 | 650W | 4.5 kW | 50 kWh | $25-40K | $27-42K |
| 1x RTX 5090 | 825W | 5.5 kW | 65 kWh | $32-50K | $35-54K |
| 2x RTX 4090 | 1.3kW | 9 kW | 100 kWh | $50-75K | $55-82K |
| 2x RTX 5090 | 1.65kW | 11.5 kW | 130 kWh | $65-95K | $72-103K |
| tinybox red v2 | 1.6kW | 11 kW | 125 kWh | $65-95K | $75-105K |
| tinybox green v2 | 3.25kW | 22 kW | 250 kWh | $120-185K | $170-235K |
| tinybox pro v2 | 6kW | 40 kW | 460 kWh | $220-350K | $270-400K |
| 8x H100 | 9kW | 60 kW | 690 kWh | $350-565K | $650-865K |

### Quick Reference: Profile C (Daylight Year-Round)

| System | Power | Solar Array | Battery | Solar Cost | Total |
|--------|-------|-------------|---------|------------|-------|
| 1x RTX 4090 | 650W | 2.8 kW | 14 kWh | $12-18K | $14-20K |
| 1x RTX 5090 | 825W | 3.5 kW | 18 kWh | $15-22K | $18-26K |
| 2x RTX 4090 | 1.3kW | 5.5 kW | 29 kWh | $22-34K | $27-41K |
| 2x RTX 5090 | 1.65kW | 7 kW | 36 kWh | $28-42K | $35-50K |
| tinybox red v2 | 1.6kW | 7 kW | 35 kWh | $28-42K | $38-52K |
| tinybox green v2 | 3.25kW | 14 kW | 72 kWh | $50-75K | $100-125K |
| tinybox pro v2 | 6kW | 25 kW | 135 kWh | $85-130K | $135-180K |
| 8x H100 | 9kW | 38 kW | 200 kWh | $130-200K | $430-500K |

### Quick Reference: Profile D (Daylight Summer, 4hr Winter)

| System | Power | Solar Array | Battery | Solar Cost | Total |
|--------|-------|-------------|---------|------------|-------|
| 1x RTX 4090 | 650W | 2.8 kW | 10 kWh | $10-15K | $12-17K |
| 1x RTX 5090 | 825W | 3.5 kW | 13 kWh | $12-18K | $15-22K |
| 2x RTX 4090 | 1.3kW | 5.5 kW | 20 kWh | $18-28K | $23-35K |
| 2x RTX 5090 | 1.65kW | 7 kW | 26 kWh | $22-34K | $29-42K |
| tinybox red v2 | 1.6kW | 7 kW | 24 kWh | $22-34K | $32-44K |
| tinybox green v2 | 3.25kW | 14 kW | 50 kWh | $40-60K | $90-110K |
| tinybox pro v2 | 6kW | 25 kW | 90 kWh | $65-100K | $115-150K |
| 8x H100 | 9kW | 38 kW | 135 kWh | $100-155K | $400-455K |

### The Cost Spectrum (tinybox pro v2 example)

| Profile | What You Get | Solar Cost | Savings vs 24/7 |
|---------|--------------|------------|-----------------|
| A: 24/7 | Always on, generator in winter | $220-350K | Baseline |
| B: 24/7 summer, 8hr winter | Full summer, limited winter | $140-210K | **Save $80-140K** |
| C: Daylight year-round | 9-15 hrs/day all year | $85-130K | **Save $135-220K** |
| D: Daylight/minimal winter | 15 hrs summer, 4 hrs winter | $65-100K | **Save $155-250K** |
| E: Peak only | 4-8 hrs/day | $45-70K | **Save $175-280K** |

---

## ⛽ Generator Backup Systems

Generators bridge the gap when solar + battery can't keep up (winter, extended clouds).

### Generator Comparison

| Fuel | Efficiency | Fuel Cost | Runtime/Gallon | Noise | Maintenance | Best For |
|------|------------|-----------|----------------|-------|-------------|----------|
| **Diesel** | 35-40% | $3.50-4.50 | 15-18 kWh | Medium | Medium | High loads, long runtime |
| **Propane** | 25-30% | $2.50-3.50 | 8-10 kWh | Low | Low | Backup, clean storage |
| **Natural Gas** | 25-30% | $1.50-2.50 | ~10 kWh equiv | Low | Low | Unlimited fuel (piped) |
| **Gasoline** | 20-25% | $3.00-4.00 | 8-12 kWh | High | High | Small loads, short term |

### Diesel Generator (Recommended for Heavy Loads)

**For 10kW continuous:**
- Generator size: 15-20 kW (run at 60-70% load for efficiency)
- Fuel consumption: ~1.5 gal/hour at load
- Daily fuel (24hr): ~36 gallons = ~$130-160/day
- Monthly (full winter reliance): ~$4,000-5,000

**Recommended models:**
| Model | Capacity | Fuel Rate | Price |
|-------|----------|-----------|-------|
| Generac 22kW | 22 kW | 1.8 gal/hr | $5,000-7,000 |
| Cummins RS20A | 20 kW | 1.5 gal/hr | $8,000-12,000 |
| Kubota GL14000 | 14 kW | 1.1 gal/hr | $12,000-18,000 |
| Cat DE22 | 22 kW | 1.6 gal/hr | $15,000-25,000 |

### Propane Generator

Cleaner than diesel, stores indefinitely, but less efficient.

**For 10kW continuous:**
- Generator size: 15-22 kW
- Fuel consumption: ~3-4 gal/hr propane
- Daily fuel: ~72-96 gallons = ~$180-340/day
- Requires large propane tanks (500-1000 gallon)

**Advantages:**
- No fuel degradation (stores 30+ years)
- Cleaner exhaust
- Lower maintenance
- Quieter operation

**Disadvantages:**
- Higher fuel cost per kWh
- Needs large tank installation ($2-5K)
- Less power per gallon

### Natural Gas Generator

If you have a gas line, this is the most convenient backup.

**For 10kW continuous:**
- Generator size: 15-22 kW
- Fuel consumption: ~200-250 cubic feet/hr
- Daily cost: ~$50-80/day (varies by region)
- No fuel storage needed

**Advantages:**
- Unlimited runtime (piped fuel)
- Low maintenance
- No fuel management
- Automatic transfer switches common

**Disadvantages:**
- Requires gas line
- Lower energy density
- Grid-dependent fuel supply (defeats off-grid)

---

## 🔄 Hybrid System Design

The practical approach: solar primary, generator backup, intelligent switching.

### Architecture

```
Solar Array (40kW+)
       │
       ▼
Charge Controllers (MPPT)
       │
       ▼
Battery Bank (500kWh+) ◄──── Generator (Auto-Start)
       │                            │
       ▼                            │
Inverter System ◄───────────────────┘
       │
       ▼
Main Panel ──► LLM Hardware
```

### Automatic Generator Integration

1. **Voltage-based start:** Generator kicks on when battery hits 30% SOC
2. **Generator runs until:** Battery reaches 80% SOC
3. **Priority:** Solar charges first when available
4. **Winter mode:** Generator scheduled for overnight when solar insufficient

### Recommended: Dual-Fuel Generator

Some generators run on both propane and diesel/gasoline:
- Use natural gas/propane for short outages
- Switch to diesel for extended winter operation
- Redundant fuel sources

---

## 🏠 Installation Considerations

### Space Requirements

| Array Size | Ground Mount Area | Roof Area Needed |
|------------|-------------------|------------------|
| 10 kW | ~700 sq ft | ~600 sq ft |
| 25 kW | ~1,750 sq ft | ~1,500 sq ft |
| 50 kW | ~3,500 sq ft | ~3,000 sq ft |
| 100 kW | ~7,000 sq ft | N/A (commercial) |

### Battery Room

- 500 kWh battery bank: ~200-400 sq ft
- Requires ventilation (hydrogen off-gassing with some chemistries)
- Temperature control (heating in winter for LiFePO4)
- Fire suppression recommended for large installations

### Electrical Infrastructure

| System Power | Service Size | Typical Cost |
|--------------|--------------|--------------|
| <3 kW | 100A existing | Minimal |
| 3-6 kW | 200A upgrade | $2,000-5,000 |
| 6-10 kW | 200A + subpanel | $5,000-10,000 |
| 10-20 kW | 400A service | $10,000-25,000 |

---

## 📈 Seasonal Operation Strategy

### Summer (May-August)
- **Solar surplus:** Array produces more than daily need
- **Battery strategy:** Shallow cycling (60-80% range)
- **Generator:** Off (maybe 5-10 days total)
- **Excess power:** Can run other loads, or waste it

### Spring/Fall (March-April, September-October)
- **Solar marginal:** Good days self-sufficient, cloudy days draw down
- **Battery strategy:** Normal cycling (40-80% range)
- **Generator:** Occasional multi-day cloudy periods (2-4 times/month)

### Winter (November-February)
- **Solar insufficient:** ~25-40% of needs
- **Battery strategy:** Generator recharges overnight
- **Generator:** Daily or every-other-day runtime
- **Generator hours:** 8-12 hours/day typical

### Annual Generator Usage Estimate

| System | Summer | Spring/Fall | Winter | Annual Hours | Annual Fuel Cost |
|--------|--------|-------------|--------|--------------|------------------|
| 2x 4090 (1.3kW) | 50 hrs | 200 hrs | 800 hrs | ~1,050 hrs | ~$700-1,000 |
| tinybox red v2 (1.6kW) | 60 hrs | 250 hrs | 1,000 hrs | ~1,300 hrs | ~$1,000-1,400 |
| tinybox green v2 (3.25kW) | 80 hrs | 400 hrs | 1,500 hrs | ~2,000 hrs | ~$2,500-3,500 |
| tinybox pro v2 (6kW) | 100 hrs | 500 hrs | 2,000 hrs | ~2,600 hrs | ~$5,000-7,000 |

At these generator hours, diesel makes economic sense over propane.

---

## 💰 Total Cost of Ownership (10 Years)

### tinybox pro v2 Example

| Cost Category | Year 0 | Annual | 10-Year Total |
|---------------|--------|--------|---------------|
| tinybox pro v2 | $50,000 | - | $50,000 |
| Solar array | $15,000 | $500 maintenance | $20,000 |
| Battery bank (750kWh) | $250,000 | $1,000 maintenance | $260,000 |
| Inverters/controllers | $20,000 | $500 | $25,000 |
| Generator (20kW diesel) | $10,000 | $500 maintenance | $15,000 |
| Fuel (2,600 hrs/yr) | - | $6,000 | $60,000 |
| Installation | $30,000 | - | $30,000 |
| Electrical upgrades | $15,000 | - | $15,000 |
| **Total** | ~$390,000 | ~$8,500/yr | **~$475,000** |

### Comparison: Grid Power (10 Years)

| Cost Category | Annual | 10-Year Total |
|---------------|--------|---------------|
| Electricity (52,600 kWh × $0.15) | $7,890 | $78,900 |
| tinybox pro v2 | - | $50,000 |
| **Total** | | **~$129,000** |

**Break-even: Never for pure economics.** Off-grid is about independence, not savings.

However, if grid electricity is $0.30/kWh or higher, the math gets closer:
- 10-year grid cost at $0.30: ~$208,000
- Still doesn't break even, but gap narrows

---

## 🔧 Equipment Recommendations

### Solar Panels

| Tier | Brand | Efficiency | Warranty | Price/Watt |
|------|-------|------------|----------|------------|
| Premium | SunPower, LG | 22-23% | 25 years | $0.80-1.20 |
| Mid-tier | Canadian Solar, REC | 20-21% | 25 years | $0.50-0.70 |
| Budget | Trina, Longi | 19-20% | 12-25 years | $0.25-0.40 |

For 40+ kW arrays, mid-tier panels are the sweet spot.

### Inverters

| Type | Brand | Capacity | Price | Notes |
|------|-------|----------|-------|-------|
| String | Fronius, SMA | 5-15 kW | $2,000-5,000 | Simple, efficient |
| Hybrid | Sol-Ark, Outback | 8-15 kW | $4,000-8,000 | Battery integration |
| Off-grid | Victron, Schneider | 3-15 kW | $3,000-10,000 | Island capable |

**For true off-grid:** Victron Quattro or Sol-Ark systems handle solar + battery + generator seamlessly.

### Battery Systems

| Brand | Chemistry | Capacity | Price | Warranty |
|-------|-----------|----------|-------|----------|
| Tesla Powerwall | LiFePO4 | 13.5 kWh | $12,000 | 10 years |
| Enphase IQ | LiFePO4 | 5/10 kWh | $6-10,000 | 10 years |
| SimpliPhi | LiFePO4 | 3.8 kWh | $3,500 | 10 years |
| EG4 | LiFePO4 | 5.12-14.3 kWh | $1,500-4,000 | 10 years |
| DIY Cells | LiFePO4 | Variable | $100-150/kWh | None |

**For large banks (500+ kWh):** DIY from EV or server rack cells is 50-70% cheaper but requires electrical knowledge.

### Charge Controllers

| Brand | Type | Capacity | Price |
|-------|------|----------|-------|
| Victron SmartSolar | MPPT | 250/100 (10kW) | $800-1,000 |
| Outback FLEXmax | MPPT | 80A (4kW) | $600-800 |
| Midnite Classic | MPPT | 150A (8kW) | $700-900 |

Multiple controllers in parallel for large arrays.

---

## ⚠️ Failure Modes and Mitigation

| Failure | Impact | Mitigation |
|---------|--------|------------|
| Cloudy week | Battery depletes | Generator auto-start |
| Inverter failure | Total loss | Redundant inverters |
| Battery cell failure | Reduced capacity | BMS isolation, modularity |
| Generator failure | No backup | Dual fuel or spare generator |
| Panel damage (hail) | Reduced generation | Insurance, spare panels |
| Charge controller failure | No charging | Multiple controllers |

Design for N+1 redundancy on critical components.

---

## 🔗 Related Concepts

- [[LLM Inference Hardware]] - Power requirements
- [[LLM Under Your Floorboards]] - Running local LLMs
- [[tinybox]] - Pre-built multi-GPU systems
- [[Solar Panel]] - Photovoltaic fundamentals
- [[LiFePO4]] - Battery chemistry
- [[Inverter]] - DC to AC conversion
- [[MPPT]] - Maximum power point tracking
- [[Off-Grid]] - Self-sufficient power systems
- [[UPS]] - Uninterruptible power supply
- [[Generator]] - Backup power generation

---

## 📚 External Resources

- [NREL PVWatts Calculator](https://pvwatts.nrel.gov/) - Solar production estimates
- [Solar Irradiance Data (NSRDB)](https://nsrdb.nrel.gov/) - Historical sun data
- [Will Prowse YouTube](https://youtube.com/@WillProwse) - DIY solar deep dives
- [Victron Energy Docs](https://www.victronenergy.com/live/start) - Professional off-grid
- [EG4 Electronics](https://eg4electronics.com/) - Budget LiFePO4
- [Signature Solar](https://signaturesolar.com/) - DIY components
- [Solar Edge Designer](https://www.solaredge.com/us/products/installer-tools/system-designer) - System planning

