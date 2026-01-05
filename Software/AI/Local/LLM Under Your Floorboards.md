# LLM Under Your Floorboards

A comprehensive guide to running powerful, unrestricted large language models locally—off the grid, outside the cloud, beyond the reach of terms of service. This document covers hardware requirements, model selection, system builds, power infrastructure, and critically, which models will actually answer your questions without corporate hand-wringing.

The premise is simple: if you want true AI sovereignty—the ability to ask any question and get an honest answer—you need to own the weights, own the hardware, and owe nothing to any API provider.

For detailed hardware benchmarks with tokens/second performance data across all GPU tiers, see [[LLM Inference Hardware]]. For model comparisons, benchmarks, and what each model excels at, see [[LLM Model Guide]].

---

## 🎯 Why Run Local?

**Privacy:** Cloud APIs log everything. Your queries, your data, your patterns. Local inference leaves no trace.

**Availability:** Cloud services go down. They rate-limit. They deprecate models. Your local system runs when you need it.

**Censorship Resistance:** Every major AI company restricts what their models will discuss. Chemistry, medicine, security research, controversial history, weapons—entire domains of human knowledge are off-limits through their APIs.

**Cost:** At scale, API costs add up. A $2000 GPU pays for itself if you're a heavy user.

**Speed:** No network latency. No queue. Instant responses for time-sensitive applications.

**Customization:** Fine-tune on your data. Merge models. Run experimental architectures. The cloud gives you what they offer; local gives you everything.

---

## 📊 Frontier Models: The Landscape

### Closed-Source (What You're Competing Against)

| Model | Provider | Parameters (est.) | Context | Strengths | Access |
|-------|----------|-------------------|---------|-----------|--------|
| [[GPT-4]] / GPT-4o | [[OpenAI]] | ~200B+ (MoE?) | 128K | Reasoning, coding, multimodal | API only |
| [[Claude]] 3.5 Sonnet | [[Anthropic]] | Unknown | 200K | Long context, nuanced writing | API only |
| [[Claude]] 3 Opus | [[Anthropic]] | Unknown | 200K | Complex reasoning | API only |
| [[Gemini]] Ultra | [[Google]] | ~1T+ (MoE) | 1M+ | Multimodal, massive context | API only |
| [[Grok]] | [[xAI]] | Unknown | 128K | Less restricted than most | API/X Premium |

These are the ceiling. They're also inaccessible for private use. You can't download them. You can't modify them. You can't ask them certain questions.

### Open-Weight Models (What You Can Actually Run)

| Model | Parameters | Context | License | VRAM (FP16) | VRAM (Q4) | Notes |
|-------|------------|---------|---------|-------------|-----------|-------|
| **[[Llama 3]] 405B** | 405B | 128K | Llama 3 | ~810GB | ~200GB | Largest open model |
| **[[Llama 3]] 70B** | 70B | 128K | Llama 3 | ~140GB | ~35-40GB | Sweet spot for serious work |
| **[[Llama 3]] 8B** | 8B | 128K | Llama 3 | ~16GB | ~4-5GB | Runs on consumer hardware |
| **[[Qwen]] 2.5 72B** | 72B | 128K | Apache 2.0 | ~144GB | ~36-40GB | Chinese lab, very capable |
| **[[Qwen]] 2.5 32B** | 32B | 128K | Apache 2.0 | ~64GB | ~16-18GB | Good mid-tier |
| **[[DeepSeek]] V2.5** | 236B MoE | 128K | DeepSeek | ~100GB active | ~50GB | MoE = less active params |
| **[[Mixtral]] 8x22B** | 176B MoE | 64K | Apache 2.0 | ~88GB active | ~40GB | 8 experts, 2 active |
| **[[Mixtral]] 8x7B** | 56B MoE | 32K | Apache 2.0 | ~28GB active | ~13GB | Popular MoE |
| **[[Yi]] 34B** | 34B | 200K | Apache 2.0 | ~68GB | ~17GB | Long context |
| **[[Command-R]]** | 104B | 128K | CC-BY-NC | ~208GB | ~52GB | RAG-optimized |
| **[[Falcon]] 180B** | 180B | 2K | Falcon | ~360GB | ~90GB | Short context limits use |
| **[[DBRX]]** | 132B MoE | 32K | Databricks | ~66GB active | ~33GB | 16 experts, 4 active |

---

## 🧠 Model Size vs. Capability

General rules:
- **7-8B:** Can follow instructions, basic coding, simple tasks. Runs on a single consumer GPU.
- **13-14B:** Noticeably smarter. Better reasoning. Still single-GPU territory.
- **30-34B:** Significant capability jump. Approaches GPT-3.5 level. Needs 24GB+ VRAM or quantization.
- **70B:** Strong reasoning, good coding, competitive with early GPT-4. Needs multi-GPU or heavy quantization.
- **100B+ / 400B+:** Frontier-class. Requires serious hardware investment.

**MoE (Mixture of Experts):** Models like Mixtral and DeepSeek use sparse activation—only a fraction of parameters are used per token. This means a 176B MoE model might only use ~44B parameters at inference time, dramatically reducing memory requirements.

---

## 🖥️ GPU Hardware Tiers

### Consumer GPUs

| GPU | VRAM | FP16 TFLOPS | Power | Street Price | Max Model (Q4) |
|-----|------|-------------|-------|--------------|----------------|
| **[[RTX 3090]]** | 24GB | 35.6 | 350W | ~$800 used | 30-34B |
| **[[RTX 4090]]** | 24GB | 82.6 | 450W | ~$1600-2000 | 30-34B |
| **[[RTX 5090]]** | 32GB | ~100+ | 575W | ~$2000+ | 40-50B |

The **[[RTX 4090]]** is the king of consumer LLM hardware. 24GB VRAM, excellent performance, still (barely) fits in a normal case with proper cooling.

The **[[RTX 5090]]** (Blackwell consumer) ups this to 32GB—a meaningful jump for larger models.

The **[[RTX 3090]]** is the budget option—same 24GB VRAM as the 4090 at half the used price. Slower, but VRAM is the bottleneck for LLMs, not compute.

### Prosumer / Workstation GPUs

| GPU | VRAM | FP16 TFLOPS | Power | Price | Notes |
|-----|------|-------------|-------|-------|-------|
| [[RTX A5000]] | 24GB | 27.8 | 230W | ~$2500 | Blower cooler, quieter |
| **[[RTX A6000]]** | 48GB | 38.7 | 300W | ~$4500 | 48GB is the sweet spot |
| [[RTX 6000 Ada]] | 48GB | 91.1 | 300W | ~$7000 | Ada architecture |
| [[RTX 5000 Ada]] | 32GB | 65.3 | 250W | ~$4000 | Good efficiency |

The **[[RTX A6000]]** at 48GB lets you run 70B models quantized without splitting across cards.

### Datacenter GPUs

| GPU | VRAM | FP16 TFLOPS | FP8 TFLOPS | Power | Price (used/new) | Notes |
|-----|------|-------------|------------|-------|------------------|-------|
| [[A100]] 40GB | 40GB | 77.9 | - | 250W | ~$8-12K used | Common on secondary market |
| **[[A100]] 80GB** | 80GB | 77.9 | - | 300W | ~$12-18K used | 80GB is much better |
| **[[H100]] SXM** | 80GB | 267 | 1979 | 700W | ~$25-40K | Current datacenter king |
| [[H100]] PCIe | 80GB | 204 | 1513 | 350W | ~$20-30K | PCIe version, less bandwidth |
| **[[H200]]** | 141GB | 267 | 1979 | 700W | ~$30-40K | [[HBM3e]], massive memory |
| **[[B200]]** | 192GB | 500+ | 4500+ | 1000W | ~$40K+ | Blackwell, next-gen |
| [[MI300X]] | 192GB | 163 | 2610 | 750W | ~$15-25K | [[AMD]]'s answer to [[H100]] |

The **[[H100]]** is the current gold standard for serious LLM work. **[[H200]]** adds more memory. **[[B200]]** (Blackwell) is the next generation with ~2x performance.

For home use, **used [[A100]]s** are often the best value—80GB models can run 70B at good quality.

---

## 🔗 Multi-GPU: NVLink vs. PCIe

Running models larger than your single-GPU VRAM requires splitting across multiple GPUs.

| Connection | Bandwidth | Latency | Notes |
|------------|-----------|---------|-------|
| PCIe 4.0 x16 | 32 GB/s | Higher | Works but slow for large models |
| PCIe 5.0 x16 | 64 GB/s | Higher | Better but still limited |
| **NVLink 3.0** | 600 GB/s | Low | A100-era, 12 links |
| **NVLink 4.0** | 900 GB/s | Low | H100-era, 18 links |
| NVLink Bridge (Consumer) | 112 GB/s | Medium | RTX 3090/4090, 2 GPUs only |

**For serious multi-GPU inference, NVLink is essential.** PCIe-based multi-GPU setups work but with significant performance penalties, especially for tensor parallelism.

Consumer NVLink bridges (RTX 3090, 4090) only support 2 GPUs and have limited bandwidth compared to datacenter NVLink.

---

## 🏠 Complete System Builds

### Tier 1: Entry Level (~$2,500-3,500)

**Goal:** Run 7-13B models comfortably, 30B with heavy [[Quantization]]

| Component | Recommendation | Price |
|-----------|----------------|-------|
| GPU | [[RTX 4090]] 24GB | $1,600 |
| CPU | [[AMD]] [[Ryzen 7 7800X3D]] or [[Intel]] i7-14700K | $350-400 |
| RAM | 64GB [[DDR5]]-5600 | $200 |
| Motherboard | B650 or Z790 | $200 |
| Storage | 2TB [[NVMe]] Gen4 | $150 |
| PSU | 1000W 80+ Gold | $150 |
| Case + Cooling | Good airflow case | $200 |

**Power draw:** ~600-700W peak

### Tier 2: Dual 4090 (~$5,000-6,000)

**Goal:** Run 30-34B models well, 70B with [[Quantization]]

| Component | Recommendation | Price |
|-----------|----------------|-------|
| GPUs | 2x [[RTX 4090]] 24GB + [[NVLink]] Bridge | $3,400 |
| CPU | [[AMD]] [[Threadripper]] or [[Intel]] i9 | $600 |
| RAM | 128GB [[DDR5]] | $400 |
| Motherboard | TRX50 or X670E with 2x x16 slots | $400 |
| Storage | 4TB [[NVMe]] | $300 |
| PSU | 1600W 80+ Platinum | $350 |
| Case | Full tower with excellent airflow | $300 |

**Power draw:** ~1200-1400W peak. **Requires 20A circuit or 240V.**

### Tier 3: [[tinybox]] v2 pro (~$25,000)

**[[tinygrad]]'s flagship machine:**

| Spec | Details |
|------|---------|
| GPUs | 6x [[RTX 4090]] with custom [[NVLink]] topology |
| Total VRAM | 144GB |
| CPU | [[AMD]] [[EPYC]] or [[Threadripper Pro]] |
| RAM | 256GB+ [[DDR5]] |
| Power | ~3500-4000W system |
| Cooling | Custom liquid or industrial air |

**Can run:** 70B models at good quality, 405B with aggressive [[Quantization]]

**Note:** The [[tinybox]] requires **240V power** (30A circuit) and serious cooling infrastructure.

### Tier 4: Multi-[[A100]]/[[H100]] Build (~$50,000-150,000+)

**Goal:** Run 405B models, train models, production inference

| Component | Recommendation |
|-----------|----------------|
| GPUs | 4-8x [[A100]] 80GB or [[H100]] 80GB |
| Interconnect | [[NVLink]]/[[NVSwitch]] fabric |
| CPU | Dual [[AMD]] [[EPYC]] or [[Intel]] [[Xeon]] |
| RAM | 512GB-1TB [[DDR5]] |
| Storage | 8TB+ [[NVMe]] RAID |
| Networking | 100GbE or [[InfiniBand]] |
| Power | 5000-10000W |
| Cooling | Datacenter-grade or liquid |

**Pre-built options:**
- **[[Lambda Labs]] Hyperplane** - 8x [[H100]] SXM
- **[[NVIDIA DGX]] H100** - 8x [[H100]], turnkey solution (~$300K+)
- **[[Supermicro]] GPU servers** - More affordable, configurable

---

## ⚡ Power Infrastructure

| System | Typical Draw | Circuit Required | Notes |
|--------|--------------|------------------|-------|
| Single 4090 | 600-700W | Standard 15A/120V | Normal outlet works |
| Dual 4090 | 1200-1400W | 20A/120V or 15A/240V | Dedicated circuit recommended |
| 4x 4090 | 2400-2800W | 30A/240V | Must be 240V |
| tinybox v2 pro | 3500-4000W | 40A/240V | Electrician required |
| 8x H100 | 7000-8000W | Multiple 30A/240V | Industrial power |

**Practical notes:**
- US standard outlets are 15A/120V = 1800W max (less with safety margin)
- 240V circuits deliver same power at half the amperage
- Running near circuit limits trips breakers and risks fire
- UPS for clean power is recommended but expensive at high wattages
- Consider whole-house generator for true off-grid operation

---

## 🌡️ Cooling Considerations

| Cooling Type | Capacity | Noise | Cost | Notes |
|--------------|----------|-------|------|-------|
| Air (stock) | 1-2 GPUs | Loud | $ | Fine for single 4090 |
| Air (case fans) | 2-4 GPUs | Very loud | $$ | Need excellent airflow |
| AIO Liquid | 1-2 GPUs | Moderate | $$$ | Per-GPU AIO coolers exist |
| Custom Loop | 2-4 GPUs | Quiet | $$$$ | Complex, maintenance required |
| Industrial AC | Unlimited | Varies | $$$$$ | Datacenter approach |

A system with 4+ high-power GPUs generates 2000-4000W of heat. That's equivalent to a space heater. Plan accordingly.

---

## 🌐 Networking Requirements

### Model Downloads

| Model Size | Download Size (FP16) | Download Size (Q4) |
|------------|---------------------|-------------------|
| 7B | ~14GB | ~4GB |
| 13B | ~26GB | ~7GB |
| 34B | ~68GB | ~18GB |
| 70B | ~140GB | ~35GB |
| 405B | ~810GB | ~200GB |

Downloading Llama 405B on a slow connection takes days. **Gigabit internet or better recommended.**

### Distributed Inference

If running models across multiple machines:
- **Minimum:** 10GbE between nodes
- **Recommended:** 25GbE or 100GbE
- **Optimal:** InfiniBand (200-400 Gb/s)

For single-machine multi-GPU, networking doesn't matter—NVLink handles inter-GPU communication.

---

## 🔓 Model Censorship Levels

This is what you actually came here for.

### Heavily Restricted (Corporate Safety Theater)

| Model | Will Refuse | Notes |
|-------|-------------|-------|
| GPT-4/4o | Chemistry, weapons, medicine, security, "harmful" content | Extensive RLHF safety training |
| Claude 3.x | Similar to GPT-4, slightly different boundaries | Constitutional AI approach |
| Gemini | Very restricted, Google-conservative | Refuses many benign requests |
| Llama 3.x (base) | Moderate restrictions | Meta's safety training |

These models will refuse to:
- Explain how common drugs are synthesized (even legal ones like penicillin)
- Discuss weapons mechanisms in detail
- Provide security research assistance that could "be misused"
- Generate content deemed "harmful" by arbitrary corporate standards
- Answer questions about many historical events "responsibly"

### Moderately Restricted

| Model | Restrictions | Notes |
|-------|--------------|-------|
| Qwen 2.5 | Chinese-aligned but fewer Western taboos | Different cultural restrictions |
| Mixtral (base) | Some restrictions, less than Llama | Mistral is more permissive |
| DeepSeek | Chinese-aligned | Restricted on Chinese politics, less on Western topics |
| Yi | Similar to Qwen | |

### Minimally Restricted / Uncensored

| Model/Fine-tune | Description | Where to Find |
|-----------------|-------------|---------------|
| **[[Dolphin]]** (various bases) | "Cognitive liberty" fine-tunes | [[HuggingFace]]: cognitivecomputations |
| **[[WizardLM]]-Uncensored** | Abliterated [[WizardLM]] | [[HuggingFace]] |
| **[[Nous-Hermes]]** (uncensored) | Nous Research variants | [[HuggingFace]]: NousResearch |
| **[[Llama 3]]-Uncensored** | Community abliterations | Various HF repos |
| **[[Mistral]]-Uncensored** | Abliterated [[Mistral]] | [[HuggingFace]] |
| **[[MythoMax]]** | Creative writing focused, fewer restrictions | [[HuggingFace]] |
| **[[Goliath]]** | Merged model, permissive | [[HuggingFace]] |
| **[[SOLAR]]-Uncensored** | Abliterated [[SOLAR]] | [[HuggingFace]] |

### What "Abliteration" Means

Abliteration is a technique to remove safety fine-tuning from models without full retraining:
1. Identify the "refusal direction" in the model's activation space
2. Surgically remove or dampen those activation patterns
3. Result: model loses its trained refusals but retains capabilities

This works because safety training is often a thin veneer on top of the model's actual knowledge. The model "knows" the information; it's just been trained to refuse to share it.

---

## 📋 Uncensored Model Comparison

| Model | Base | Parameters | Quality | Restrictions | Best For |
|-------|------|------------|---------|--------------|----------|
| **Dolphin-2.9-Llama3-70B** | Llama 3 70B | 70B | Excellent | Minimal | General unrestricted use |
| **Dolphin-2.9-Mixtral-8x22B** | Mixtral 8x22B | 176B MoE | Excellent | Minimal | High capability, less VRAM |
| **WizardLM-70B-Uncensored** | WizardLM 70B | 70B | Very Good | Minimal | Coding + general |
| **Nous-Hermes-2-Mixtral** | Mixtral 8x7B | 56B MoE | Good | Low | Efficient uncensored |
| **Llama-3-8B-Abliterated** | Llama 3 8B | 8B | Good | Minimal | Consumer hardware |
| **MythoMax-L2-13B** | Llama 2 13B | 13B | Good | Low | Creative writing |

**Recommended starting point:** Dolphin-2.9 on whatever Llama 3 base fits your hardware.

---

## 🧪 What Unrestricted Models Will Discuss

Topics that mainstream models refuse but uncensored models will engage with:

| Category | Restricted Models Say | Uncensored Models Do |
|----------|----------------------|---------------------|
| **Chemistry** | "I can't help synthesize..." | Explain synthesis routes, mechanisms |
| **Medicine** | "Consult a doctor" | Dosing calculations, drug interactions, synthesis |
| **Security Research** | "Could be misused" | Vulnerability details, exploit development |
| **Weapons** | Refuse entirely | Mechanisms, history, manufacturing principles |
| **Controversial History** | Sanitized narratives | Multiple perspectives, uncomfortable details |
| **Dangerous Knowledge** | "I can't provide that" | Information that exists in textbooks anyway |

**Important:** This isn't about doing illegal things. It's about accessing information that:
- Is freely available in libraries and textbooks
- Was common knowledge before AI companies decided to restrict it
- Is necessary for legitimate research, education, and preparedness

The same chemistry knowledge that lets you make drugs also lets you make antibiotics when supply chains collapse. The same security knowledge that lets you attack systems lets you defend them. Knowledge is neutral; restricting it doesn't prevent misuse, it just prevents legitimate use.

---

## 🛠️ Software Stack

### Inference Engines

| Engine | GPU Support | Multi-GPU | Quantization | Best For |
|--------|-------------|-----------|--------------|----------|
| **[[llama.cpp]]** | [[CUDA]], [[ROCm]], [[Metal]] | Yes | [[GGUF]] (Q2-Q8) | CPU+GPU, flexibility |
| **[[vLLM]]** | [[CUDA]] | Yes | [[AWQ]], [[GPTQ]] | High throughput serving |
| **[[Ollama]]** | [[CUDA]], [[ROCm]] | Limited | [[GGUF]] | Easy setup |
| **[[LM Studio]]** | [[CUDA]] | No | [[GGUF]] | GUI, beginners |
| **[[text-generation-webui]]** | [[CUDA]] | Yes | Multiple | Feature-rich UI |
| **[[exllamav2]]** | [[CUDA]] | Yes | EXL2 | Fast inference |
| **[[TensorRT-LLM]]** | [[CUDA]] | Yes | FP8, INT4 | Production, [[NVIDIA]] |

### Recommended Setup

1. **For single GPU:** [[Ollama]] or [[LM Studio]] for ease; [[llama.cpp]] for flexibility
2. **For multi-GPU:** [[vLLM]] or [[exllamav2]]
3. **For maximum performance:** [[TensorRT-LLM]] (complex setup)
4. **For training/fine-tuning:** [[Axolotl]], [[Unsloth]], or [[transformers]] + [[PEFT]]

---

## 📥 Model Sources

| Source | Content | Notes |
|--------|---------|-------|
| **HuggingFace** | Everything | Primary repository |
| **TheBloke** | Quantized models | GGUF, GPTQ, AWQ versions |
| **CognitiveComputations** | Dolphin models | Uncensored fine-tunes |
| **NousResearch** | Hermes, Nous models | Quality fine-tunes |
| **Ollama Library** | Curated models | Easy pull, limited selection |
| **civitai** | Creative models | Image + some text models |

**Tip:** For large models, use `huggingface-cli` with `--resume-download` flag. Downloads can be interrupted and resumed.

---

## 🔮 Future-Proofing

### Upcoming Hardware (2025-2026)

| Hardware | Expected | Significance |
|----------|----------|--------------|
| RTX 5090 | Q1 2025 | 32GB consumer card |
| B200 | 2025 | 192GB, ~2x H100 |
| MI350 | 2025 | AMD's B200 competitor |
| GB200 NVL72 | 2025 | 72 GPUs linked, datacenter-only |

### Trends to Watch

- **Memory bandwidth** increasingly bottlenecks LLM inference
- **FP8 and FP4 quantization** becoming standard, less quality loss
- **Speculative decoding** and other inference optimizations
- **Smaller, better models** (Phi-3, Gemma 2) may reduce hardware needs
- **MoE architectures** becoming more common

---

## 🔐 OpSec Considerations

If you're serious about "under your floorboards":

| Concern | Mitigation |
|---------|------------|
| Model download tracking | VPN, Tor, download from mirrors |
| Power usage monitoring | Solar + battery, off-peak usage |
| Heat signature | Proper ventilation, don't run 24/7 |
| Network traffic analysis | Local inference only, air-gap option |
| Hardware purchase tracking | Cash, used market, multiple purchases |

**For most people, this is overkill.** Running local LLMs is currently legal everywhere. But regulatory environments can change, and the option to be private exists.

---

## 📋 Quick Reference: What Should I Buy?

| Budget | Recommendation | Can Run |
|--------|----------------|---------|
| $800-1,000 | Used [[RTX 3090]] 24GB | 30-34B quantized, 70B at Q2-Q3 |
| $2,000-2,500 | [[RTX 4090]] 24GB | 30-34B well, 70B quantized |
| $5,000-6,000 | 2x [[RTX 4090]] + NVLink | 70B at good quality |
| $15,000 | Used [[A100]] 80GB system | 70B at full quality |
| $25,000 | [[tinybox]] v2 pro or similar | 70B excellent, 405B possible |
| $50,000+ | Multi-[[H100]] system | Anything |

---

## 🔗 Related Concepts

- [[LLM Inference Hardware]] - Detailed hardware benchmarks and tokens/second data
- [[LLM Model Guide]] - Model benchmarks and recommendations
- [[LLM Inference Engines]] - llama.cpp vs vLLM vs Ollama comparison
- [[Uncensored Models]] - Abliteration technique and uncensored model families
- [[Off-Grid LLM Power Systems]] - Solar, battery, and generator sizing
- [[Local LLMs]] - Coding-focused local models
- [[Local AI]] - General local AI overview
- [[LM Studio]] - GUI for local inference
- [[LLM]] - Large Language Models overview
- [[CUDA]] - GPU compute platform
- [[CUDA Toolkit]] - NVIDIA development tools
- [[Tensor Cores]] - GPU hardware for AI
- [[GPUs]] - Graphics processing units
- [[Quantization]] - Reducing model precision
- [[NVLink]] - GPU interconnect
- [[vLLM]] - Inference engine
- [[HuggingFace]] - Model repository
- [[PyTorch]] - ML framework
- [[Llama 3]] - Meta's open model family
- [[Mixtral]] - Mistral's MoE models
- [[Dolphin]] - Uncensored fine-tunes
- [[RTX 4090]] - Consumer GPU king
- [[H100]] - Datacenter GPU
- [[A100]] - Previous gen datacenter GPU
- [[tinygrad]] - ML framework / tinybox creator
- [[ROCm]] - AMD GPU compute platform
- [[CodeLlama]] - Code-focused Llama variant

---

## 📚 External Resources

- [HuggingFace Model Hub](https://huggingface.co/models)
- [TheBloke's Quantized Models](https://huggingface.co/TheBloke)
- [r/LocalLLaMA](https://reddit.com/r/LocalLLaMA) - Community discussion
- [llama.cpp GitHub](https://github.com/ggerganov/llama.cpp)
- [vLLM Documentation](https://docs.vllm.ai/)
- [tinygrad / tinybox](https://tinygrad.org/)
- [Lambda Labs](https://lambdalabs.com/)
- [LLM Benchmarks - lmsys Arena](https://chat.lmsys.org/)
