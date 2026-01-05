# LLM Model Guide

A practical guide to open-weight large language models—what they're good at, how they compare on benchmarks, and which to choose for your use case. This focuses on models you can actually run locally, not cloud-only offerings.

For hardware requirements and tokens/second performance, see [[LLM Inference Hardware]]. For running unrestricted models and censorship levels, see [[LLM Under Your Floorboards]].

---

## 🎯 The Big Picture

As of late 2025, the open-weight LLM landscape is dominated by four major families:

| Family | Developer | Strengths | Weaknesses |
|--------|-----------|-----------|------------|
| **[[Llama 3]]** | Meta | Broad ecosystem, stable, well-supported | Not best-in-class at any specific task |
| **[[Qwen]]** | Alibaba | Coding, math, multilingual (29+ languages) | Chinese political censorship |
| **[[DeepSeek]]** | DeepSeek | Coding, reasoning, efficiency (MoE) | Chinese political censorship |
| **[[Mistral]]** / [[Mixtral]] | Mistral AI | European, less restricted, efficient MoE | Smaller community than Llama |

---

## 📊 Understanding Benchmarks

Before comparing models, understand what benchmarks measure:

| Benchmark | Measures | What It Means |
|-----------|----------|---------------|
| **MMLU** | General knowledge (57 subjects) | How much the model "knows" |
| **MMLU-Pro** | Harder MMLU variant | Better discriminates top models |
| **HumanEval** | Python code generation | Can it write working code? |
| **MBPP** | Basic Python problems | Simpler coding tasks |
| **LiveCodeBench** | Real-world coding | More realistic than HumanEval |
| **GSM8K** | Grade-school math | Basic arithmetic reasoning |
| **MATH** | Competition math | Hard mathematical reasoning |
| **AIME** | Math olympiad problems | Elite math capability |
| **GPQA** | PhD-level science | Expert knowledge |
| **ARC-Challenge** | Common-sense reasoning | Basic logic |
| **HellaSwag** | Sentence completion | Language understanding |
| **TruthfulQA** | Factual accuracy | Hallucination resistance |

**Benchmark saturation:** Many models now score 85%+ on MMLU and 90%+ on GSM8K. These benchmarks no longer discriminate well between top models. MMLU-Pro, GPQA, and AIME are now more meaningful.

---

## 🏆 Model Comparison by Task

### General Intelligence (Late 2025)

| Model | MMLU | MMLU-Pro | Parameters | Notes |
|-------|------|----------|------------|-------|
| [[Qwen]] 2.5-Max | 87.9% | 76.1% | ~200B+ | Top open general knowledge |
| [[DeepSeek]] V3 | 87.1% | 75.9% | 671B MoE | Very close to Qwen |
| [[Llama 3]] 405B | 86.1% | ~72% | 405B | Largest Llama |
| [[Qwen]] 2.5 72B | 85.3% | 71.2% | 72B | Sweet spot for quality/size |
| [[Llama 3]] 70B | 82.0% | ~68% | 70B | Solid all-rounder |
| [[Mixtral]] 8x22B | 77.8% | ~65% | 176B MoE (44B active) | Efficient MoE |
| [[DeepSeek]] R1-Distill-70B | 79.4% | 71.2% | 70B | Reasoning-focused |

---

### Coding Models

Coding is where model choice matters most. Specialized coding models dramatically outperform general models.

| Model | HumanEval | LiveCodeBench | Size | Best For |
|-------|-----------|---------------|------|----------|
| **[[Qwen]] 2.5 Coder 32B** | 91.0% | 69.5% | 32B | Best open coding model |
| **[[Qwen]] 2.5 Coder 7B** | 88.4% | ~55% | 7B | Amazing for size |
| **[[DeepSeek]] Coder V2** | 81.1% | ~60% | 236B MoE | Specialist, fast (MoE) |
| [[Codestral]] 22B | 81.1% | ~58% | 22B | Mistral's coding model |
| [[CodeLlama]] 70B | 72.0% | ~50% | 70B | Older but solid |
| [[CodeLlama]] 34B | 67.8% | ~45% | 34B | Good mid-range |

**Key insight:** Qwen 2.5 Coder 7B at 88.4% HumanEval outperforms models 10x its size. For coding on consumer hardware, this is the obvious choice.

**Practical recommendations:**
- **Consumer GPU (24GB):** Qwen 2.5 Coder 7B or 14B
- **Prosumer (48GB+):** Qwen 2.5 Coder 32B
- **Multi-GPU:** DeepSeek Coder V2 (MoE efficiency)

---

### Reasoning Models

"Reasoning" models use chain-of-thought (CoT) to work through problems step-by-step, often showing their thinking process.

| Model | AIME 2024 | MATH-500 | Architecture | Notes |
|-------|-----------|----------|--------------|-------|
| **[[DeepSeek]] R1** | 79.8% | 97.3% | 671B MoE | Open reasoning king |
| **[[DeepSeek]] R1-0528** | 87.5% | 97.9% | 671B MoE | Updated, even better |
| [[DeepSeek]] R1-Distill-Llama-70B | ~60% | 94.5% | 70B | Distilled R1 |
| [[DeepSeek]] R1-Distill-Qwen-32B | ~55% | 94.3% | 32B | Smaller distill |
| [[Qwen]] QwQ 32B | ~50% | ~85% | 32B | Qwen's reasoning model |

**What makes R1 special:**
- 671B total parameters, only 37B active per token (MoE)
- Shows chain-of-thought reasoning (averages 23K tokens of "thinking")
- Can inspect reasoning trace when it makes mistakes
- 90.8% MMLU, competitive with GPT-4 on most tasks
- **Open weights** - you can run this locally

**Cost context:** DeepSeek R1 API costs ~$2.19/M output tokens. GPT-4 costs ~$60/M. Similar capability, 27x cheaper (or free if local).

---

### Creative Writing & Roleplay

Different models for creative use—less about benchmarks, more about style and restrictions.

| Model | Size | Strengths | Restrictions |
|-------|------|-----------|--------------|
| **[[MythoMax]] L2** | 13B | Roleplay gold standard, character consistency | Minimal |
| **[[Nous-Hermes]] 2 Yi** | 34B | Very human-like, long context | Low |
| **[[Dolphin]] Mixtral** | 8x7B MoE | Unrestricted, efficient | Minimal |
| **[[Mistral]] Large 2** | 123B | Long-form fiction | Moderate |
| [[OpenHermes]] 2.5 Mistral | 7B | Good RP on weak hardware | Low |
| [[Llama 3]] 70B | 70B | Clean prose, controllable | Moderate (Meta safety) |
| [[Qwen]] 2.5 72B | 72B | Multilingual creative | Chinese politics |

**For uncensored creative work:** MythoMax L2 13B or Dolphin variants. These don't refuse content and maintain character well.

**For quality prose:** Llama 3 70B or Mistral Large 2 produce cleaner writing but have more guardrails.

---

## 📏 Model Size Tiers

What you get at each size level:

### 7-8B Parameters
- **VRAM:** 4-8 GB (Q4), 16 GB (FP16)
- **Speed:** 100-200 t/s on RTX 4090
- **Capability:** Basic tasks, simple coding, chat
- **Examples:** Llama 3 8B, Qwen 2.5 7B, Mistral 7B
- **Best for:** Fast iteration, consumer hardware, coding assistants

### 13-14B Parameters
- **VRAM:** 7-10 GB (Q4), 28 GB (FP16)
- **Speed:** 80-130 t/s on RTX 4090
- **Capability:** Noticeably smarter, better reasoning
- **Examples:** MythoMax L2 13B, Qwen 2.5 14B
- **Best for:** Roleplay, creative writing, better coding

### 30-34B Parameters
- **VRAM:** 17-22 GB (Q4), 68 GB (FP16)
- **Speed:** 35-60 t/s on RTX 4090 (Q4)
- **Capability:** Significant quality jump, approaches GPT-3.5
- **Examples:** Qwen 2.5 Coder 32B, Yi 34B, DeepSeek R1-Distill-32B
- **Best for:** Serious coding, complex reasoning

### 70B Parameters
- **VRAM:** 35-45 GB (Q4), 140 GB (FP16)
- **Speed:** 20-40 t/s on 2x RTX 4090
- **Capability:** Strong reasoning, competitive with GPT-4 on many tasks
- **Examples:** Llama 3 70B, Qwen 2.5 72B, DeepSeek R1-Distill-70B
- **Best for:** Production use, complex tasks, best local quality

### 100B+ Parameters
- **VRAM:** 50-200+ GB (Q4), 200-800+ GB (FP16)
- **Speed:** 10-25 t/s on multi-GPU
- **Capability:** Frontier-class, competes with best closed models
- **Examples:** Llama 3 405B, DeepSeek V3/R1 671B, Qwen 2.5-Max
- **Best for:** When you need the absolute best and have the hardware

---

## 🎯 Quick Recommendations

### By Use Case

| Use Case | Recommended Model | Why |
|----------|-------------------|-----|
| **General chat (consumer GPU)** | Llama 3 8B or Qwen 2.5 7B | Fast, capable, fits in 8GB |
| **Coding (consumer GPU)** | Qwen 2.5 Coder 7B | 88% HumanEval at 7B! |
| **Coding (prosumer)** | Qwen 2.5 Coder 32B | Matches GPT-4o on coding |
| **Complex reasoning** | DeepSeek R1-Distill-70B | Chain-of-thought, inspectable |
| **Best reasoning (multi-GPU)** | DeepSeek R1 671B | Open reasoning SOTA |
| **Creative writing** | MythoMax L2 13B | Unrestricted, consistent |
| **Roleplay** | Dolphin Mixtral 8x7B | Uncensored, efficient |
| **Long documents** | Yi 34B (200K ctx) | Best long context |
| **Math** | DeepSeek R1 or Qwen 2.5 72B | Top math reasoning |
| **Multilingual** | Qwen 2.5 (29 languages) | Best non-English support |

### By Hardware

| Hardware | Best General | Best Coding | Best Reasoning |
|----------|--------------|-------------|----------------|
| **8GB VRAM** | Llama 3 8B Q4 | Qwen Coder 7B Q4 | R1-Distill-7B |
| **16GB VRAM** | Qwen 2.5 14B Q4 | Qwen Coder 14B Q4 | R1-Distill-14B |
| **24GB VRAM** | Qwen 2.5 32B Q4 | Qwen Coder 32B Q4 | R1-Distill-32B |
| **48GB VRAM** | Qwen 2.5 72B Q4 | Qwen Coder 32B FP16 | R1-Distill-70B Q4 |
| **80GB+ VRAM** | Llama 3 70B FP16 | DeepSeek Coder V2 | R1-Distill-70B FP16 |
| **256GB+ VRAM** | Llama 3 405B Q4 | - | DeepSeek R1 |

---

## 🔄 MoE: Mixture of Experts

Some models use sparse Mixture of Experts architecture - more total parameters but only a fraction active per token.

| Model | Total Params | Active Params | Experts | Why It Matters |
|-------|--------------|---------------|---------|----------------|
| [[Mixtral]] 8x7B | 46.7B | 12.9B | 8 (2 active) | Fast like 13B, smart like 47B |
| [[Mixtral]] 8x22B | 176B | 44B | 8 (2 active) | Fast like 45B, smart like 176B |
| [[DeepSeek]] V3 | 671B | 37B | Many | Frontier quality, 37B speed |
| [[DeepSeek]] R1 | 671B | 37B | Many | Best reasoning, 37B inference cost |
| [[DBRX]] | 132B | 36B | 16 (4 active) | Databricks, efficient |

**Key insight:** MoE models give you the intelligence of a massive model at the inference cost of a much smaller one. DeepSeek R1 at 671B total only uses 37B per token - that's why it can compete with GPT-4 while being runnable on ~256GB VRAM.

---

## ⚠️ Model Censorship Levels

Not all open models are equally open about what they'll discuss:

| Model Family | Political Censorship | Content Restrictions | Notes |
|--------------|---------------------|----------------------|-------|
| **Llama 3** | Low | Moderate | Meta safety training |
| **Qwen** | Chinese topics | Low otherwise | Won't discuss Taiwan, Tiananmen |
| **DeepSeek** | Chinese topics | Low otherwise | Same Chinese restrictions |
| **Mistral** | Low | Low | European, more permissive |
| **Dolphin** | None | None | Abliterated versions |
| **Nous-Hermes** | None | Low | Minimal restrictions |
| **MythoMax** | None | None | Creative-focused |

For unrestricted use, see [[Uncensored Models]] for abliterated and uncensored model recommendations.

---

## 📈 Benchmark Trends (2025)

What's changed recently:

1. **Qwen dominates coding** - Qwen 2.5 Coder consistently beats larger models
2. **DeepSeek R1 changed reasoning** - Open-weight CoT at frontier level
3. **MoE is winning** - Best models (DeepSeek, Mixtral) are sparse
4. **7B models are surprisingly good** - Qwen Coder 7B matches older 70B on coding
5. **MMLU is saturated** - Top models all score 85%+, look at MMLU-Pro, GPQA
6. **Chinese labs leading** - Qwen and DeepSeek often beat Llama on benchmarks

---

## 🔗 Related Concepts

- [[LLM Under Your Floorboards]] - Hardware, censorship, running locally
- [[LLM Inference Hardware]] - Tokens/second, VRAM requirements
- [[LLM Inference Engines]] - llama.cpp vs vLLM vs Ollama
- [[Quantization]] - GGUF, AWQ, quality vs size tradeoffs
- [[Llama 3]] - Meta's model family
- [[Qwen]] - Alibaba's models
- [[DeepSeek]] - DeepSeek's MoE models
- [[Mixtral]] - Mistral's MoE models
- [[Dolphin]] - Uncensored fine-tunes
- [[CodeLlama]] - Meta's coding models
- [[MythoMax]] - Creative writing model
- [[LoRA]] / [[QLoRA]] - Fine-tuning methods

---

## 📚 External Resources

- [LLM Stats Leaderboards](https://llm-stats.com)
- [Lambda LLM Benchmarks](https://lambda.ai/llm-benchmarks-leaderboard)
- [Vellum LLM Leaderboard](https://www.vellum.ai/llm-leaderboard)
- [Open LLM Leaderboard (HuggingFace)](https://huggingface.co/spaces/HuggingFaceH4/open_llm_leaderboard)
- [Chatbot Arena (LMSYS)](https://chat.lmsys.org/)
- [LocalAI Master Models Directory](https://localaimaster.com/models)

