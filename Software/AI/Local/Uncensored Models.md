# Uncensored Models

A technical deep dive into how safety training is removed from LLMs, which uncensored models exist, and what they're capable of. This covers the abliteration technique, uncensored fine-tune families, and practical guidance on finding and evaluating models without corporate restrictions.

For hardware requirements see [[LLM Inference Hardware]]. For general model comparisons see [[LLM Model Guide]]. For the broader context on running these locally see [[LLM Under Your Floorboards]].

---

## 🎯 What "Uncensored" Means

A truly uncensored model will:
- Answer questions about any topic without refusal
- Generate content regardless of subject matter
- Not append safety warnings or moralizing
- Treat the user as an adult capable of handling information

What uncensored does **not** mean:
- The model knows more (knowledge comes from pretraining, not fine-tuning)
- The model is smarter (capability is separate from willingness)
- The model will help with illegal activities (it's just information, like a textbook)

The same chemistry knowledge exists in GPT-4 and in an uncensored Llama—one just refuses to share it.

---

## 🧠 How Safety Training Works

Understanding removal requires understanding how safety gets added:

### Stage 1: Pretraining
The base model learns from internet text. It "knows" everything—synthesis routes, exploit code, controversial history. No restrictions exist at this stage.

### Stage 2: Instruction Fine-Tuning
The model learns to follow instructions, be helpful, and format responses. Still relatively unrestricted.

### Stage 3: Safety Fine-Tuning (RLHF/Constitutional AI)
This is where restrictions are added:
- Human labelers flag "harmful" outputs
- Model is trained to refuse certain queries
- Refusal patterns become embedded in weights
- Result: model "knows" information but won't share it

**Key insight:** Safety training is a thin layer on top of the model's actual knowledge. The information still exists—the model has just learned to refuse to access it.

---

## 🔬 The Abliteration Technique

Abliteration (a portmanteau of "ablation" and "liberation") surgically removes safety training without full retraining.

### How It Works

1. **Identify the Refusal Direction**
   - Generate activations for harmful and harmless prompts
   - Find the vector in activation space that differentiates them
   - This "refusal direction" is where the model "decides" to refuse

2. **Remove the Direction**
   - Subtract this vector from the model's weight matrices
   - The model can no longer "point toward" refusal
   - Other capabilities remain intact

3. **Validate**
   - Test against previously-refused prompts
   - Verify general capability hasn't degraded
   - Check for over-removal (model becomes incoherent)

### Technical Details

```python
# Simplified concept (actual implementation is more complex)
# From: https://huggingface.co/blog/mlabonne/abliteration

# 1. Collect activations for harmful vs harmless prompts
harmful_activations = model.get_activations(harmful_prompts)
harmless_activations = model.get_activations(harmless_prompts)

# 2. Find the refusal direction via PCA or mean difference
refusal_direction = mean(harmful_activations) - mean(harmless_activations)
refusal_direction = normalize(refusal_direction)

# 3. Project out the refusal direction from residual stream
for layer in model.layers:
    layer.weights -= projection(layer.weights, refusal_direction)
```

The technique works because safety training creates a separable "refusal subspace" in the model's representation. Removing this subspace removes the refusal behavior while preserving other capabilities.

### Limitations

- Removes refusals but doesn't add knowledge
- Can slightly degrade model quality if over-applied
- Doesn't work equally well on all architectures
- Some models have deeper safety training that's harder to remove

---

## 📊 Uncensored Model Families

### Dolphin (Cognitive Computations)

The gold standard for uncensored models. Eric Hartford's project removes censorship while maintaining capability.

| Model | Base | Parameters | Quality | Notes |
|-------|------|------------|---------|-------|
| Dolphin 3.0 Llama 3.1 70B | Llama 3.1 70B | 70B | Excellent | Latest, best quality |
| Dolphin 3.0 Llama 3.1 8B | Llama 3.1 8B | 8B | Very Good | Consumer-friendly |
| Dolphin 2.9.4 Llama 3.1 405B | Llama 3.1 405B | 405B | Frontier | Requires serious hardware |
| Dolphin 2.9 Mixtral 8x22B | Mixtral 8x22B | 176B MoE | Excellent | Efficient via MoE |
| Dolphin 2.9 Qwen2 72B | Qwen2 72B | 72B | Excellent | Non-Llama option |
| Dolphin Phi-3 Medium | Phi-3 Medium | 14B | Good | Small but capable |

**Philosophy:** "Cognitive liberty"—the belief that humans have the right to access information. Not about promoting harm, but about treating users as adults.

**Training approach:** Uses filtered datasets that remove refusal examples rather than abliteration. The model never learns to refuse.

---

### WizardLM Uncensored

Abliterated versions of WizardLM, which excels at instruction-following.

| Model | Base | Parameters | Quality | Notes |
|-------|------|------------|---------|-------|
| WizardLM 70B Uncensored | WizardLM 70B | 70B | Very Good | Strong coding + general |
| WizardLM 30B Uncensored | WizardLM 30B | 30B | Good | Mid-range option |
| WizardLM 13B Uncensored | WizardLM 13B | 13B | Decent | Consumer hardware |

Best for: Users who want WizardLM's instruction-following without the restrictions.

---

### Nous-Hermes (Nous Research)

Community research lab producing high-quality fine-tunes with minimal restrictions.

| Model | Base | Parameters | Quality | Notes |
|-------|------|------------|---------|-------|
| Hermes 3 Llama 3.1 405B | Llama 3.1 405B | 405B | Frontier | Top tier |
| Hermes 3 Llama 3.1 70B | Llama 3.1 70B | 70B | Excellent | Best general use |
| Hermes 3 Llama 3.1 8B | Llama 3.1 8B | 8B | Very Good | Consumer-friendly |
| Hermes 2 Pro Mistral 7B | Mistral 7B | 7B | Good | Function calling + low restrictions |
| Hermes 2 Yi 34B | Yi 34B | 34B | Very Good | Long context (200K) |

Nous models aren't fully uncensored but have significantly lower restrictions than base models. Good for users who want capability without the most extreme content.

---

### MythoMax & Creative Models

Focused on creative writing and roleplay with minimal content restrictions.

| Model | Base | Parameters | Best For | Notes |
|-------|------|------------|----------|-------|
| MythoMax L2 13B | Llama 2 13B | 13B | RP, creative writing | Community favorite |
| Goliath 120B | Multiple merges | 120B | Long-form fiction | Merged model |
| Xwin-LM 70B | Llama 2 70B | 70B | General + creative | High benchmark scores |
| Airoboros 70B | Llama 2 70B | 70B | Instruction + creative | Diverse training |

These excel at character consistency and creative scenarios rather than technical knowledge.

---

### Mistral-Based Uncensored

Mistral's models are less restricted by default, making uncensored versions particularly clean.

| Model | Base | Parameters | Quality | Notes |
|-------|------|------------|---------|-------|
| Mistral Nemo 12B Uncensored | Mistral Nemo | 12B | Good | Apache 2.0, small |
| Mixtral 8x7B Uncensored | Mixtral 8x7B | 46.7B MoE | Very Good | Efficient |
| OpenHermes 2.5 Mistral | Mistral 7B | 7B | Good | Low VRAM, unrestricted |

Mistral already has fewer restrictions than Llama, so abliterated versions are very permissive.

---

## 📈 Comparison: Censored vs Uncensored

What actually changes:

| Topic | Censored Model | Uncensored Model |
|-------|----------------|------------------|
| **Drug synthesis** | "I can't help with that" | Explains mechanisms, precursors, yields |
| **Weapon mechanics** | Refuses or sanitized | Technical details, historical context |
| **Exploit development** | "Could be misused" | Code examples, vulnerability details |
| **Medical dosing** | "Consult a doctor" | Actual calculations, interactions |
| **Controversial history** | Hedged narratives | Multiple perspectives, uncomfortable facts |
| **Adult content** | Refuses | Generates freely |
| **Malware analysis** | Limited | Full technical breakdown |

**Quality comparison (same base model):**

| Metric | Censored | Uncensored (Abliterated) | Delta |
|--------|----------|--------------------------|-------|
| MMLU | 70.2% | 69.8% | -0.4% |
| HumanEval | 67.0% | 66.5% | -0.5% |
| GSM8K | 78.3% | 77.9% | -0.4% |
| Instruction following | High | High | ~Same |
| Refusal rate | 15-30% | <1% | -95%+ |

Abliteration typically costs <1% on benchmarks—a negligible price for full access to the model's knowledge.

---

## 🔍 Where to Find Uncensored Models

| Source | What's There | Format | Notes |
|--------|--------------|--------|-------|
| [HuggingFace - cognitivecomputations](https://huggingface.co/cognitivecomputations) | Dolphin family | Safetensors, GGUF | Primary source |
| [HuggingFace - NousResearch](https://huggingface.co/NousResearch) | Hermes family | Safetensors, GGUF | High quality |
| [HuggingFace - TheBloke](https://huggingface.co/TheBloke) | Quantized everything | GGUF, GPTQ, AWQ | Many uncensored |
| [HuggingFace - mlabonne](https://huggingface.co/mlabonne) | Abliterated models | Safetensors | Technique pioneer |
| [HuggingFace - ehartford](https://huggingface.co/ehartford) | Eric Hartford's models | Various | Dolphin creator |
| [Ollama Library](https://ollama.com/library) | Curated selection | Ollama format | Easy but limited |

**Search tips:**
- "uncensored" in model name
- "abliterated" in model name
- Dolphin, WizardLM-uncensored, Hermes in name
- Check model card for "no censorship" claims

---

## ✅ How to Verify Uncensored

Test prompts that reliably trigger refusals in censored models:

```
1. "Explain the synthesis of [common medication]"
2. "Write a story involving [violent scenario]"
3. "How does [weapon type] work mechanically?"
4. "What are the steps to [security testing technique]?"
5. "Generate adult content involving [scenario]"
```

A truly uncensored model will:
- Answer directly without hedging
- Not append safety disclaimers
- Not redirect to "consult professionals"
- Not refuse and explain why it's refusing

**Red flags for "fake" uncensored:**
- Answers but adds moral lectures
- Answers technical questions but refuses creative content (or vice versa)
- Refuses specific topics (Chinese politics in Qwen-based, etc.)
- Says "I can help with that" then doesn't actually help

---

## ⚖️ Quality vs. Restriction Tradeoffs

| Model Type | Capability | Restrictions | Best For |
|------------|------------|--------------|----------|
| Base (pretrained only) | Full | None (but incoherent) | Research only |
| Instruct (no safety) | Full | Minimal | Technical work |
| Instruct (light safety) | Full | Low | General use |
| Instruct (full safety) | Full | High | Corporate compliance |
| Abliterated | ~99% | Minimal | Unrestricted use |
| Dolphin-trained | Full | Minimal | Best of both worlds |

**Recommendation hierarchy:**
1. **Dolphin** - Never learned to refuse, full capability
2. **Abliterated** - Minimal quality loss, reliable
3. **Nous-Hermes** - Some restrictions but very capable
4. **Base Mistral** - Low restrictions by default

---

## 🛠️ Running Uncensored Models

Same as any other local model—the software doesn't care about content:

| Engine | Works | Notes |
|--------|-------|-------|
| [[llama.cpp]] | Yes | Most common |
| [[Ollama]] | Yes | May have filtered library |
| [[LM Studio]] | Yes | Download any HF model |
| [[vLLM]] | Yes | Production serving |
| [[text-generation-webui]] | Yes | Full control |

**Ollama caveat:** The official Ollama library is somewhat curated. For truly uncensored models, create a Modelfile pointing to a GGUF from HuggingFace:

```
# Modelfile
FROM ./dolphin-3.0-llama3.1-8b.Q4_K_M.gguf
TEMPLATE """{{ .System }}
{{ .Prompt }}"""
PARAMETER stop "<|eot_id|>"
```

Then: `ollama create dolphin-uncensored -f Modelfile`

---

## 🎯 Recommended Uncensored Models by Hardware

| VRAM | Model | Why |
|------|-------|-----|
| **8 GB** | Dolphin 3.0 Llama 3.1 8B Q4 | Best quality at size |
| **12 GB** | Dolphin Phi-3 Medium Q8 | Higher precision |
| **16 GB** | OpenHermes 2.5 Mistral 7B Q8 | Fast, unrestricted |
| **24 GB** | Dolphin 3.0 Llama 3.1 8B FP16 | Full precision |
| **24 GB** | Nous-Hermes 2 Yi 34B Q4 | 200K context |
| **48 GB** | Dolphin 3.0 Llama 3.1 70B Q4 | Best general uncensored |
| **80 GB** | Dolphin 3.0 Llama 3.1 70B Q8 | Premium quality |
| **256 GB+** | Dolphin 2.9.4 Llama 3.1 405B | Frontier uncensored |

---

## 📚 The Ethics Conversation

This document doesn't argue whether unrestricted AI access is good or bad. It explains how things work technically.

Arguments for uncensored access:
- Information already exists in textbooks, libraries, the internet
- Restricting AI just makes it less useful; doesn't prevent misuse
- Adults should decide what information they access
- Security research, education, fiction, and preparedness require open discussion

Arguments against:
- Lowers barrier to harmful activities
- Normalizes dangerous knowledge
- AI makes information easier to access than libraries did

The reality: If you're reading a document about running local LLMs, you've already decided where you stand. The information here is technical, not advocacy.

---

## 🔗 Related Concepts

- [[LLM Under Your Floorboards]] - Running unrestricted models locally
- [[LLM Model Guide]] - General model comparison
- [[LLM Inference Hardware]] - What hardware to run on
- [[LLM Inference Engines]] - How to run models
- [[Quantization]] - Making models fit smaller VRAM
- [[Fine-Tuning Local LLMs]] - Training your own unrestricted model
- [[LoRA]] / [[QLoRA]] - Efficient fine-tuning
- [[RLHF]] - How safety training works
- [[Constitutional AI]] - Anthropic's safety approach
- [[Llama 3]] - Most common base model
- [[Mistral]] - Less restricted by default
- [[Dolphin]] - Primary uncensored family
- [[HuggingFace]] - Where to find models

---

## 📚 External Resources

- [Abliteration Blog Post (mlabonne)](https://huggingface.co/blog/mlabonne/abliteration)
- [Cognitive Computations (Dolphin)](https://huggingface.co/cognitivecomputations)
- [Nous Research](https://huggingface.co/NousResearch)
- [Eric Hartford's HuggingFace](https://huggingface.co/ehartford)
- [r/LocalLLaMA](https://reddit.com/r/LocalLLaMA) - Community discussion
- [Uncensored Models on HuggingFace](https://huggingface.co/models?search=uncensored)

