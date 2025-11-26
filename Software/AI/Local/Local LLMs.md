# Local LLMs (for coding)

Local Large Language Models (LLMs) are increasingly viable for coding tasks — generating, completing, refactoring, or explaining code — on on‑premise hardware. With modern quantization and inference toolkits, many models that were once cloud‑only can now run on consumer or prosumer GPUs. This note surveys the main options, their hardware requirements, performance tradeoffs, and how they compare to state‑of‑the‑art closed models for coding tasks.

---

## 🔎 Overview

Running LLMs locally gives you privacy, offline capability, and control over latency and cost. For coding work — where you may feed in private repositories, test snippets, or iterate interactively — local LLMs are especially attractive.  

Recent advances (better quantization — Q4, Q5, Q6; improved backends like `llama.cpp`, `Ollama`, `LM Studio`, `KoboldCpp`) and well‑tuned code‑specialized LLMs (e.g., CodeLlama, DeepSeek‑Coder, StarCoder2, and newer community models) make this feasible without needing a datacenter-grade cluster.

But there are tradeoffs: quality vs hardware demand, context window size, quantization effects, and inference speed.  

In the sections below you’ll find a breakdown of the most relevant models, what hardware you need, how they perform on coding benchmarks, and how they compare to closed-source giants like ChatGPT / Claude.

---

## 🧠 Core Concepts & Requirements

- **Model size (parameters):** Larger models (e.g., 34B, 33B) generally yield better code quality or reasoning, but require more VRAM / RAM.  
- **Quantization:** Reducing model precision (e.g., Q4_K_M, Q5_K_M, Q8_0) dramatically reduces memory footprint and VRAM load, at the cost of some quality. Quantization enables many to run on consumer GPUs.
- **Context window / KV cache:** Long context allows the model to understand larger codebases or multiple files; but longer windows increase memory use.  
- **Inference backend:** Tools like `llama.cpp`, `Ollama`, `LM Studio`, `KoboldCpp`, `vLLM`, etc., make local inference possible. Some support GPU acceleration (CUDA / Vulkan / ROCm), multi-GPU, or CPU fallback.
- **Hardware bottlenecks:** VRAM size is usually the limiting factor. When a model doesn’t fit, backends may spill weights/KV cache to system RAM — but that hurts performance severely.
- **Benchmark metrics:** For coding, common metrics are pass@1 on benchmarks like [[HumanEval]], [[MBPP]], DS-1000 (data science), MultiPL‑E (multilingual), or DevEval (real-world code completion/infilling).

---

## 📊 Models Comparison — Key Coding LLMs for Local Use

| Model (size / variant) | Typical VRAM (quantized) | Inference Speed* (on a ~12–24 GB GPU) | Coding Benchmarks (pass@1 / comments) | Strengths / Notes |
|------------------------|--------------------------|----------------------------------------|----------------------------------------|------------------|
| **CodeLlama 7B / 13B / 34B** | 7B: ~6–8 GB<br>13B: ~9–12 GB<br>34B: ~16–24 GB (Q4_K_M / Q5_K_M) | 7B: ~70–90 toks/s<br>13B: ~40–55 toks/s<br>34B: ~15–25 toks/s (quantized) | ~50–70% (varies) on HumanEval depending on size / tuning | Official code‑specialized LLM from Meta; good balance of performance and VRAM. 13B often sweet‑spot for typical dev rigs. |
| **DeepSeek-Coder 6.7B / 33B** | 6.7B: ~8–10 GB<br>33B: ~24–32 GB+ | 6.7B: comparable to CodeLlama 7B<br>33B: slower, but usable on 24 GB+ GPU | 6.7B: ~64% HumanEval; 33B: ~73–74% on HumanEval, MBPP, DS-1000 | Among best open-source code LLMs; 33B often outperforms CodeLlama-34B. Strong multilingual + data-science code support. |
| **StarCoder2 3B / 7B / 15B** | 3B: ~4–5 GB<br>7B: ~6–8 GB<br>15B: ~12–16 GB (quantized) | 7B: ~60–80 toks/s<br>15B: ~25–40 toks/s | 3B: best among small models; 7B: medium-class strong performer; 15B: matches or exceeds CodeLlama 34B on some benchmarks | Very efficient for smaller GPUs; good for quick completions or chat-based coding assistance. Great open-source project (BigCode). |
| **(Emerging) Olmo3 7B** | Likely similar to other 7B models (≈ 6–8 GB) — community reports of local runs. | Not widely benchmarked yet, but expected similar to StarCoder2‑7B or CodeLlama-7B | No public large-scale published coding benchmark (as of 2025) | Newer architecture; some community interest. Might be worth watching, especially for efficient inference. |

\* Speeds are approximate and depend heavily on context window, quantization, GPU, backend.

---

## ✅ Use Cases — Where Local Coding LLMs Shine

- **Interactive coding assistance:** autocomplete, inline docstring generation, quick function stubs, refactoring help.  
- **Privacy-sensitive codebases:** local inference ensures your proprietary code never leaves your machine.  
- **Rapid prototyping / small scripting:** quick generation of small scripts, data analysis notebooks, automation tooling.  
- **Offline development setup / remote environments:** Linux server running LLM locally, no internet needed.  
- **Long-context code reasoning:** 13B–33B models with 16K+ context windows can handle multi-file contexts or long files.  

---

## 🏆 Strengths & 🛠️ Weaknesses (Pros & Cons)

### ✅ Strengths / Pros

- Full control over data — no external API, no privacy concerns.  
- No per‑query cost (after GPU purchase).  
- Good real-world performance: several open models now rival older closed models (e.g., DeepSeek‑Coder-33B vs GPT-3.5).
- Flexibility: can fine‑tune, quantize, host on your own server, or integrate into local tools.  
- Versatility: many models support multiple languages, long contexts, code explanation, infilling, and more.  

### ❌ Weaknesses / Cons

- **Hardware constraints:** VRAM is always the limiting factor. Large models (30B+) may require 24 GB+ and often 32–48 GB VRAM, or multi‑GPU setups.
- **Speed/performance tradeoffs:** when models don’t fully fit in VRAM, inference slows drastically (spill to RAM).
- **Quality gap vs closed SOTA:** while top open models are strong, they still lag behind cutting-edge closed models in reasoning, long-horizon planning, very complex code tasks, or code bases requiring deep project-level context.  
- **Licensing / commercial restrictions:** some open models have non-commercial or restrictive licenses.  
- **Setup complexity:** tooling (quantization, model conversion, backend config) can be nontrivial if you want optimal performance.  

---

## 🔄 How They Compare to Closed SOTA (ChatGPT, Claude, etc.)

- Closed models (e.g., GPT-4 / GPT-4o, Claude Opus) still generally excel at complex reasoning, sophisticated code generation across multiple files/modules, and tasks requiring real-world context (e.g., integrating with APIs, architecture-level design, natural language requirements).  
- Among open models, **DeepSeek-Coder-33B** now approaches GPT-3.5-level performance on many coding benchmarks.
- Smaller models (7B to 15B), like StarCoder2 or CodeLlama, provide a smoother, more responsive local experience — ideal for quick tasks, refactoring, autocompletes, or when working on limited VRAM hardware.  
- For many everyday coding tasks (scripts, data pipelines, tool automation), a 13B model may provide “good enough” output, especially with good prompting and human review.  

In short: local LLMs are very competitive for many developer workflows — but for large-scale software architecture or advanced reasoning, closed cloud models still often win.

---

## 🖥️ Running Local LLMs — Setup & Hardware Guidance

### 🔧 Summary Hardware/Software Requirements

- **Minimum plausible setup (for small models):** 8–10 GB VRAM (e.g., consumer GPU like RTX 3060, 4060) + 16–32 GB system RAM. Enough for 3B–7B models (with quantization).
- **Recommended for 13B–15B:** ~12–16 GB VRAM (e.g., 3060 Ti, 4070 / 4070 Ti / 4070 Ti Super), 32–64 GB RAM.
- **For 30B–34B models:** 24–32 GB VRAM (e.g., 3090, 4090, A6000), 64+ GB RAM, NVMe SSD, fast CPU (8+ cores).
- **For 70B+ or multi-file heavy workloads:** workstation or small server-grade GPU(s) (e.g., dual H100, multi‑GPU setup) — or use multi-GPU / offloading/backing strategies.

### 💻 Tools & Backends Frequently Used

- **LM Studio:** GUI tool that supports many GGML-based or quantized LLMs (LLaMA, CodeLlama, StarCoder, MPT, etc.). Works with NVIDIA / AMD GPUs. For small GPUs (6–8 GB VRAM), entry-level models; for mid-range GPUs (8–12 GB), 7–14B models; for 24 GB+, heavier models.
- **Ollama:** CLI / server-based system for local hosting + inference + API, good for building coding assistants or integrating with IDEs. Supports GPU (CUDA) or CPU fallback.
- **llama.cpp / KoboldCpp / vLLM / other backends:** For advanced users who want minimal overhead, custom quantization, multi-GPU, or optimized performance.

### 🚀 Typical Workflow (Example)

1. Pick model (e.g., CodeLlama-13B-instruct) → quantize with Q4_K_M (or download community GGUF)  
2. Load in LM Studio or Ollama with GPU acceleration enabled  
3. Optionally expose a local API (useful for IDE integration, code generation scripts, agents)
4. For larger models that don’t fit fully, configure offloading (to RAM or CPU), but expect slower token-per-second speeds.  

---

## 📚 Coding LLMs — Use Cases, Strengths & Weaknesses by Model

### **CodeLlama**

- ✅ Great for developers who want a “set-and-forget” code model that works out of the box.  
- ✅ 13B is a good sweet-spot on many dev rigs; 7B is snappy for quick completions.  
- ⚠️ For very large codebases, 34B may be needed — but that requires beefy hardware.  
- ⚠️ Performance is decent but lags behind the top open-code models (e.g., DeepSeek) on hardest tasks.  

### **DeepSeek-Coder**

- ✅ Top open-source model for code as of 2025. 33B version often outperforms CodeLlama-34B.
- ✅ Good for data science code, library-heavy tasks, multilingual code.  
- ✅ 6.7B variant is useful if hardware is limited — reasonable performance with modest VRAM.  
- ⚠️ 33B requires stronger GPU + more RAM. Also, quantization/performance tradeoffs if context is large.  

### **StarCoder2**

- ✅ Ideal for smaller GPUs or quick, frequent coding completions.  
- ✅ In the 15B class, it punches above weight — comparable to 34B CodeLlama in many cases.
- ✅ Good open-source license (BigCode), wide language coverage, good for learning, prototyping, or scripting tasks.  
- ⚠️ For heavy-duty project-level coding or very long context, may be outclassed by larger models.  

### **Emerging / Experimental (e.g. Olmo3)**

- ℹ️ Promising newer architectures, potentially more efficient. Community interest is rising.
- ⚠️ Not yet widely benchmarked for coding tasks — uncertain quality/stability compared to mature code LLMs.  

---

## 🔗 Related Concepts / Other Notes

- [[HumanEval]] (benchmark for Python code generation)  
- [[MBPP]] (Another popular code generation benchmark)  
- [[StarCoder2]]  
- [[CodeLlama]]  
- [[DeepSeek-Coder]]  
- [[llama.cpp]] / [[Ollama]] / [[LM Studio]] (tools for local LLM inference)  
- [[Quantization]] (Q4, Q5, Q6, etc.) — critical for running models locally
- [[Olmo3]]

---

## 🏗️ What This Means for Developer Workflows

- For many day‑to-day tasks — small scripts, prototyping, boilerplate generation, refactoring — a local 7B or 13B model (CodeLlama, StarCoder2, or DeepSeek‑Coder) is often “good enough,” with the benefit of instant, private, offline access.  
- If you work on larger codebases, data‑science pipelines, or cross‑file refactoring and want higher correctness, opting for a 30B+ model may pay off — provided you have the hardware.  
- Local LLMs are especially compelling when integrated into your dev environment: IDE plugins, local API servers, automation tools, etc., giving near‑instant completion/refactor cycles without hitting cloud APIs.  

---

## 📖 Further Reading / Resources

- Running LLMs locally — general guide and setup instructions.
- LM Studio GPU / VRAM requirements for different model sizes.
- DeepSeek-Coder performance and benchmarks.
- StarCoder2 paper and benchmark results.
- Community discussion and experience around new models like Olmo 3.

