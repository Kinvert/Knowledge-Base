# Qwen3.6-35B-A3B

This is the MoE sibling to Qwen3.6-27B and the one to reach for when local coding-agent workloads need stronger planning depth and context-aware control than dense models can give at this size.

---

## Why this model for local coding agents

- Better instruction-following and patch-planning depth for complex workflows.
- Very long context behavior is the key differentiator for monorepo-level refactor tasks.
- Supports the same `qwen3_coder` tool-call path expected by schema-locked action loops.
- Good for multi-step control tasks where local agent must inspect many files and produce staged commands.

Official sources:
- https://github.com/QwenLM/Qwen3.6/blob/main/README.md
- https://huggingface.co/Qwen/Qwen3.6-35B-A3B
- https://huggingface.co/Qwen/Qwen3.6-35B-A3B/blob/main/config.json

---

## What this model is

- Architecture: sparse MoE (`qwen3_5_moe_text`).
- MoE details from HF config:
  - `num_experts: 256`
  - `num_experts_per_tok: 8`
  - `num_hidden_layers: 40`
  - `num_attention_heads: 16`
  - `num_key_value_heads: 2`
- Release date: April 16, 2026.
- Parameter count shown in HF metadata: around 36B class.
- Context:
  - default window: `262144`
  - model documentation also exposes long-context scaling behavior up to around 1,010,000 tokens.
- Useful for local coding agents only when you can afford high GPU count or aggressive quantization.

---

## Local deployment trade-off

This model is expensive locally:
- Dense-equivalent memory use is much lower than 35B dense because only a subset of experts are active, but still requires substantial VRAM for weights + kv-cache + beam width.
- For many agents it runs best at higher tensor parallelism than 27B.
- In practical coding loops, use text-only mode unless vision materially improves your control output quality.

---

## Local serving recipes

### vLLM strong baseline

```bash
export MODEL=Qwen/Qwen3.6-35B-A3B

vllm serve "$MODEL" \
  --host 0.0.0.0 --port 8000 \
  --tensor-parallel-size 8 \
  --max-model-len 262144 \
  --reasoning-parser qwen3 \
  --tool-call-parser qwen3_coder \
  --enable-auto-tool-choice
```

### Text-only coding mode

```bash
vllm serve "$MODEL" \
  --tensor-parallel-size 8 \
  --max-model-len 262144 \
  --reasoning-parser qwen3 \
  --language-model-only \
  --tool-call-parser qwen3_coder \
  --enable-auto-tool-choice
```

### SGLang equivalent

```bash
python -m sglang.launch_server \
  --model-path Qwen/Qwen3.6-35B-A3B \
  --port 8000 \
  --tp-size 8 \
  --context-length 262144 \
  --reasoning-parser qwen3 \
  --tool-call-parser qwen3_coder
```

### MTP/Speculative decode

```bash
vllm serve "$MODEL" \
  --tensor-parallel-size 8 \
  --max-model-len 262144 \
  --reasoning-parser qwen3 \
  --speculative-config '{"method":"qwen3_next_mtp","num_speculative_tokens":2}'
```

---

## Coding-agent loop example (JSON + validator)

```python
from pydantic import BaseModel
from openai import OpenAI
import json

class Plan(BaseModel):
    objective: str
    steps: list[str]
    risk: str

client = OpenAI(base_url="http://localhost:8000/v1", api_key="EMPTY")

out = client.chat.completions.create(
    model="Qwen/Qwen3.6-35B-A3B",
    messages=[
        {"role": "system", "content": "You are a coding agent. Return only JSON."},
        {"role": "user", "content": "Create a rollback-safe patch plan for the navigation planner."},
    ],
    temperature=0.1,
    max_tokens=1800,
    extra_body={"chat_template_kwargs": {"enable_thinking": False, "preserve_thinking": False}},
)

payload = Plan.model_validate_json(out.choices[0].message.content)
print(payload.steps)
```

---

## When to choose 35B instead of 27B

Use 35B-A3B if:
- You need better long-horizon planning across many files.
- You want stronger multimodal coding context retention.
- You can tolerate slower generation and higher cost.

Use 27B if:
- You need tighter latency.
- You run on fewer GPUs.
- You are mostly in text-only coding with simpler planning graphs.

---

## Comparison chart for coding use

| Model | Architecture | Context | Strength for coding agent |
|---|---|---:|---|
| Qwen3.6-35B-A3B | MoE (8 active / 256) | 262k (to ~1,010k) | Best for hard planning + long history |
| Qwen3.6-27B | Dense | 262k | Strong baseline, cheaper to run |
| Qwen3.6-VL variants (other sizes) | Dense/MoE family | 262k+ | Tradeoff by size, same tooling |
| Qwen2.5-Coder-32B | Dense | 131k | Good coder baseline, easier to run |
| DeepSeek-Coder-V2 | MoE | 16k+ | Strong legacy coder model |
| Llama 3.3 70B Instruct | Dense | 128k | Very capable but expensive for 100% local |

---

## Related notes

- [[Qwen3.6-27B]]
- [[Qwen-Agent]]
- [[Qwen Code]]
- [[vLLM]]
- [[SGLang]]
- [[Local LLMs]]

---

## External links

- https://github.com/QwenLM/Qwen3.6/blob/main/README.md
- https://huggingface.co/Qwen/Qwen3.6-35B-A3B
- https://huggingface.co/Qwen/Qwen3.6-35B-A3B/blob/main/config.json
- https://huggingface.co/Qwen/Qwen3.6-35B-A3B-FP8
- https://github.com/QwenLM/Qwen-Agent
- https://github.com/QwenLM/qwen-code
