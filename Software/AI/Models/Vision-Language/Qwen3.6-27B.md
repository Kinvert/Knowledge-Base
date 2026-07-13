# Qwen3.6-27B

Use this model when you want a **dense, local coding agent** with strong instruction following and multimodal context. It is practical for tool loops when you are willing to allocate significant VRAM and need long-context coding memory.

---

## Why this model for local coding agents

- Works with local tool loops that require long context (up to 262k tokens) across many file diffs.
- Supports structured tool payloads with Qwen’s `qwen3_coder` style parser path.
- Good when you want both reasoning-style plans and deterministic JSON behavior in one model.
- Better than older pure-chat models if your workload includes screenshots, diagrams, or diff screenshots.

Official sources:
- https://github.com/QwenLM/Qwen3.6/blob/main/README.md
- https://huggingface.co/Qwen/Qwen3.6-27B
- https://huggingface.co/Qwen/Qwen3.6-27B/blob/main/config.json

---

## What this model is

- Model family: Qwen3.6 dense multimodal (text + image token stream).
- Release date: April 22, 2026.
- Parameter count: listed as ~28B in HF metadata.
- Context window: `max_position_embeddings = 262144`.
- Not MoE; this is a dense architecture.
- `config.json` fields shown in HF include:
  - `num_hidden_layers: 64`
  - `num_attention_heads: 24`
  - `num_key_value_heads: 4`
  - vision stack with `depth: 27`, `hidden_size: 1152`, `out_hidden_size: 5120`

---

## Local coding-agent architecture pattern

Use a strict schema gate between model and execution layer.

1. High-level planner asks for an action.
2. Model returns JSON with exactly one action object.
3. Host validates schema.
4. Controller executes shell/Git/robot interface.
5. Result is fed back as compact state.

This is the minimum safe pattern for any local coding agent.

---

## Server setup for local use

### Minimal vLLM serving (recommended for tooling)

```bash
export MODEL=Qwen/Qwen3.6-27B

vllm serve "$MODEL" \
  --host 0.0.0.0 --port 8000 \
  --tensor-parallel-size 4 \
  --max-model-len 262144 \
  --reasoning-parser qwen3 \
  --tool-call-parser qwen3_coder \
  --enable-auto-tool-choice
```

### Pure coding (text-only, faster than multimodal)

```bash
vllm serve "$MODEL" \
  --host 0.0.0.0 --port 8000 \
  --tensor-parallel-size 4 \
  --max-model-len 262144 \
  --reasoning-parser qwen3 \
  --language-model-only \
  --enable-auto-tool-choice \
  --tool-call-parser qwen3_coder
```

### SGLang equivalent

```bash
python -m sglang.launch_server \
  --model-path Qwen/Qwen3.6-27B \
  --port 8000 \
  --tp-size 4 \
  --context-length 262144 \
  --reasoning-parser qwen3 \
  --tool-call-parser qwen3_coder
```

### Faster generation (if backend supports it)

```bash
vllm serve "$MODEL" \
  --tensor-parallel-size 4 \
  --max-model-len 262144 \
  --reasoning-parser qwen3 \
  --speculative-config '{"method":"qwen3_next_mtp","num_speculative_tokens":2}'
```

---

## Coding loop template (strict JSON)

```python
import json
from pydantic import BaseModel, ValidationError
from openai import OpenAI

class Action(BaseModel):
    op: str
    files: list[str]
    rationale: str
    command: str | None = None

client = OpenAI(base_url="http://localhost:8000/v1", api_key="EMPTY")

resp = client.chat.completions.create(
    model="Qwen/Qwen3.6-27B",
    messages=[
        {
            "role": "system",
            "content": (
                "Return valid JSON only. Schema: "
                '{"op","files","rationale","command"}'
            ),
        },
        {"role": "user", "content": "Add null-check to parser.py and add tests."},
    ],
    max_tokens=2048,
    temperature=0.2,
    extra_body={"chat_template_kwargs": {"enable_thinking": False, "preserve_thinking": False}},
)

raw = resp.choices[0].message.content
data = json.loads(raw)
try:
    action = Action.model_validate(data)
except ValidationError as err:
    raise RuntimeError(f"Bad schema: {err}")
```

---

## Practical limits

- Full BF16 usually needs 2+ GPUs unless using heavier quantization (Q4/Q5/AWQ/GGUF-style).
- Long context increases KV memory fast; for interactive coding loops keep effective context tighter unless needed.
- Use tool mode only if outputs are schema-validated.
- Turn off multimodal and reduce `--max-model-len` for local responsiveness.

---

## Compare vs local alternatives

| Model | Params | Context | Why choose for coding agent |
|---|---:|---:|---|
| Qwen3.6-27B | ~28B dense | 262k | Best balance for multimodal local coding |
| Qwen2.5-Coder-32B | 32B dense | 131k | Good coding quality, no official long context at this scale |
| CodeLlama-34B-Instruct | 34B dense | 16k (base), lower extensions | Good open baseline |
| DeepSeek-Coder-V2 | MoE | 16k+ (varies) | Strong code and tool support, older ecosystem |
| Llama 3.1-70B Instruct | 70B dense | 128k | Strong chat + broad tool ecosystem |
| Qwen3.6-35B-A3B | 35B MoE | 262k+ | Bigger, pricier, stronger planning |

---

## Strengths / limits

- Strengths:
  - strong coding and reasoning for structured tasks;
  - long context for planning across many files;
  - workable tool-call path for local agents.
- Limits:
  - high VRAM and latency at high context;
  - structured output is only as good as your schema gate and retries;
  - large prompts can cause occasional drift.

---

## Related notes

- [[Qwen3.6-35B-A3B]]
- [[Qwen3.6-27B]] *(this note)*
- [[vLLM]]
- [[SGLang]]
- [[Local LLMs]]
- [[Qwen-Agent]]
- [[Qwen Code]]

---

## External links

- https://github.com/QwenLM/Qwen3.6/blob/main/README.md
- https://huggingface.co/Qwen/Qwen3.6-27B
- https://huggingface.co/Qwen/Qwen3.6-27B/blob/main/config.json
- https://huggingface.co/Qwen/Qwen3.6-27B-FP8
- https://github.com/QwenLM/Qwen-Agent
- https://github.com/QwenLM/qwen-code
