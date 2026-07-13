# Qwen3.6-27B for Local Coding Agents

`Qwen3.6-27B` is the dense member of the Qwen3.6 family and a practical open-weight choice for local coding agents because it avoids MoE routing variance and still targets long-context coding tasks.

---

## Why this one for local coding agents

- Dense model (`27B` active params) with stable behavior under agentic loops.
- Built for multimodal coding workflows (vision + text), but you can run text-only mode for cheaper local inference.
- Strong local agent benchmark posture versus older 27B/35B generation baselines.

See [[LLM Under Your Floorboards]] for hardware strategy and cost tradeoffs.

---

## Core spec (what matters for local agents)

| Property | Value |
|---|---|
| Model ID | `Qwen/Qwen3.6-27B` |
| Family | Qwen3.6 |
| Type | Causal LM with Vision Encoder |
| Parameters | `27B` (fully dense) |
| Layers | `64` |
| Hidden size | `5120` |
| Experts | no MoE (dense) |
| Context window | `262,144` native, and `1,010,000` with YaRN-style extension |
| Activation style | token-by-token autoregressive |

Sources: [HF model card](https://huggingface.co/Qwen/Qwen3.6-27B) and [Qwen3.6 repo](https://github.com/QwenLM/Qwen3.6/blob/main/README.md).

---

## Coding-relevant performance profile

From the model card (language benchmarks), Qwen3.6-27B is strongest on coding-heavy metrics among many dense baselines and close to the strongest coding baselines in its table at long context:

| Benchmark | 27B score |
|---|---:|
| SWE-bench Verified | `77.2` |
| SWE-bench Pro | `53.5` |
| SWE-bench Multilingual | `71.3` |
| Terminal-Bench 2.0 | `59.3` |
| Claw-Eval Avg | `72.4` |
| QwenClawBench | `53.4` |
| SkillsBench Avg5 | `48.2` |

The table notes that SWE-bench style runs use around `200k` context and explicit tool scaffolding in evaluation.

---

## Local serving (OpenAI-compatible endpoint)

The docs recommend SGLang/vLLM-style serving with `qwen3` reasoning parser and `qwen3_coder` tool parser for coding loops.

### SGLang (closest to tested examples)

```bash
python -m sglang.launch_server \
  --model-path Qwen/Qwen3.6-27B \
  --port 8000 \
  --tp-size 8 \
  --mem-fraction-static 0.8 \
  --context-length 262144 \
  --reasoning-parser qwen3 \
  --tool-call-parser qwen3_coder
```

### vLLM

```bash
vllm serve Qwen/Qwen3.6-27B \
  --port 8000 \
  --tensor-parallel-size 8 \
  --max-model-len 262144 \
  --reasoning-parser qwen3 \
  --enable-auto-tool-choice \
  --tool-call-parser qwen3_coder
```

### Lower-memory/local-tuned variant

- `262k` context is ideal, but if memory is tight, reduce context length and run text-only mode:

```bash
vllm serve Qwen/Qwen3.6-27B \
  --port 8000 \
  --tensor-parallel-size 8 \
  --max-model-len 131072 \
  --reasoning-parser qwen3 \
  --language-model-only
```

- The model card explicitly recommends reducing context on OOM and says the large-task behavior needs context to stay useful.

---

## Agent workflow that actually works in practice

### 1) Run one OpenAI-compatible server

Expose `/v1/chat/completions` from SGLang or vLLM and use a local controller.

### 2) Enforce strict JSON action schema

Example action object for tool calls:

```json
{
  "action": "read|edit|patch|command|ask|done",
  "rationale": "why this action is required",
  "path": "repo/path/file.py",
  "patch": "--- a.py\n+++ b.py\n@@\n+...",
  "command": "pytest -q",
  "risk": "low|medium|high"
}
```

### 3) Gate execution

- Validate every tool call against:
  - action enum
  - repository path allowlist
  - max patch size
  - blocked commands (`rm`, `sudo`, network by default off)
- Reject malformed JSON and ask for explicit retry.

---

## Agent wrapper options for this model

- [Qwen Code](https://qwenlm.github.io/qwen-code-docs/) with local backend; has approval modes and tool integration.
- [Aider](https://aider.chat/) (`openai` compatible) for focused diff-driven coding loops.
- [OpenHands](https://docs.openhands.dev/) when you need terminal + web + file + patch in one loop.

Minimal command pattern (examples are endpoint-agnostic):

```bash
export OPENAI_API_BASE=http://127.0.0.1:8000/v1
export OPENAI_API_KEY=dummy
```

Then wire your harness to `Qwen/Qwen3.6-27B`.

---

## What to expect when working locally

- Stronger long-context consistency than smaller 14B coding models.
- Higher latency and memory than dense 14B class; budget for retrieval and context pruning.
- Best results come from:
  - short, explicit tasks
  - strong static analyzers/tests
  - deterministic sampling (`temperature 0`)

---

## Related notes

- [[Qwen3.6-35B-A3B]]
- [[LLM Inference Engines]]

## External references

- [Qwen3.6-27B HF](https://huggingface.co/Qwen/Qwen3.6-27B)
- [Qwen3.6 repository](https://github.com/QwenLM/Qwen3.6)
- [vLLM OpenAI-compatible server](https://docs.vllm.ai/)
- [SGLang project docs](https://docs.sglang.ai/)
- [Qwen Code docs](https://qwenlm.github.io/qwen-code-docs/)
