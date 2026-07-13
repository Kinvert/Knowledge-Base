# Qwen3.6-35B-A3B for Local Coding Agents

`Qwen3.6-35B-A3B` is a sparse MoE model in the Qwen3.6 family with only about `3B` active parameters per token, but a larger total parameter pool. That makes it attractive for agentic coding when your hardware and serving stack can handle MoE routing overhead.

---

## Why this matters for coding agents

- Better long-form agent reasoning than many dense peers on public coding and tool-use benchmarks.
- `A3B` active routing can reduce active compute per token while keeping high parameter capacity.
- Strong fit for repository-level refactors, architecture planning, and task decomposition when latency variability is acceptable.

---

## Core spec (important for local planning)

| Property | Value |
|---|---|
| Model ID | `Qwen/Qwen3.6-35B-A3B` |
| Family | Qwen3.6 |
| Type | Causal LM with Vision Encoder |
| Total params | `35B` |
| Active params | `3B` (MoE active) |
| Experts | `256` total (8 routed + 1 shared) |
| Layers | `40` |
| Hidden size | `2048` |
| Context window | `262,144` native, +1,010,000 with long-context extension |

Source: [HF model card](https://huggingface.co/Qwen/Qwen3.6-35B-A3B).

---

## Local-relevant benchmark behavior

From the published language table:

| Benchmark | 35B-A3B score | 27B score | Commentary |
|---|---:|---:|---|
| SWE-bench Verified | `75.0` | `77.2` | Slightly lower absolute, still very strong |
| SWE-bench Pro | `51.2` | `53.5` | Similar planning strength, narrower gap |
| Terminal-Bench 2.0 | `51.5` | `59.3` | Dense variant can be ahead here |
| Claw-Eval Avg | `68.7` | `72.4` | 27B currently leads in this slice |
| QwenClawBench | `52.6` | `53.4` | Very close |
| SkillsBench Avg5 | `28.7` | `48.2` | 27B lead on that benchmark entry |

The point is not always raw score domination; for local deployment you should weight:
- routing stability,
- tool schema fidelity,
- and whether your stack likes MoE latency variance.

---

## Local deployment (OpenAI-compatible endpoint)

The Qwen3.6 model cards show vLLM + SGLang examples around `qwen3` reasoning parser and 262k context.

### SGLang

```bash
python -m sglang.launch_server \
  --model-path Qwen/Qwen3.6-35B-A3B \
  --port 8000 \
  --tp-size 8 \
  --mem-fraction-static 0.8 \
  --context-length 262144 \
  --reasoning-parser qwen3 \
  --tool-call-parser qwen3_coder
```

### vLLM

```bash
vllm serve Qwen/Qwen3.6-35B-A3B \
  --port 8000 \
  --tensor-parallel-size 8 \
  --max-model-len 262144 \
  --reasoning-parser qwen3 \
  --enable-auto-tool-choice \
  --tool-call-parser qwen3_coder
```

### Practical local adjustment when memory is not enough

- Use less context length than `262k`.
- For text-only coding loops, drop vision with `--language-model-only` where supported.
- Keep one-model-per-worker for first-pass reliability, then add routing/parallelism once you can reproduce tool-call behavior.

```bash
vllm serve Qwen/Qwen3.6-35B-A3B \
  --port 8000 \
  --tensor-parallel-size 8 \
  --max-model-len 131072 \
  --reasoning-parser qwen3 \
  --language-model-only
```

---

## Why 35B-A3B is harder than 27B locally

### Better for
- deeper architecture reasoning,
- complex multi-file changes,
- agentic workflows with long plan horizons.

### Harder because
- MoE routing adds latency variance,
- tool-call consistency is more sensitive to server/harness settings,
- memory and compile/runtime settings are less forgiving.

Use this model when your project rewards planning quality over raw patch latency.

---

## Local coding-agent control loop

Use strict JSON action contracts; if your wrapper does not validate, MoE outputs can look “smart” but execute risky actions.

```json
{
  "step": "plan|read|patch|run|ask_human|handoff",
  "goal": "complete a repository change request",
  "ops": [
    {
      "type": "patch",
      "path": "src/agent.py",
      "patch": "--- old\n+++ new\n@@"
    }
  ],
  "checks": ["pytest -q tests/unit", "python -m mypy src"],
  "confidence": 0.86,
  "ask_before_execute": true
}
```

### Recommended loop stack
- [Qwen Code](https://qwenlm.github.io/qwen-code-docs/) (when you want a local terminal-native agent UX),
- [Aider](https://aider.chat/) (surgical patch workflow),
- custom Python loop when you need hard safety gates and custom telemetry.

---

## Deployment sanity checklist

- `vllm`/`sglang` startup succeeds with OpenAI-compatible `/v1` routes.
- tool parser returns valid JSON action payloads (never raw prose).
- schema validator catches:
  - unknown actions
  - path traversal (`../`)
  - unsupported shell commands
- run at least 5 deterministic cycles before scaling concurrency.

---

## Related notes

- [[Qwen3.6-27B]]
- [[LLM Inference Engines]]

## External references

- [Qwen3.6-35B-A3B HF](https://huggingface.co/Qwen/Qwen3.6-35B-A3B)
- [Qwen3.6 repository](https://github.com/QwenLM/Qwen3.6)
- [vLLM docs](https://docs.vllm.ai/)
- [SGLang docs](https://docs.sglang.ai/)
- [Qwen Code docs](https://qwenlm.github.io/qwen-code-docs/)
