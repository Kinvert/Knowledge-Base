# 14B Local Coding Agents on an RTX 5090

You want a local stack that behaves like a coding agent: read files, edit files, run shell commands, browse web references, and act on the results. This note is the practical setup for a single RTX 5090 box with ~14B models.

Linked from [[LLM Under Your Floorboards]].

---

## 1) Reality check for a 5090 setup

An RTX 5090 is a good fit for single-GPU 14B-class workflows because it has **32 GB of GDDR7** and Blackwell-class SM/Tensor resources [[NVIDIA RTX 5090 Specs](https://www.nvidia.com/en-us/geforce/graphics-cards/50-series/rtx-5090/)].

A ~14B model at full precision is usually too large for comfortable single-device use (weights + KV cache + context). On 5090 you can run 14B class models reliably with quantization and memory-aware config.

Practical mindset:
- Use a **strong 14B baseline** (or one 12–16B in size) for editing speed.
- Keep a smaller model for routine refactors/tests and a heavier reasoning model for deep debugging.
- Keep all agent sessions restricted to project roots and explicit allowlists.

---

## 2) Model shortlist (what to actually install)

| Model | Source / Parameters | Why you care for coding agents | Use this if | Rough notes |
|---|---|---|---|---|
| `Qwen/Qwen2.5-Coder-14B-Instruct` | 14.7B [[HF model card](https://huggingface.co/Qwen/Qwen2.5-Coder-14B-Instruct)] | Code-tokenization and tool-like edit behavior are directly targeted to coding tasks | Main production coding agent | Primary coder choice for repo work |
| `Qwen/Qwen2.5-14B-Instruct` | 14.7B [[HF model card](https://huggingface.co/Qwen/Qwen2.5-14B-Instruct)] | General reasoning + coding utility, full 131,072 token context in model config | When you need larger planning windows | Base generalist model in same family |
| `deepseek-ai/DeepSeek-R1-Distill-Qwen-14B` | ~14B distill family (from DeepSeek) [[HF model card](https://huggingface.co/deepseek-ai/DeepSeek-R1-Distill-Qwen-14B?inference_provider=featherless-ai)] | Strong reasoning behavior for hard debugging/review tasks | A second brain for design reasoning and bug hunts | Keep temperature low and be explicit in instructions |
| `microsoft/phi-4` | 14B [[HF card](https://huggingface.co/microsoft/phi-4)] + [technical report](https://arxiv.org/abs/2412.08905) | Good reasoning head, compact size, useful fallback when task is architecture/design-heavy | Keep as alternate model for design tasks | Not as code-specialized as Qwen-Coder |
| `meta-llama/Meta-Llama-3.1-8B-Instruct` | 8B [[HF card](https://huggingface.co/meta-llama/Meta-Llama-3.1-8B-Instruct)] | Very useful fallback for cheap tests/quick diffs | Lightweight “cheap brain” when latency matters | Not 14B, but helps when you need responsiveness |
| `qwen2.5-coder-14b-128k-style quantizations` (community/quant files) | GGUF/GGML/GGUF-compatible quant builds | Same architecture with quantization and often fewer VRAM surprises | If you need long-context with stricter memory envelope | Prefer quantizations that are community-vetted |

Notes:
- Qwen model cards explicitly show 14.7B and context details [[Qwen2.5-14B](https://huggingface.co/Qwen/Qwen2.5-14B-Instruct)] [[Qwen2.5-Coder-14B](https://huggingface.co/Qwen/Qwen2.5-Coder-14B-Instruct)].
- DeepSeek distill has explicit 14B variant in its model family listing [[DeepSeek card](https://huggingface.co/deepseek-ai/DeepSeek-R1-Distill-Qwen-14B?inference_provider=featherless-ai)].

---

## 3) Core stack choices (server + agent harness)

Use one of these patterns:

### Option A (recommended): vLLM + Qwen Code

**What it gives you:** local OpenAI-compatible endpoint + Codex-like terminal/file workflow with MCP + shell + web tools.

vLLM is used as the model server, Qwen Code is the agent UI/orchestrator.

### Option B: SGLang + Qwen Code

Same concept, alternative runtime for some models and config patterns.

### Option C: vLLM + Aider

More minimal CLI agent style, easier to script, less integrated web tooling than Qwen Code.

### Option D: vLLM + OpenHands

Research/dev-centric generalist agent stack with built-in file/terminal/web agents.

---

## 4) Install base tools

```bash
# System deps (Ubuntu/Debian examples)
sudo apt-get update
sudo apt-get install -y git python3 python3-venv curl wget build-essential

# Python environment (recommended)
curl -LsSf https://astral.sh/uv/install.sh | sh
source "$HOME/.cargo/env"
uv venv .venv
source .venv/bin/activate
```

### Install vLLM (OpenAI-compatible serving)

vLLM’s official CLI is the `vllm` command and includes the OpenAI-compatible server flow, including `vllm serve`. [[vLLM CLI docs](https://docs.vllm.ai/en/latest/cli/)]

```bash
uv pip install vllm
vllm --help
```

### Install SGLang (optional alternative runtime)

SGLang documents OpenAI-compatible API support and launch patterns explicitly. [[SGLang OpenAI API](https://docs.sglang.io/backend/openai-compatible-api)]

```bash
uv pip install sglang
python3 -m sglang --help
```

### Install Qwen Code

Qwen Code install can be done from shell script, npm, or Homebrew per upstream docs. [[Install guide (GitHub)](https://github.com/QwenLM/qwen-code)]

```bash
curl -fsSL https://qwen-code-assets.oss-cn-hangzhou.aliyuncs.com/installation/install-qwen-standalone.sh | bash
# Restart terminal after install (as docs mention)
```

### Install Aider (alternative harness)

Aider supports OpenAI-compatible endpoints via env vars. [[Aider OpenAI-compatible docs](https://aider.chat/docs/llms/openai-compat.html)]

```bash
uv pip install aider-chat
```

### Install OpenHands CLI (optional)

OpenHands supports local, web, and headless agent execution modes. [[OpenHands quick start](https://docs.openhands.dev/overview/quickstart)] [[Headless mode](https://docs.openhands.dev/openhands/usage/cli/headless)]
]

```bash
# check current OpenHands install path for your release before following
```

---

## 5) Serve a model locally (practical commands)

### Serve with vLLM

```bash
source .venv/bin/activate

vllm serve Qwen/Qwen2.5-Coder-14B-Instruct \
  --host 0.0.0.0 \
  --port 8000 \
  --dtype auto \
  --max-model-len 32768 \
  --gpu-memory-utilization 0.90 \
  --served-model-name qwen25-coder-14b
```

Test endpoint:

```bash
curl -s http://127.0.0.1:8000/v1/chat/completions \
  -H 'Content-Type: application/json' \
  -d '{"model":"qwen25-coder-14b","messages":[{"role":"user","content":"Explain what this repo does in 3 bullets."}],"max_tokens":120}'
```

### Serve with SGLang

```bash
python3 -m sglang.launch_server \
  --model-path Qwen/Qwen2.5-Coder-14B-Instruct \
  --host 0.0.0.0 \
  --port 30000 \
  --tokenizer-chat-template qwen2.5
```

Query:

```bash
curl -s http://127.0.0.1:30000/v1/chat/completions \
  -H 'Content-Type: application/json' \
  -d '{"model":"Qwen/Qwen2.5-Coder-14B-Instruct","messages":[{"role":"user","content":"You are doing local tooling. Write a 1-line plan."}],"max_tokens":64}'
```

---

## 6) Qwen Code setup for read/edit/run/search parity

Qwen Code docs list file tools and shell tool coverage (`read_*`, `edit`, `run_shell_command`, etc.). [[Tool docs](https://qwenlm.github.io/qwen-code-docs/en/developers/tools/introduction/)]
]

It also supports MCP and web search tool integrations. [[MCP docs](https://qwenlm.github.io/qwen-code-docs/en/users/features/mcp/)]

Minimal environment wiring:

```bash
export QWEN_API_BASE_URL=http://127.0.0.1:8000/v1
export QWEN_API_KEY=not-used
export QWEN_MODEL=qwen/Qwen2.5-Coder-14B-Instruct
```

Run with a safe-by-default profile:

```bash
qwen --approval-mode default --model openai/Qwen2.5-Coder-14B-Instruct
```

Optional JSON stream mode for scripting:

```bash
# stream-json requires both input/output format flags when using streaming JSON protocol
qwen --input-format stream-json --output-format stream-json
```

MCP example for web search integration:

```bash
qwen mcp add --transport stdio tavily \
  npx -y @modelcontextprotocol/server-fetch

# In-session: /mcp then verify the fetch tool is visible
```

Web+repo policy defaults:
- Start with project-level `.qwen/settings.json` deny-lists and allow-lists.
- If you run remote MCP/web tools, isolate API tokens in env vars and avoid global sharing.

---

## 7) Aider: compact harness setup

Aider is good if you prefer a fast CLI patch loop with OpenAI-compatible servers.

```bash
export OPENAI_API_BASE=http://127.0.0.1:8000/v1
export OPENAI_API_KEY=not-used

aider --model openai/Qwen2.5-Coder-14B-Instruct .
```

For scripted workflows, keep a command file and require periodic manual commit points (`/diff`, `/run`, etc.).

---

## 8) OpenHands: file+terminal+web agent pattern

OpenHands supports file-based agents, terminal execution, and built-in web-researcher tool when browser tools are enabled. [[File-Based Agents](https://docs.openhands.dev/sdk/guides/agent-file-based)]

Headless execution is fully autonomous and runs without confirmation by design, so only in trusted repo boundaries. [[Headless](https://docs.openhands.dev/openhands/usage/cli/headless)]

```bash
# Example concept (check your installed package's CLI entrypoint for exact syntax)
openhands --headless -t "Run tests, fix failures, and summarize patch diff in 10 bullets"
```

---

## 9) Minimal custom harness (if you want your own control)

If you want strict parity with typical coding workflows, build one custom loop around an OpenAI-compatible endpoint and a tool schema.

```python
import json
import os
import subprocess
from openai import OpenAI

client = OpenAI(base_url="http://127.0.0.1:8000/v1", api_key="not-used")

TOOLS = {
    "read_file": lambda path: open(path).read(),
    "write_file": lambda path, content: open(path, "w").write(content),
    "run_shell": lambda cmd: subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=120),
}

messages = [
    {"role": "system", "content": "Operate only in /workspace/projects/current. Return JSON objects only for actions."},
    {"role": "user", "content": "Read README.md, fix broken import in src/main.py, run pytest."},
]

resp = client.chat.completions.create(
    model="Qwen/Qwen2.5-Coder-14B-Instruct",
    messages=messages,
    max_tokens=512,
)

# In practice, parse assistant function calls or strict JSON payloads and execute selected tools.
print(resp.model_dump_json(indent=2))
```

This is a starting point; add:
- command timeout + output size caps
- AST-aware patch safety (no blind rewrites)
- allowlist of writable paths
- task-specific guardrails (`git diff` review checkpoints)
- rollback (always keep a clean commit before task)

---

## 10) Recommended performance knobs (for 5090)

Start here:
- `--max-model-len 8192` or `16384` then increase only if stable.
- `--gpu-memory-utilization 0.88` to avoid allocator pressure.
- `--max-num-seqs 1` for deterministic dev workflows.
- Prefer long-context by disabling aggressive history truncation only when needed.
- Keep a smaller model for lightweight tasks to avoid queueing.

If you must increase speed over quality:
- reduce max tokens per request
- keep `top_p=0.9`, `temperature=0.2` for patch edits
- keep tools focused (single-tool-per-turn policy)


**Why these knobs:** long context and high parallel sequences can blow VRAM first on Blackwell-class consumer cards.

---

## 11) What “Parity with Codex/Claude” really means

You can get close on capabilities if you cover three layers:
1. **Model layer**: 14B instruction model with code/editing behavior (`Qwen2.5-Coder-14B-Instruct`).
2. **Tool layer**: file read/write + shell + web fetch in a constrained policy.
3. **Safety layer**: structured output, patch gating, and auto-rules.

What is usually missing vs cloud agents:
- cloud-grade memory of long project history at scale
- managed grounding/rag systems (build this yourself)
- built-in compliance/restrictive tool audit in enterprise environments

But for local software engineering tasks (debug, refactor, grep, test-fix), this setup is workable.

---

## 12) Validation checklist before declaring this "done"

Use this in order:

- [ ] Model server answers JSON/chat request on `/v1/chat/completions`.
- [ ] Agent can read a file from repo and summarize it.
- [ ] Agent edits one file and you can open generated diff cleanly.
- [ ] Agent runs one shell command and captures output.
- [ ] One headless cycle can fix at least one failing test.
- [ ] One web-fetch action returns and cites content in file.
- [ ] Every destructive command is sandboxed or denied by default.
- [ ] Keep a rollback commit strategy and a patch review gate.

---

## Sources

- NVIDIA RTX 5090 spec page: https://www.nvidia.com/en-us/geforce/graphics-cards/50-series/rtx-5090/
- Qwen2.5-14B (HF): https://huggingface.co/Qwen/Qwen2.5-14B-Instruct
- Qwen2.5-Coder-14B-Instruct (HF): https://huggingface.co/Qwen/Qwen2.5-Coder-14B-Instruct
- DeepSeek R1 Distill Qwen-14B (HF): https://huggingface.co/deepseek-ai/DeepSeek-R1-Distill-Qwen-14B?inference_provider=featherless-ai
- Microsoft Phi-4 card: https://huggingface.co/microsoft/phi-4
- DeepSeek technical report: https://arxiv.org/abs/2501.12948
- Qwen Code docs (tools): https://qwenlm.github.io/qwen-code-docs/en/developers/tools/introduction/
- Qwen Code MCP docs: https://qwenlm.github.io/qwen-code-docs/en/users/features/mcp/
- Qwen Code config rules/output format: https://github.com/QwenLM/qwen-code/blob/main/docs/users/configuration/settings.md
- Qwen Code install: https://github.com/QwenLM/qwen-code
- vLLM CLI docs: https://docs.vllm.ai/en/latest/cli/
- vLLM serve docs: https://docs.vllm.ai/en/stable/cli/serve/
- SGLang OpenAI API: https://sgl-project-sglang-93.mintlify.app/backend/openai-compatible-api
- SGLang docs start: https://docs.sglang.io/
- Aider OpenAI-compatible API support: https://aider.chat/docs/llms/openai-compat.html
- OpenHands docs: https://docs.openhands.dev/overview/quickstart
- OpenHands headless: https://docs.openhands.dev/openhands/usage/cli/headless
- OpenHands file/web agents: https://docs.openhands.dev/sdk/guides/agent-file-based
