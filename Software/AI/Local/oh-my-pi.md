# oh-my-pi

`oh-my-pi` is a terminal-first AI coding harness with IDE-level awareness, built around one model loop and one integrated tool set. The project describes itself as a fork of Mario Zechner’s Pi, with an expanded and performance-tuned command surface for serious agent workflows.([GitHub README](https://github.com/can1357/oh-my-pi))  

---

## 🧠 Why this exists

`oh-my-pi` is useful when you want a single coding agent that can:

- edit files structurally, not just by rewriting full text
- run shell commands, debugger sessions, and read/write tool calls through one control plane
- coordinate multiple worker sessions
- consume both hosted APIs and local model endpoints

The repository advertises these capabilities directly in its feature list and runtime docs, including persistent Python and Bun kernels, task fan-out, debugger support, and session collaboration.([README feature list](https://github.com/can1357/oh-my-pi))  

---

## 🧩 What it is

- `oh-my-pi` is branded as *A coding agent with the IDE wired in*.([README](https://github.com/can1357/oh-my-pi))
- It states **40+ providers**, **32 built-in tools**, **14 LSP ops**, and **28 DAP ops**, with a Rust-heavy core.([README](https://github.com/can1357/oh-my-pi), [license](https://raw.githubusercontent.com/can1357/oh-my-pi/main/LICENSE))
- It is MIT-licensed software, so the code and docs are redistributable under permissive terms.([MIT License](https://raw.githubusercontent.com/can1357/oh-my-pi/main/LICENSE))
- It is a fork lineage from the `Pi` project by Mario Zechner, which explains the “harness + IDE glue” approach.([README](https://github.com/can1357/oh-my-pi))

---

## 🛠️ Core workflow

### Install surface

The README lists install methods for:

- macOS / Linux bootstrap script
- Homebrew tap
- Bun (recommended package install)
- Windows PowerShell installer
- mise pinned install
- minimum Bun version `1.3.14`
- shell completions for bash, zsh, fish  
([README Install](https://github.com/can1357/oh-my-pi))

### Runtime tool surface

`oh-my-pi` exposes a broad single namespace with filesystem, shell, code intelligence, coordination, and I/O tools such as:

- `read`, `write`, `edit`, `ast_edit`, `search`, `find`
- `bash`, `eval`, `ssh`
- `lsp`, `debug`
- `task`, `hub`, `todo`
- `browser`, `web_search`, `github`
- `web_search` + `read` chains for URL/doc ingestion  
([README tools](https://github.com/can1357/oh-my-pi))

### Deep features repeatedly highlighted by the project

- **Tool-calling across kernels:** persistent Python and Bun execution, with either calling back into agent tools.([Feature section](https://github.com/can1357/oh-my-pi))
- **LSP-aware edits:** rename and symbol workflows can be run through language server actions, not ad-hoc string replacement.([Feature section](https://github.com/can1357/oh-my-pi))
- **Debugger hooks:** lldb and dlv attachment with stack/thread/variable workflows in-agent.([Feature section](https://github.com/can1357/oh-my-pi))
- **Tasked subagents:** parallel workers with typed structured outputs for isolation and aggregation.([Feature section](https://github.com/can1357/oh-my-pi))
- **Second reviewer/advisor model:** optional second model can shadow turns with inline notes and concerns.([Feature section](https://github.com/can1357/oh-my-pi))
- **GitHub integration as URL scheme:** PR/issue/file artifacts surfaced via internal `pr://`/`issue://` navigation.([Feature section](https://github.com/can1357/oh-my-pi))

---

## 🔧 Configuration and extension points

### Model and provider routing

Model behavior is driven by runtime registries and config files (`models.yml` / `models.yaml` under `~/.omp/agent/`, with legacy migration behavior documented).([models config docs](https://raw.githubusercontent.com/can1357/oh-my-pi/main/docs/models.md))

The project documents direct local model targets via OpenAI-compatible protocols:
- Ollama
- Ollama Cloud
- LM Studio
- llama.cpp
- vLLM
- LiteLLM  
([provider table and examples](https://github.com/can1357/oh-my-pi))

### Environment behavior

Environment loading precedence is:
1. process env (`Bun.env`)
2. project `.env`
3. agent `.env`
4. config-root `.env`
5. home `.env`  
with `OMP_*` mirrored to `PI_*` inside parsed files.([env reference](https://raw.githubusercontent.com/can1357/oh-my-pi/main/docs/environment-variables.md))

This makes local secrets portable across session contexts, but you should avoid embedding credentials in shell history and treat config files as sensitive.([env reference](https://raw.githubusercontent.com/can1357/oh-my-pi/main/docs/environment-variables.md))

### Skills and custom tools

`oh-my-pi` has a file-discovery model for `SKILL.md` packs and strict provider layout rules for discovery.([skills docs](https://raw.githubusercontent.com/can1357/oh-my-pi/main/docs/skills.md))  
It also supports custom tool injection via TypeScript/JS tool modules and extension loaders.([custom tools docs](https://raw.githubusercontent.com/can1357/oh-my-pi/main/docs/custom-tools.md))

---

## 🧪 How it fits with local LLM workflows

For users building private, local-first stacks, `oh-my-pi` can route to local inference backends and custom OpenAI-compatible endpoints using model-provider settings. This is a direct compatibility point with your broader local-first stack strategy.([provider list](https://github.com/can1357/oh-my-pi), [models docs](https://raw.githubusercontent.com/can1357/oh-my-pi/main/docs/models.md))

If you are mapping where local model policy matters, connect this to [[LLM Under Your Floorboards]].([floorboards note](Software/AI/Local/LLM%20Under%20Your%20Floorboards.md))

---

## 📊 Comparison chart

| Harness | Core role | Tool philosophy | Model routing style | Collaboration / scope |
|---|---|---|---|---|
| oh-my-pi | Terminal coding agent harness | Rich single namespace (`read`, `edit`, `lsp`, `debug`, `task`, `web_search`) + custom tools | Multi-provider + custom OpenAI-compatible local endpoints (`~/.omp/agent/models.yml`) | Subagents, collab links, session controls, advisor model | 
| Claude Code | Anthropic terminal agent | CLI command surface with permission model and plugin support | Primarily Anthropic APIs by default, with CLI command/config path ecosystem | Built-in CLI controls, plugin tooling, session model | [Anthropic docs](https://code.claude.com/docs/en/getting-started) |
| OpenAI Codex | General terminal coding agent | CLI plus optional IDE integration and codex app modes | Provider-side OpenAI model services | Supports terminal + editor bindings | [OpenAI Codex repo](https://github.com/openai/codex) |
| OpenHands | Autonomous coding platform | Multi-agent server/client architecture with server front-end | Multiple backends and model providers via server config | Multi-server and team shared agent canvas | [OpenHands README](https://github.com/OpenHands/OpenHands) |
| Aider | Pair-programming terminal assistant | Diff-oriented conversational editing in terminal sessions | Works with multiple LLM endpoints in its config model | Lightweight per-repo workflow | [Aider docs](https://aider.chat/docs/) |
| GitHub Copilot CLI | GitHub-native terminal agent | Integrated terminal agent workflows and issue/PR flow | Cloud/API-centric GitHub-integrated model calls | Terminal-native command/agent flow inside GitHub ecosystem | [GitHub Copilot CLI docs](https://docs.github.com/en/copilot/concepts/agents/copilot-cli/about-copilot-cli) |
| Cody | Code intelligence assistant | Search + chat + agentic context workflows on Sourcegraph-backed index | Model provider managed by Sourcegraph service model abstraction | IDE/web context-aware workflows | [Sourcegraph Cody docs](https://sourcegraph.com/docs/cody) |

---

## ✅ Strengths and caveats

### Strengths
- Broad and explicit tool breadth for terminal + language-tooling tasks.
- Strong local-model path via local-compatible provider list and custom provider registration.
- Fast onboarding through explicit install matrix and documented completion support.
- Open-source and modular extension paths for internal teams.

### Caveats
- Rust/Bun-heavy stack can mean heavier runtime demands than thin CLI agents.
- Advanced usage depends on understanding model roles, provider auth keys, and environment precedence.
- The project evolves quickly; confirm behavior against the current tag/release before scripting production workflows.

---

## Related Concepts/Notes

- [[LLM Under Your Floorboards]]
- [[Model Context Protocol]]
- [[LSP]]
- [[Git]]
- [[Docker]]

---

## 📚 External resources

- oh-my-pi repository and official README: https://github.com/can1357/oh-my-pi
- Source code metadata and scripts (package metadata, Bun version, workspace layout): https://raw.githubusercontent.com/can1357/oh-my-pi/main/package.json
- MIT license: https://raw.githubusercontent.com/can1357/oh-my-pi/main/LICENSE
- Skills docs: https://raw.githubusercontent.com/can1357/oh-my-pi/main/docs/skills.md
- Environment variable precedence: https://raw.githubusercontent.com/can1357/oh-my-pi/main/docs/environment-variables.md
- Model registry behavior: https://raw.githubusercontent.com/can1357/oh-my-pi/main/docs/models.md
- Custom tool loading: https://raw.githubusercontent.com/can1357/oh-my-pi/main/docs/custom-tools.md
