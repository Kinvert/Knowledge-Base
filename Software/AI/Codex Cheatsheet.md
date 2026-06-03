# Codex Cheatsheet

Codex CLI is OpenAI's local coding agent. It runs in your terminal, works inside the current workspace, edits files, runs shell commands, reviews diffs, uses skills, installs plugins, connects to MCP servers, and can also run non-interactively for scripts or CI.

This note is based on local `codex-cli 0.130.0` plus the official Codex docs as of 2026-05-21.

---

## Fast Start

- `codex` - open interactive Codex in the current directory.
- `codex "fix the failing tests"` - start interactive Codex with an initial prompt.
- `codex -C /path/to/repo` - start Codex with a specific working root.
- `codex -m gpt-5.5` - run with a specific model.
- `codex -c model_reasoning_effort='"xhigh"'` - run with extra-high reasoning for this session.
- `codex --search` - enable live web search for the session.
- `codex --no-alt-screen` - keep output in normal terminal scrollback.
- `codex resume --last` - continue the most recent interactive session.
- `codex fork --last` - branch the most recent session into a new thread.
- `codex update` - update Codex.
- `codex login status` - show auth status.

Important correction: `--xhigh` is not a Codex CLI flag. Use `-c model_reasoning_effort='"xhigh"'`, set `model_reasoning_effort = "xhigh"` in `~/.codex/config.toml`, or use `/model` in the TUI.

---

## Current Local Defaults

Your current `~/.codex/config.toml` defaults:

```toml
sandbox_mode = "workspace-write"
approval_policy = "untrusted"
model_reasoning_effort = "xhigh"
model = "gpt-5.5"
```

Meaning:

- `workspace-write` lets Codex edit the current workspace and configured writable roots.
- `untrusted` asks for approval for commands outside the trusted allowlist.
- `gpt-5.5` is your default model.
- `xhigh` is already your default reasoning effort, so you usually do not need a CLI flag for it.

---

## CLI Command Map

| Command | What it does |
|---|---|
| `codex` | Interactive terminal UI. |
| `codex exec` / `codex e` | Non-interactive run for scripts, one-shot tasks, or CI. |
| `codex review` | Non-interactive code review. |
| `codex login` | Manage login. |
| `codex logout` | Remove stored credentials. |
| `codex mcp` | Manage external MCP servers. |
| `codex plugin` | Manage Codex plugin marketplaces. |
| `codex mcp-server` | Start Codex as an MCP server over stdio. |
| `codex app-server` | Experimental app server tooling. |
| `codex remote-control` | Experimental headless app-server with remote control. |
| `codex completion` | Generate shell completion scripts. |
| `codex update` | Update Codex. |
| `codex sandbox` | Run commands inside Codex-provided sandbox wrappers. |
| `codex debug` | Debug tools, model catalog, and prompt input inspection. |
| `codex apply` / `codex a` | Apply a Codex Cloud/task diff locally with `git apply`. |
| `codex resume` | Resume an interactive session. |
| `codex fork` | Fork an interactive session. |
| `codex cloud` | Experimental Codex Cloud task browsing and diff application. |
| `codex features` | Inspect and persist feature flags. |
| `codex help` | Show command help. |

---

## Global Flags

These work on the top-level interactive CLI and many subcommands:

| Flag | Use |
|---|---|
| `-c, --config key=value` | Override a config value for this run. Values are TOML parsed. |
| `--enable FEATURE` | Equivalent to `-c features.FEATURE=true`. Repeatable. |
| `--disable FEATURE` | Equivalent to `-c features.FEATURE=false`. Repeatable. |
| `-i, --image FILE` | Attach image files to the initial prompt. Repeatable. |
| `-m, --model MODEL` | Override model for this run. |
| `--oss` | Use an open-source/local provider. |
| `--local-provider lmstudio|ollama` | Pick the local OSS provider. |
| `-p, --profile PROFILE` | Load a config profile. |
| `-s, --sandbox read-only|workspace-write|danger-full-access` | Set sandbox mode. |
| `--dangerously-bypass-approvals-and-sandbox` | No approvals and no sandbox. Extremely risky. |
| `--yolo` | Hidden/deprecated alias for dangerous no-sandbox/no-approval mode. Only use inside an external sandbox. |
| `-C, --cd DIR` | Set working root. |
| `--add-dir DIR` | Add another writable directory. |
| `-a, --ask-for-approval untrusted|on-request|never|on-failure` | Set approval policy. |
| `--search` | Enable live web search. |
| `--no-alt-screen` | Keep normal terminal scrollback. |
| `-h, --help` | Help. |
| `-V, --version` | Version. |

Approval policies:

- `untrusted` - only trusted read-ish commands run without approval.
- `on-request` - Codex chooses when to ask.
- `never` - never ask; failures are returned to the model.
- `on-failure` - deprecated; prefer `on-request` or `never`.

Sandbox modes:

- `read-only` - safest; no file edits.
- `workspace-write` - normal useful mode; edits workspace.
- `danger-full-access` - full local access. Use carefully.

---

## Reasoning Effort

Supported reasoning levels in the bundled model catalog:

- `low` - faster, lighter reasoning.
- `medium` - default balanced reasoning.
- `high` - deeper reasoning.
- `xhigh` - extra-high reasoning for complex tasks.

Use it per run:

```bash
codex -c model_reasoning_effort='"xhigh"'
codex exec -c model_reasoning_effort='"xhigh"' "refactor this module"
codex -m gpt-5.5 -c model_reasoning_effort='"high"'
```

Set it globally:

```toml
model = "gpt-5.5"
model_reasoning_effort = "xhigh"
```

Interactive shortcut:

- `/model` - choose model and reasoning effort from the TUI.

---

## Non-Interactive Exec

Use `codex exec` when you want Codex to do one job and exit.

```bash
codex exec "fix the lint errors and run tests"
codex e "summarize this repo"
codex exec -C /path/to/repo "add tests for the parser"
printf 'Review this diff\n' | codex exec -
git diff | codex exec "review this patch"
```

Useful `exec` flags:

| Flag | Use |
|---|---|
| `--json` | Print JSONL event stream. Good for scripts. |
| `-o, --output-last-message FILE` | Write final answer to a file. |
| `--output-schema FILE` | Require final response to match a JSON Schema. |
| `--ephemeral` | Do not persist session files. |
| `--skip-git-repo-check` | Allow running outside a Git repo. |
| `--ignore-user-config` | Ignore `~/.codex/config.toml`; auth still uses `CODEX_HOME`. |
| `--ignore-rules` | Ignore execpolicy `.rules` files. |
| `--color always|never|auto` | Control ANSI color. |
| `--full-auto` | Deprecated; prefer `--sandbox workspace-write`. |
| `--yolo` | Alias for bypassing approvals and sandbox. Dangerous. |

Good automation patterns:

```bash
codex exec --json "find dead code" > codex-events.jsonl
codex exec -o result.md "write release notes from git log"
codex exec --output-schema schema.json "extract a migration checklist"
codex exec --sandbox read-only "inspect this repo and report risks"
codex exec --ask-for-approval never --sandbox workspace-write "format and test"
```

---

## Review

```bash
codex review --uncommitted
codex review --base main
codex review --commit abc1234
codex review --title "Parser refactor"
codex review "focus on security and data loss risks"
```

Use review mode when you want findings first: bugs, regressions, missing tests, and risk.

---

## Resume, Fork, Apply

```bash
codex resume
codex resume --last
codex resume --all
codex resume SESSION_ID "continue from here"
codex fork
codex fork --last
codex fork SESSION_ID "try a different approach"
codex apply TASK_ID
```

Use `resume` to keep working in the same thread. Use `fork` when you want to branch the conversation without losing the old one.

---

## Slash Commands

Inside interactive `codex`, type `/` to open the command picker. A running task can have a slash command queued with `Tab`.

Most useful:

| Command | Use |
|---|---|
| `/model` | Change model and reasoning effort. |
| `/permissions` | Change approval/sandbox behavior mid-session. |
| `/status` | Show model, approval policy, writable roots, and token usage. |
| `/debug-config` | Debug config precedence and active policy. |
| `/diff` | Show Git diff including untracked files. |
| `/review` | Review current working tree. |
| `/compact` | Summarize long history to free context. |
| `/clear` | Clear terminal and start a fresh chat. |
| `/new` | Start a new conversation in same CLI. |
| `/resume` | Resume a saved conversation. |
| `/fork` | Branch current conversation into a new thread. |
| `/side` | Start an ephemeral side conversation. |
| `/mention` | Attach a file/folder to the prompt. |
| `/skills` | Browse and insert skills. |
| `/plugins` | Browse/install/manage plugins. |
| `/apps` | Browse connectors/apps. |
| `/mcp` | List MCP tools. |
| `/hooks` | View lifecycle hooks. |
| `/memories` | Configure memory use/generation. |
| `/experimental` | Toggle experimental features. |
| `/approve` | Retry a recent auto-review denial once. |
| `/fast on|off|status` | Toggle/check fast service tier if the model supports it. |
| `/personality` | Choose `friendly`, `pragmatic`, or `none`. |
| `/plan` | Switch into plan mode. |
| `/goal` | Experimental persistent task goal. Requires `features.goals`. |
| `/ps` | Show background terminals. |
| `/stop` | Stop background terminals. |
| `/raw on|off` | Toggle raw scrollback. |
| `/copy` | Copy latest completed output. |
| `/keymap` | Remap TUI shortcuts. |
| `/vim` | Toggle Vim composer mode. |
| `/statusline` | Configure footer/status line. |
| `/title` | Configure terminal title. |
| `/theme` | Choose syntax theme. |
| `/init` | Generate `AGENTS.md`. |
| `/logout` | Sign out. |
| `/quit` or `/exit` | Exit. |

Power habit:

- Use `/status` before risky work to confirm model, sandbox, approvals, cwd, and writable roots.
- Use `/diff` before finalizing.
- Use `/compact` after long investigative work.
- Use `/fork` before trying a risky alternate implementation.
- Use `/side` for quick questions that should not pollute the main thread.

---

## Config Overrides

`-c` is the real power-user flag. It overrides `~/.codex/config.toml` for one run.

Examples:

```bash
codex -c model='"gpt-5.5"'
codex -c model_reasoning_effort='"xhigh"'
codex -c sandbox_mode='"read-only"'
codex -c approval_policy='"never"'
codex -c 'features.goals=true'
codex -c 'shell_environment_policy.inherit="all"'
codex -c 'sandbox_permissions=["disk-full-read-access"]'
```

TOML quoting matters. Strings should usually be wrapped like `'"xhigh"'` in shell commands so Codex receives TOML string syntax.

Config locations:

- `~/.codex/config.toml` - user config.
- Project trust entries live under `[projects."/path"]`.
- `AGENTS.md` - repo/project instructions that Codex reads.
- `.rules` files - execpolicy rules, unless `--ignore-rules` is used.

---

## Features

```bash
codex features list
codex features enable memories
codex features disable memories
codex --enable goals
codex --disable goals
```

Useful stable features on this install include:

- `apps`
- `browser_use`
- `computer_use`
- `fast_mode`
- `hooks`
- `image_generation`
- `multi_agent`
- `personality`
- `plugins`
- `shell_tool`
- `tool_search`
- `unified_exec`
- `workspace_dependencies`

Useful experimental features:

- `goals`
- `memories`
- `prevent_idle_sleep`
- `terminal_resize_reflow`

---

## MCP Servers

MCP gives Codex external tools and resources.

```bash
codex mcp list
codex mcp add docs --url https://example.com/mcp
codex mcp add local-tool -- node server.js
codex mcp add local-tool --env KEY=value -- node server.js
codex mcp get local-tool
codex mcp get local-tool --json
codex mcp login local-tool
codex mcp login local-tool --scopes read,write
codex mcp logout local-tool
codex mcp remove local-tool
```

Official OpenAI docs MCP setup:

```bash
codex mcp add openaiDeveloperDocs --url https://developers.openai.com/mcp
```

During an interactive session:

- `/mcp` - list configured MCP tools.
- `/mcp verbose` - show more server details.

---

## Plugins

Plugins bundle reusable skills, app integrations, and MCP servers. Use them when you want installable workflows, tool access, or app connectors.

Interactive:

```bash
codex
/plugins
```

CLI marketplace commands:

```bash
codex plugin marketplace add owner/repo
codex plugin marketplace add owner/repo@ref
codex plugin marketplace add https://github.com/owner/repo.git
codex plugin marketplace add /local/marketplace/root
codex plugin marketplace add owner/repo --ref main
codex plugin marketplace add owner/repo --sparse path/in/repo
codex plugin marketplace upgrade
codex plugin marketplace upgrade MARKETPLACE_NAME
codex plugin marketplace remove MARKETPLACE_NAME
```

Plugin manifest shape:

- Plugin root contains `.codex-plugin/plugin.json`.
- A plugin can include `skills`, `hooks`, `mcpServers`, `apps`, and UI metadata.
- A marketplace uses `.agents/plugins/marketplace.json`.

Curated local marketplace categories visible on this machine:

- Coding: `build-ios-apps`, `build-macos-apps`, `build-web-apps`, `circleci`, `cloudflare`, `cloudinary`, `coderabbit`, `codex-security`, `expo`, `game-studio`, `github`, `hostinger`, `hugging-face`, `marcopolo`, `neon-postgres`, `netlify`, `quicknode`, `render`, `sendgrid`, `sentry`, `statsig`, `supabase`, `superpowers`, `temporal`, `twilio-developer-kit`, `vantage`, `vercel`, `yepcode`.
- Productivity: `asana`, `gmail`, `google-calendar`, `linear`, `notion`, `outlook-calendar`, `outlook-email`, `sharepoint`, `slack`, `stripe`, `teams`, `zoom`, plus many CRM/ops tools.
- Research: `alpaca`, `binance`, `cb-insights`, `daloopa`, `dow-jones-factiva`, `life-science-research`, `morningstar`, `pitchbook`, `readwise`, `scite`, `zotero`, and others.
- Design: `biorender`, `canva`, `figma`, `heygen`, `hyperframes`, `remotion`.
- Engineering: `openai-developers`.

Good installs to know:

- `github` - inspect repos, PRs, issues, CI, and publish changes.
- `openai-developers` - OpenAI APIs, Agents SDK, ChatGPT Apps, API key workflows.
- `build-web-apps` - frontend app building and browser testing workflows.
- `superpowers` - planning, TDD, debugging, and collaboration workflow system.

---

## Skills

Skills are task-specific instructions in a directory with a required `SKILL.md`. Codex loads the full skill only when it is selected or when the task matches its description.

Use skills explicitly:

```text
$skill-creator
$plugin-creator
$openai-docs
```

Interactive:

```text
/skills
```

Installed skills on this machine:

| Skill | Use |
|---|---|
| `imagegen` | Generate or edit raster images. |
| `openai-docs` | Use current official OpenAI docs. |
| `plugin-creator` | Scaffold Codex plugins and marketplace entries. |
| `skill-creator` | Create or update Codex skills. |
| `skill-installer` | Install skills from curated lists or repos. |
| `vxdb` | Persistent semantic KV/memory over MCP-style infrastructure. |

Skill structure:

```text
my-skill/
  SKILL.md
  scripts/
  references/
  assets/
  agents/
```

Disable a skill in `~/.codex/config.toml`:

```toml
[[skills.config]]
path = "/path/to/skill/SKILL.md"
enabled = false
```

Restart Codex after skill config changes.

---

## Shell Completion

```bash
codex completion bash
codex completion zsh
codex completion fish
codex completion powershell
codex completion elvish
```

---

## Auth

```bash
codex login
codex login status
printenv OPENAI_API_KEY | codex login --with-api-key
printenv CODEX_ACCESS_TOKEN | codex login --with-access-token
codex login --device-auth
codex logout
```

---

## Debugging Codex Itself

```bash
codex debug models
codex debug models --bundled
codex debug prompt-input
codex debug app-server
codex features list
```

Useful checks:

```bash
codex --version
codex --help
codex exec --help
codex review --help
codex mcp --help
codex plugin --help
```

---

## Sandbox Command Wrapper

Codex can run arbitrary commands inside its sandbox wrappers:

```bash
codex sandbox linux -- <command>
codex sandbox macos -- <command>
codex sandbox windows -- <command>
```

Linux uses the Linux sandbox, bubblewrap by default. macOS uses Seatbelt. Windows uses a restricted token.

---

## Remote, App Server, Cloud

Remote TUI:

```bash
codex --remote ws://localhost:PORT
codex --remote wss://host:PORT --remote-auth-token-env CODEX_REMOTE_TOKEN
codex resume --remote ws://localhost:PORT
codex fork --remote ws://localhost:PORT
```

App server:

```bash
codex app-server
codex app-server --listen stdio://
codex app-server --listen unix://
codex app-server --listen ws://127.0.0.1:PORT
codex app-server proxy
codex app-server generate-ts
codex app-server generate-json-schema
codex remote-control
```

Codex Cloud:

```bash
codex cloud
codex cloud list
codex cloud status TASK_ID
codex cloud diff TASK_ID
codex cloud apply TASK_ID
codex cloud exec "do the task"
```

---

## Practical Presets

Safe inspection:

```bash
codex --sandbox read-only --ask-for-approval untrusted
```

Normal coding:

```bash
codex --sandbox workspace-write --ask-for-approval untrusted
```

Hands-off local run, still sandboxed:

```bash
codex exec --sandbox workspace-write --ask-for-approval never "fix tests and format"
```

Maximum reasoning:

```bash
codex -m gpt-5.5 -c model_reasoning_effort='"xhigh"'
```

Danger mode:

```bash
codex --dangerously-bypass-approvals-and-sandbox
codex --yolo
```

Only use danger mode inside an external VM/container/sandbox. It removes both approval prompts and Codex sandboxing.

---

## Good Habits

- Start in `workspace-write`, not `danger-full-access`.
- Use `--add-dir` instead of full access when one extra writable directory is enough.
- Use `/status` before a long task.
- Use `/diff` before committing.
- Use `/review` after Codex edits code.
- Use `/compact` when context gets long.
- Use `/fork` before trying a risky alternate direction.
- Use `codex exec --json` for scripting and CI.
- Use `-c` overrides for one-off config instead of permanently changing defaults.
- Put repo-specific rules in `AGENTS.md` so Codex sees them every time.

---

## References

- Codex CLI command line options: https://developers.openai.com/codex/cli/reference
- Codex CLI slash commands: https://developers.openai.com/codex/cli/slash-commands
- Codex config reference: https://developers.openai.com/codex/config-reference
- Codex skills: https://developers.openai.com/codex/skills
- Codex plugins: https://developers.openai.com/codex/plugins
- Codex permissions: https://developers.openai.com/codex/permissions
