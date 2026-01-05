# Claude Code

Anthropic's official CLI for interacting with Claude directly in the terminal. Unlike web interfaces, Claude Code gives full filesystem access, can run bash commands, edit files, and integrate with your development workflow. This is the power user's interface to Claude.

For local model alternatives see [[LLM Under Your Floorboards]]. For the underlying API see [[Anthropic API]].

---

## 🎯 Core Concepts

**What Claude Code Does:**
- Reads and writes files directly
- Executes bash commands
- Searches codebases with grep/glob
- Manages git operations
- Connects to external tools via MCP
- Delegates to specialized subagents

**Key Difference from ChatGPT/Claude Web:**
You're not copy-pasting code. Claude operates directly on your filesystem with your permission.

---

## ⌨️ Keyboard Shortcuts

*Verified from [official docs](https://code.claude.com/docs/en/interactive-mode)*

| Shortcut | Action |
|----------|--------|
| `Ctrl+C` | Cancel current input or generation |
| `Ctrl+D` | Exit Claude Code |
| `Ctrl+L` | Clear terminal (keeps history) |
| `Ctrl+O` | Toggle verbose output (see thinking) |
| `Ctrl+R` | Reverse search command history |
| `Ctrl+B` | Background long-running process |
| `Esc` + `Esc` | Rewind to previous checkpoint |
| `Shift+Tab` | Cycle permission modes |
| `Tab` | Toggle thinking on/off (v2) |
| `Option+P` (Mac) / `Alt+P` | Switch model without clearing prompt |
| `Ctrl+V` (Mac/Linux) / `Alt+V` (Win) | Paste image from clipboard |

### Multiline Input

| Method | Shortcut |
|--------|----------|
| Quick escape | `\` + `Enter` |
| macOS | `Option+Enter` |
| After `/terminal-setup` | `Shift+Enter` |
| Control sequence | `Ctrl+J` |

### Quick Prefixes

| Prefix | Effect |
|--------|--------|
| `#` | Add note to CLAUDE.md memory |
| `/` | Invoke slash command |
| `!` | Execute bash directly (output added to context) |
| `@` | File path autocomplete |

---

## 📋 Slash Commands

*Verified from [official docs](https://code.claude.com/docs/en/slash-commands)*

### Confirmed Built-in Commands

| Command | Purpose |
|---------|---------|
| `/help` | List available commands |
| `/clear` | Clear conversation history |
| `/compact [focus]` | Compress conversation, optionally focus on topic |
| `/context` | View token usage as colored grid |
| `/model <alias>` | Switch models (sonnet, opus, haiku) |
| `/memory` | Open CLAUDE.md in system editor |
| `/resume` | Interactive session picker |
| `/rewind` | Restore to checkpoint / create fork |
| `/rename <name>` | Name current session |
| `/init` | Create CLAUDE.md file |
| `/hooks` | Configure hook events |
| `/terminal-setup` | Set up Shift+Enter for multiline |
| `/vim` | Enable vim editing mode |

### Custom Slash Commands

Create reusable prompts as markdown files:

**Location:** `.claude/commands/` (project) or `~/.claude/commands/` (personal)

**Example:** `.claude/commands/review.md`

```yaml
---
description: Review code changes
argument-hint: [files]
---

Review these files for bugs and security issues:
$ARGUMENTS
```

**Use:** `/review src/auth.ts`

---

## 🧠 Thinking Modes

*Verified from [community research](https://news.ycombinator.com/item?id=43739997) and [Anthropic best practices](https://www.anthropic.com/engineering/claude-code-best-practices)*

Claude Code has a preprocessing layer that intercepts thinking keywords and allocates thinking budget. This only works in Claude Code CLI - not the web interface or API.

### Thinking Keywords

| Keyword | Budget |
|---------|--------|
| `think` | Minimal |
| `think hard` | Moderate |
| `think harder` | High |
| `ultrathink` | Maximum (31,999 tokens) |

**Usage:** Include keyword in your prompt:
```
ultrathink: design a caching layer for our API
```

### Important Notes

- `ultrathink` only works when `MAX_THINKING_TOKENS` env var is NOT set
- When `MAX_THINKING_TOKENS` is set, it overrides keyword-based allocation
- `Tab` toggles thinking on/off globally (v2)
- `Ctrl+O` to view Claude's thinking process

### When to Use Ultrathink

- Complex architectural decisions
- Multi-step debugging
- Evaluating tradeoffs between approaches
- Unfamiliar codebase analysis

---

## 🔧 Permission Modes

Cycle with `Shift+Tab`:

| Mode | Behavior | Use When |
|------|----------|----------|
| **Normal** | Ask before each action | Default, careful review |
| **Auto-Accept** | Approve most edits | Trust Claude's work |
| **Plan Mode** | Read-only, no changes | Analysis/planning phase |

Start in plan mode: `claude --permission-mode plan`

---

## 🤖 Subagents

Claude can delegate to specialized agents with their own context and tool access.

### Creating Custom Agents

Create `.claude/agents/agent-name.md`:

```yaml
---
name: code-reviewer
description: Reviews code for quality and security
tools: Read, Grep, Glob, Bash
model: inherit
---

You are a senior code reviewer...
```

Or via CLI flag:
```bash
claude --agents '{
  "reviewer": {
    "description": "Reviews code",
    "prompt": "You are a code reviewer...",
    "tools": ["Read", "Grep"],
    "model": "sonnet"
  }
}'
```

---

## 🎨 Custom Skills

*Verified from [support article](https://support.claude.com/en/articles/12512198-how-to-create-custom-skills)*

Skills are comprehensive packages that enhance Claude with specialized knowledge and workflows. Unlike simple slash commands, Skills can include reference files, executable scripts, and complex instructions.

### Skill.md Structure

Every Skill requires a `Skill.md` file with YAML frontmatter:

```yaml
---
name: brand-guidelines
description: Enforces company brand standards for documents and presentations
dependencies: python>=3.8, pillow>=9.0
---

You are a brand compliance assistant...

## Visual Identity
- Primary color: #1a73e8
- Typography: Inter for headings, System UI for body

## Application Rules
...
```

**Required metadata:**
- `name` - Human-friendly identifier (max 64 chars)
- `description` - What it does and when to use it (max 200 chars) - Claude uses this to decide when to invoke

**Optional metadata:**
- `dependencies` - Required packages (must be pre-installed; no runtime installation)

### Adding Resources

Skills can include supplementary files:

```
my-skill/
├── Skill.md           # Main instructions
├── REFERENCE.md       # Additional context
├── scripts/
│   └── analyze.py     # Executable code
└── assets/
    └── template.json  # Data files
```

**Executable scripts** - Python, JavaScript/Node.js for data processing, visualization, file manipulation.

### Packaging

ZIP structure must match the skill name:

```
my-skill.zip
└── my-skill/
    ├── Skill.md
    └── resources/
```

### Skill vs Slash Command vs Agent

| Feature | Slash Command | Skill | Agent |
|---------|---------------|-------|-------|
| Location | `.claude/commands/` | Packaged ZIP | `.claude/agents/` |
| Complexity | Simple prompts | Full workflows | Tool delegation |
| Resources | None | Files, scripts | None |
| Code execution | No | Yes (scripts) | Via tools |
| Packaging | Single file | ZIP folder | Single file |

### Best Practices

- Create focused Skills for individual workflows rather than monolithic ones
- Include clear examples showing expected inputs/outputs
- Start with basic markdown before adding executable code
- Multiple small Skills compose better than one comprehensive Skill
- Avoid hardcoding API keys - use MCP for external services

### Example Skills

Official examples: [github.com/anthropics/skills](https://github.com/anthropics/skills/tree/main/skills)

---

## 🔌 MCP Servers

*Verified from [official docs](https://code.claude.com/docs/en/mcp)*

Model Context Protocol connects Claude to external tools and APIs.

### Adding Servers

```bash
# HTTP transport
claude mcp add --transport http github https://api.githubcopilot.com/mcp/

# Stdio transport (local process)
claude mcp add --transport stdio airtable \
  --env AIRTABLE_API_KEY=xxx \
  -- npx -y airtable-mcp-server
```

### Managing

```bash
claude mcp list          # Show all servers
claude mcp get github    # Details
claude mcp remove github # Delete
```

---

## 🪝 Hooks

*Verified from [official docs](https://code.claude.com/docs/en/hooks)*

Shell commands that execute at specific points in Claude's workflow.

### Hook Events

| Event | When |
|-------|------|
| `PreToolUse` | Before tool calls (can block) |
| `PostToolUse` | After tool completes successfully |
| `PostToolUseFailure` | After tool fails |
| `PermissionRequest` | Permission dialog shown |
| `UserPromptSubmit` | User submits prompt |
| `Notification` | Claude sends notification |
| `Stop` | Claude finishes response |
| `SubagentStart` | Subagent task starts |
| `SubagentStop` | Subagent task completes |
| `SessionStart` | Session begins |
| `SessionEnd` | Session ends |
| `PreCompact` | Before conversation compact |

### Configuration

Use `/hooks` to configure interactively, or add to settings:

```json
{
  "hooks": {
    "PostToolUse": [{
      "matcher": "Edit|Write",
      "hooks": [{
        "type": "command",
        "command": "prettier --write \"$(jq -r '.tool_input.file_path')\""
      }]
    }]
  }
}
```

---

## 📊 Model Configuration

*Verified from [official docs](https://code.claude.com/docs/en/model-config)*

### Model Aliases

| Alias | Behavior |
|-------|----------|
| `default` | Account-dependent recommended model |
| `sonnet` | Latest Sonnet (currently 4.5) |
| `opus` | Opus (currently 4.5) |
| `haiku` | Fast, efficient for simple tasks |
| `sonnet[1m]` | Sonnet with 1M token context |
| `opusplan` | Opus for planning, Sonnet for execution |

### Setting Model

```bash
claude --model opus              # At startup
/model sonnet                    # During session
export ANTHROPIC_MODEL=opus      # Environment
```

### Environment Variables

| Variable | Purpose |
|----------|---------|
| `ANTHROPIC_MODEL` | Default model |
| `MAX_THINKING_TOKENS` | Thinking budget (overrides keywords) |
| `ANTHROPIC_DEFAULT_OPUS_MODEL` | Model for opus alias |
| `ANTHROPIC_DEFAULT_SONNET_MODEL` | Model for sonnet alias |
| `ANTHROPIC_DEFAULT_HAIKU_MODEL` | Model for haiku alias |
| `CLAUDE_CODE_SUBAGENT_MODEL` | Model for subagents |
| `DISABLE_PROMPT_CACHING` | Disable caching (1 to disable) |

---

## 🖥️ VS Code Integration

*Verified from [VS Code docs](https://code.claude.com/docs/en/vs-code)*

### Opening Claude

| Method | How |
|--------|-----|
| Spark icon | Top-right corner (when file is open) |
| Status bar | Click "✱ Claude Code" bottom-right |
| Command Palette | `Cmd+Shift+P` → "Claude Code" |
| Keyboard | `Cmd+Esc` (Mac) / `Ctrl+Esc` (Win) |

### VS Code Keyboard Shortcuts

| Shortcut | Action |
|----------|--------|
| `Cmd+Esc` / `Ctrl+Esc` | Toggle focus between editor and Claude |
| `Cmd+Shift+Esc` / `Ctrl+Shift+Esc` | Open new conversation in tab |
| `Cmd+N` / `Ctrl+N` | New conversation (when Claude focused) |
| `Alt+K` | Insert @-mention with file path + line numbers |

### Key Settings

`Cmd+,` → Extensions → Claude Code:

| Setting | What It Does |
|---------|--------------|
| **Use Ctrl+Enter to Send** | Enter = newline, Ctrl+Enter = send (multiline friendly) |
| **Use Terminal** | CLI mode instead of graphical panel |
| **Initial Permission Mode** | Default approval behavior |
| **Autosave** | Auto-save before Claude reads/writes |
| **Preferred Location** | Sidebar (right) or panel (tab) |

### VS Code Exclusive Features

- **Inline diff viewing** - See proposed changes with accept/reject
- **@-mention with line numbers** - Select text + `Alt+K` = precise file reference
- **Plan review** - Review and edit Claude's plans before executing
- **Resume CLI sessions** - `claude --resume` in integrated terminal picks up extension conversations

### What VS Code Can't Do (CLI Only)

| Feature | CLI | VS Code |
|---------|-----|---------|
| All slash commands | ✅ | Subset only |
| MCP server config | ✅ | No (configure in CLI) |
| `!` bash shortcut | ✅ | No |
| Tab completion | ✅ | No |
| Checkpoints/rewind | ✅ | Coming soon |

### JetBrains

- Install "Claude Code Beta" plugin
- `Cmd+Esc` / `Ctrl+Esc` to open
- Similar diff viewing and file reference features

---

## 💡 CLI Usage

*Verified from [CLI reference](https://code.claude.com/docs/en/cli-reference)*

```bash
claude                           # Start interactive REPL
claude "explain this project"    # Start with prompt
claude -p "query"                # Print mode (non-interactive)
claude -c                        # Continue most recent conversation
claude -r "session-name"         # Resume specific session
cat file | claude -p "explain"   # Pipe content
```

### Key Flags

| Flag | Purpose |
|------|---------|
| `--print, -p` | Print response, exit |
| `--continue, -c` | Continue last conversation |
| `--resume, -r` | Resume specific session |
| `--model` | Set model |
| `--permission-mode` | Set permission mode |
| `--append-system-prompt` | Add to system prompt |
| `--output-format` | text, json, or stream-json |
| `--max-turns` | Limit agentic turns |

---

## 🔗 Related Concepts

- [[Anthropic API]] - The underlying API
- [[LLM Under Your Floorboards]] - Local alternatives
- [[VS Code]] - Primary IDE integration

---

## 📚 External Resources

- [Claude Code Documentation](https://code.claude.com/docs/en/)
- [Slash Commands Reference](https://code.claude.com/docs/en/slash-commands)
- [Hooks Reference](https://code.claude.com/docs/en/hooks)
- [CLI Reference](https://code.claude.com/docs/en/cli-reference)
- [Model Configuration](https://code.claude.com/docs/en/model-config)
- [GitHub Issues](https://github.com/anthropics/claude-code/issues)
