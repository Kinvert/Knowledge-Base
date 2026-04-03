# OpenClaw

**OpenClaw** (formerly Clawdbot → Moltbot) is an open-source autonomous AI personal assistant that runs on your own devices and integrates with messaging platforms like WhatsApp, Telegram, Slack, Discord, Signal, and iMessage. Unlike chatbots that just generate text, OpenClaw is an **agent** that can execute shell commands, manage files, control browsers, send emails, and automate workflows across your digital life.

It gained explosive popularity in early 2026, hitting 100,000+ GitHub stars in three days. The project represents a shift toward **AI sovereignty**—your assistant, your machine, your rules. But that power comes with serious security implications that make it suitable primarily for advanced users.

For the hardware side of running your own AI infrastructure, see [[LLM Under Your Floorboards]].

---

## 🦞 The Naming Drama

| Name | Period | Why It Changed |
|------|--------|----------------|
| **Clawdbot** | Late 2025 | Original name (weekend hack that went viral) |
| **Moltbot** | Jan 2026 | Anthropic cease-and-desist over "Claude" similarity |
| **OpenClaw** | Jan 2026 | Final rebrand after chaos period |

The "lobster" motif (🦞) references molting—shedding an old shell to grow. The name changes happened within a single week, causing community whiplash. During the Moltbot transition, there were account hijackings, crypto scammers exploiting confusion, and exposed servers revealing security vulnerabilities.

---

## ⚙️ Core Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                      OpenClaw Gateway                        │
│                  (Node.js daemon on your machine)            │
├─────────────────────────────────────────────────────────────┤
│  WebSocket Control Plane: ws://127.0.0.1:18789              │
├──────────┬──────────┬──────────┬──────────┬────────────────┤
│ WhatsApp │ Telegram │  Slack   │ Discord  │ iMessage/etc.  │
│ (Baileys)│ (grammY) │  (API)   │  (API)   │ (imsg CLI)     │
├──────────┴──────────┴──────────┴──────────┴────────────────┤
│              LLM Backend (Anthropic/OpenAI/Local)           │
└─────────────────────────────────────────────────────────────┘
```

**Key components:**
- **Gateway**: Central daemon that routes messages and executes tools
- **Channels**: Bridges to messaging platforms
- **Agents**: Configurable AI personalities with tool permissions
- **Sessions**: Persistent conversation state with memory
- **Nodes**: Device integrations (browser, camera, screen capture)

---

## 📊 OpenClaw vs Claude Code vs Other Tools

| Feature | OpenClaw | Claude Code | OpenCode | Cursor |
|---------|----------|-------------|----------|--------|
| **Primary Focus** | Personal life automation | Code/dev tasks | Code/dev tasks | Code editing |
| **Interface** | Chat apps (WhatsApp, etc.) | Terminal | Terminal | IDE |
| **Voice** | Yes (ElevenLabs) | No | No | No |
| **Browser Control** | Yes (Chromium) | No | Limited | No |
| **Email/Calendar** | Yes | No | No | No |
| **File System** | Full access | Project-scoped | Project-scoped | Project-scoped |
| **Shell Commands** | Yes | Yes | Yes | Limited |
| **Multi-Platform** | macOS, iOS, Android, Linux | macOS, Linux, Windows | macOS, Linux | All |
| **Model Lock-in** | No (any LLM) | Claude only | Any LLM | Any LLM |
| **Cost** | Free + API usage | $17-100/mo + API | Free + API | $20/mo |
| **Self-Hosted** | Yes | No | Yes | No |
| **Memory/Persistence** | Yes | Limited | Yes | No |
| **Security Risk** | High | Medium | Medium | Low |

---

## 🔧 Installation

### Prerequisites

- Node.js ≥22 (required for WhatsApp/Telegram)
- macOS, Linux, or Windows (WSL2 recommended)
- API key for your chosen LLM provider

### Quick Start

```bash
# Install globally
npm install -g openclaw@latest

# Run onboarding wizard
openclaw onboard --install-daemon

# Start gateway
openclaw gateway --port 18789 --verbose
```

The wizard configures:
- Gateway as persistent service (launchd/systemd)
- Model provider (Anthropic, OpenAI, OpenRouter, etc.)
- Messaging channels (WhatsApp, Telegram, etc.)

### From Source

```bash
git clone https://github.com/openclaw/openclaw.git
cd openclaw
pnpm install
pnpm ui:build
pnpm build
openclaw onboard
```

### Platform Notes

| Platform | Notes |
|----------|-------|
| **macOS** | Menu bar app, Voice Wake, full feature set |
| **Linux** | Gateway + CLI, systemd service |
| **Windows** | WSL2 required; native Windows is problematic |
| **iOS** | Device node with Canvas, voice triggers |
| **Android** | Bridge pairing, Canvas, optional SMS |

---

## 📱 Supported Channels

| Channel | Method | Notes |
|---------|--------|-------|
| WhatsApp | Baileys (Web) | Scan QR via Linked Devices |
| Telegram | Bot API (grammY) | Create bot via @BotFather |
| Discord | Bot API | OAuth + bot token |
| Slack | Bot API | Workspace app installation |
| Signal | signal-cli | Requires phone number |
| iMessage | imsg CLI | macOS only |
| Microsoft Teams | Bot API | Enterprise setup |
| Matrix | matrix-js-sdk | Self-hosted compatible |
| Google Chat | API | Workspace required |
| WebChat | Built-in | Local web UI |

**WhatsApp/Telegram require Node.js** (not Bun—known issues).

---

## ⚙️ Configuration

Minimal config at `~/.openclaw/openclaw.json`:

```json5
{
  agent: {
    model: "anthropic/claude-opus-4-5"
  },
  gateway: {
    bind: "loopback",
    port: 18789,
    auth: {
      mode: "token",
      token: "your-secret-token"
    }
  },
  channels: {
    whatsapp: {
      dmPolicy: "pairing"
    },
    telegram: {
      token: "${TELEGRAM_BOT_TOKEN}"
    }
  }
}
```

### Model Support

| Provider | Models | Notes |
|----------|--------|-------|
| Anthropic | Claude Opus 4.5, Sonnet | Recommended for tool use |
| OpenAI | GPT-4o, o1 | Full support |
| OpenRouter | Any | Aggregator for many models |
| Google | Gemini | Supported |
| Local | Ollama, llama.cpp | Via OpenAI-compatible API |
| KIMI | Chinese models | Added post-rebrand |
| MiMo | Xiaomi models | Added post-rebrand |

**Recommendation**: Use Claude Opus 4.5 or GPT-4o for tool-enabled agents. Smaller models have weaker prompt injection resistance.

---

## 🔐 Security Model

### The "Lethal Trifecta" (Why OpenClaw is Dangerous)

OpenClaw combines three elements that create serious risk:

1. **Access to private data** (emails, files, messages)
2. **Exposure to untrusted content** (web pages, attachments, forwarded messages)
3. **Ability to take actions** (shell commands, send messages, browser control)

This means a prompt injection attack embedded in a webpage, email, or message can potentially:
- Exfiltrate your files and credentials
- Send messages as you
- Execute arbitrary commands
- Install malware

### DM Access Policies

| Policy | Behavior | Risk Level |
|--------|----------|------------|
| `pairing` (default) | Unknown senders get pairing code | Medium |
| `allowlist` | Block all unknown senders | Low |
| `open` | Allow anyone | **High** |
| `disabled` | Ignore inbound DMs | None |

### Sandbox Modes

```json5
sandbox: {
  mode: "all",           // Containerize all tool execution
  scope: "session",      // Per-session isolation (strictest)
  workspaceAccess: "ro"  // Read-only file access
}
```

| Mode | Description |
|------|-------------|
| `off` | No isolation (host execution) |
| `all` | All tools in containers |
| `scope: agent` | Per-agent containers |
| `scope: session` | Per-session containers (strictest) |
| `scope: shared` | Single container (least secure) |

### Tool Allowlists

```json5
tools: {
  allow: ["read", "sessions_list"],
  deny: ["write", "exec", "process", "browser"]
}
```

**High-risk tools to restrict:**
- `exec` - Shell command execution
- `browser` - Chromium control
- `web_fetch` / `web_search` - External content (injection vector)
- `write` / `edit` - File modification

### Hardening Checklist

- [ ] Bind to `loopback` only (never expose to network)
- [ ] Use `pairing` or `allowlist` DM policy
- [ ] Require mentions in group chats
- [ ] Enable sandbox mode for multi-user scenarios
- [ ] Set file permissions: `chmod 700 ~/.openclaw && chmod 600 ~/.openclaw/*.json`
- [ ] Use modern models (Opus 4.5+) for tool-enabled agents
- [ ] Run `openclaw security audit --deep` regularly
- [ ] Never use personal browser profile for browser control

### Security Audit

```bash
openclaw security audit          # Basic check
openclaw security audit --deep   # Comprehensive scan
openclaw security audit --fix    # Auto-apply safe defaults
```

---

## ⚠️ Known Security Incidents

| Issue | Description | Status |
|-------|-------------|--------|
| API key leaks | Plaintext credentials in early configs | Fixed (config encryption) |
| Exposed servers | Default open binding leaked data | Fixed (loopback default) |
| Malicious skills | ClawHub skills with prompt injection | Ongoing (skill review) |
| Crypto scammers | Fake repos during Moltbot transition | Resolved |

### Prompt Injection Mitigations

**Reader Agent Pattern**: Use a sandboxed read-only agent to summarize untrusted content before passing to main agent:

```json5
agents: {
  list: [
    {
      id: "reader",
      tools: { allow: ["read", "web_fetch"], deny: ["write", "exec"] },
      sandbox: { mode: "all", workspaceAccess: "ro" }
    },
    {
      id: "main",
      tools: { allow: ["*"], deny: ["web_fetch"] }
    }
  ]
}
```

---

## 💰 Cost Analysis

OpenClaw itself is free. You pay for API usage.

| Usage Level | Daily Cost | Monthly Cost | Notes |
|-------------|------------|--------------|-------|
| Light | $5-10 | $150-300 | Occasional queries |
| Medium | $15-25 | $450-750 | Regular assistant use |
| Heavy | $30-50+ | $900-1500+ | Constant automation |

**Cost drivers:**
- Model choice (Opus 4.5 > Sonnet > Haiku)
- Tool use (each tool call = more tokens)
- Context length (long conversations)
- Memory features (persistent context)

**Reduce costs by:**
- Using Haiku for simple tasks
- Limiting context window
- Disabling memory for ephemeral queries
- Using local models for non-critical tasks

---

## 🔧 Common Use Cases

| Use Case | Tools Needed | Risk Level |
|----------|--------------|------------|
| Smart inbox triage | Email read | Low |
| Calendar management | Calendar read/write | Low |
| Research assistant | web_search, web_fetch | Medium |
| Code review | read, exec (git) | Medium |
| File organization | read, write, exec | High |
| Browser automation | browser | High |
| Home automation | exec (API calls) | Medium |
| Multi-platform messaging | channel bridges | Medium |

---

## ✅ Strengths

- **True personal assistant**: Manages email, calendar, files, messages across platforms
- **Model agnostic**: Use Claude, GPT, local models, or mix them
- **Self-hosted**: No cloud dependency, full data ownership
- **Persistent memory**: Retains context across sessions
- **Voice enabled**: Wake words on macOS/iOS/Android
- **Multi-channel**: One assistant across all your chat apps
- **Open source**: 100k+ GitHub stars, active development
- **Extensible**: ClawHub skills, custom tools

---

## ❌ Weaknesses

- **Security nightmare**: Full system access + untrusted input = high risk
- **Complex setup**: Not for casual users
- **API costs**: Heavy use gets expensive ($30-50/day)
- **Node.js dependency**: WhatsApp/Telegram require Node, not Bun
- **WSL2 on Windows**: Native Windows support is poor
- **Prompt injection**: Not fully solved; model-dependent
- **Young project**: Rapidly changing, documentation gaps
- **Enterprise concerns**: Shadow IT risk, credential exposure

---

## 🔗 Related Concepts

- [[LLM Under Your Floorboards]] - Self-hosted LLM infrastructure
- [[Local LLMs]] - Running models locally
- [[LLM Inference Engines]] - llama.cpp, Ollama, vLLM
- [[Claude]] - Anthropic's model (recommended backend)
- [[GPT-4]] - OpenAI's model (alternative backend)
- [[Prompt Injection]] - Key security concern
- [[AI Agents]] - Autonomous AI systems
- [[LLM]] - Large Language Models overview
- [[Ollama]] - Local model server (can be backend)
- [[HuggingFace]] - Model source for local options

---

## 📚 External Resources

### Official

- [OpenClaw Website](https://openclaw.ai/)
- [OpenClaw Documentation](https://docs.openclaw.ai/)
- [GitHub Repository](https://github.com/openclaw/openclaw)
- [Security Documentation](https://docs.openclaw.ai/gateway/security)

### Guides

- [Getting Started](https://docs.openclaw.ai/start/getting-started)
- [Vultr Deploy Guide](https://docs.vultr.com/how-to-deploy-openclaw-autonomous-ai-agent-platform)
- [DigitalOcean One-Click Deploy](https://www.digitalocean.com/community/tutorials/how-to-run-openclaw)

### Analysis

- [Wikipedia - OpenClaw](https://en.wikipedia.org/wiki/OpenClaw)
- [Cisco Security Blog](https://blogs.cisco.com/ai/personal-ai-agents-like-openclaw-are-a-security-nightmare)
- [VentureBeat CISO Guide](https://venturebeat.com/security/openclaw-agentic-ai-security-risk-ciso-guide)
- [Fast Company Overview](https://www.fastcompany.com/91484506/what-is-clawdbot-moltbot-openclaw)
- [DEV.to - From Moltbot to OpenClaw](https://dev.to/sivarampg/from-moltbot-to-openclaw-when-the-dust-settles-the-project-survived-5h6o)

### Community

- [GitHub Discussions](https://github.com/openclaw/openclaw/discussions)
- r/LocalLLaMA (frequent OpenClaw discussion)
