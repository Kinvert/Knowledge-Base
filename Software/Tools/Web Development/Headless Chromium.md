---
aliases:
  - Headless Chrome
  - chromium --headless
  - --dump-dom
tags:
  - browser
  - scraping
  - automation
  - cli
title: Headless Chromium
---

# Headless Chromium 🕸️

**Headless Chromium** is the regular Chromium browser run from the command line with no visible window, exposing useful subcommands like `--dump-dom`, `--screenshot`, and `--print-to-pdf`. It is the foundation that [[Playwright]] and Puppeteer drive remotely — but for many quick tasks (rendered scraping, page captures, agent DOM inspection) you can skip those libraries entirely and just call the binary.

The flag that makes this surprisingly powerful is **`--dump-dom`**: it prints the fully rendered DOM *after* JavaScript has executed, unlike `curl` or `wget` which only see the static HTML the server sent.

---

## 🪄 The Headline Command

```bash
chromium --headless --no-sandbox --disable-gpu \
  --virtual-time-budget=8000 \
  --dump-dom http://127.0.0.1:3011/docs
```

What each flag does:

- `--headless` — no GUI; run as a background process
- `--no-sandbox` — needed when running as root or in containers (CI, Docker)
- `--disable-gpu` — historically required on headless Linux without a GPU; mostly cosmetic now
- `--virtual-time-budget=8000` — fast-forwards the page's clock up to 8000 ms so timers, animations, and async fetches resolve before dump
- `--dump-dom <URL>` — prints the post-JS DOM to stdout and exits

On modern Chrome you may also see `--headless=new` (the newer headless mode that shares more code with headed Chrome).

---

## 🧩 Why `--dump-dom` Is Useful

- **JavaScript-rendered scraping** without spinning up [[Playwright]] or Puppeteer
- **LLM/agent UI debugging** — paste the real rendered DOM into the chat so the model can see what the user sees, not the stub HTML
- **CI smoke tests** — grep the DOM for an expected string after a deploy
- **SSR / hydration verification** — confirm that server-rendered pages contain the expected markup
- **Quick diffs** — capture the DOM before and after a code change

Vs. `curl https://example.com`:

| | `curl` | `chromium --dump-dom` |
|---|---|---|
| Sees JS-rendered content | ❌ | ✅ |
| Runs the page's JS | ❌ | ✅ |
| Speed | Very fast | Slower (full browser) |
| Resource cost | Tiny | Hundreds of MB RAM per launch |
| Setup | None | Needs Chromium installed |

---

## 📊 Comparison Chart

| Tool | Layer | Best For | Overhead |
|---|---|---|---|
| `curl` / `wget` | Raw HTTP | Static HTML, APIs | Minimal |
| `chromium --dump-dom` | Browser CLI | One-shot rendered DOM | Moderate |
| `chromium --screenshot` | Browser CLI | One-shot PNG capture | Moderate |
| [[Playwright]] | Browser SDK | Interaction, multi-step flows, assertions | Higher |
| Puppeteer | Browser SDK | Same as Playwright (Chromium-only) | Higher |
| Selenium | Browser SDK | Cross-browser legacy automation | Higher |
| `wkhtmltopdf` | Webkit CLI | PDF rendering (older engine) | Moderate |
| Playwright MCP | MCP server | LLM agents driving a real browser | High |

---

## 🛠️ Other Useful CLI Modes

### Screenshot
```bash
chromium --headless --disable-gpu \
  --window-size=1280,1024 \
  --screenshot=out.png https://example.com
```

### PDF
```bash
chromium --headless --disable-gpu \
  --print-to-pdf=out.pdf https://example.com
```

### Pipe rendered DOM into a tool
```bash
chromium --headless --disable-gpu --dump-dom http://localhost:3000 \
  | grep -o 'data-test="[^"]*"' | sort -u
```

### Feed rendered DOM to an LLM via clipboard
```bash
chromium --headless --disable-gpu --dump-dom "$URL" | xclip -selection clipboard
```

---

## 🧠 Use Cases for AI Agents

- **"Why won't this div scroll?"** — dump the DOM and let the model inspect computed structure instead of guessing from source
- **Frontend regression diffs** — agent dumps DOM before and after a change, runs `diff`
- **RAG over JS-heavy docs sites** — pre-render before chunking so the index isn't full of `<div id="root"></div>` skeletons
- **Lightweight alternative to Playwright MCP** when no interaction is needed
- **CI checks** — fail the build if a critical selector is missing post-render

---

## ✅ Strengths

- Zero extra dependencies if Chromium/Chrome is already installed
- Output is just text on stdout — pipe-friendly
- Honors the actual browser engine (no rendering quirks)
- Great fit for shell scripts and Makefiles
- Faster to set up than [[Playwright]] for one-off tasks

---

## ❌ Weaknesses

- No interaction (no clicking, typing, scrolling) — for that, use [[Playwright]] or Puppeteer
- Heavyweight per launch — each invocation spawns a full browser
- `--virtual-time-budget` does not always wait for arbitrary network idle; tricky pages may need real Playwright
- Flags occasionally change between Chrome versions (`--headless=new` vs `--headless`)
- Authentication-gated pages need cookies or a user-data-dir to log in first
- Not a replacement for full E2E test runners

---

## 🔧 Common Flags

| Flag | Purpose |
|---|---|
| `--headless` / `--headless=new` | No visible window |
| `--no-sandbox` | Required as root/in containers |
| `--disable-gpu` | Skip GPU init on Linux servers |
| `--virtual-time-budget=N` | Fast-forward timers up to N ms |
| `--dump-dom URL` | Print rendered DOM to stdout |
| `--screenshot[=path]` | Save PNG of viewport |
| `--print-to-pdf[=path]` | Save PDF of page |
| `--window-size=W,H` | Set viewport size |
| `--user-data-dir=DIR` | Use a profile directory (cookies, sessions) |
| `--disable-extensions` | Skip extensions for clean runs |
| `--hide-scrollbars` | Cleaner screenshots |
| `--user-agent="..."` | Override UA string |

---

## 📚 Related Concepts / Notes

- [[Playwright]] (Higher-level browser automation built on the same engine)
- [[Web Development Tools]] (General tooling overview)
- [[Service Workers]] (Often part of what `--dump-dom` actually renders)
- [[PWA]] (Heavily JS-rendered apps that benefit from `--dump-dom`)
- [[REST API]] (When `curl` is sufficient instead)
- [[Electron]] (Also Chromium-based, related ecosystem)
- [[WASM]] (Sometimes the reason a page needs JS execution to render)
- [[Cloudflare Workers]] (Edge SSR — useful to verify with `--dump-dom`)

---

## 🌐 External Resources

- Chromium command-line switches list: https://peter.sh/experiments/chromium-command-line-switches/
- Chrome Headless announcement (still a great primer)
- `--virtual-time-budget` Chromium design doc
- Puppeteer and [[Playwright]] both expose the same flags via API

---

## 🔑 Key Highlights

- `chromium --dump-dom URL` prints the **post-JavaScript** DOM — far more useful than `curl` for modern web apps
- Pair with `--virtual-time-budget` so async work resolves before the dump
- Pipe-friendly: feed straight into `grep`, `diff`, an LLM, or a clipboard
- For interaction (clicks, forms, multi-page flows), graduate to [[Playwright]]
