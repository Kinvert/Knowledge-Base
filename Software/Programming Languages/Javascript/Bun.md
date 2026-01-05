# Bun

**Bun** is an all-in-one [[JavaScript]] runtime, bundler, transpiler, and package manager built from scratch in Zig. It aims to replace [[Node.js]], Webpack, and npm with a single fast tool. Bun runs [[TypeScript]] and JSX natively without configuration, and is designed for speed with startup times and throughput significantly faster than Node.js.

---

## 📚 Overview

Bun was created by Jarred Sumner and released in 2022 as a performance-focused alternative to the Node.js ecosystem. Rather than building on V8 like Node, Bun uses JavaScriptCore (Safari's engine) and is written in Zig for low-level performance control.

Key highlights:
- Native TypeScript/JSX execution (no build step)
- Drop-in Node.js compatibility for most packages
- Built-in bundler, test runner, and package manager
- SQLite driver and other APIs built-in
- Extremely fast cold starts and package installs

---

## 🧠 Core Concepts

- **Runtime**
  Executes JS/TS files directly: `bun run app.ts`

- **Package Manager**
  npm-compatible but much faster: `bun install`

- **Bundler**
  Built-in bundling for production: `bun build`

- **Test Runner**
  Jest-compatible testing: `bun test`

- **Transpiler**
  Converts TypeScript/JSX on the fly without config

- **Node.js Compatibility**
  Implements Node APIs (fs, path, http, etc.) for drop-in replacement

---

## 📊 Comparison Chart

| Runtime | Engine | TypeScript | Package Manager | Bundler | Speed |
|---------|--------|------------|-----------------|---------|-------|
| **Bun** | JavaScriptCore | Native | Built-in | Built-in | Very fast |
| **Node.js** | V8 | Via transpiler | npm/yarn/pnpm | External | Moderate |
| **Deno** | V8 | Native | Built-in (URL imports) | Built-in | Fast |
| **Cloudflare Workers** | V8 | Via build | Wrangler | Wrangler | Fast (edge) |
| **txiki.js** | QuickJS | No | No | No | Lightweight |
| **Just-js** | V8 | No | No | No | Minimal |

---

## 🔧 Use Cases

- Fast development servers and hot reloading
- TypeScript projects without build configuration
- Rapid prototyping and scripting
- CI/CD pipelines (fast installs)
- Game development with [[TypeScript]] engines
- REST APIs and web backends
- CLI tools and build scripts

---

## ✅ Pros

- Dramatically faster than Node.js for many operations
- Native TypeScript without tsconfig or build step
- Single tool replaces Node + npm + bundler + test runner
- npm-compatible (most packages work)
- Modern APIs and DX improvements
- Active development and growing ecosystem

---

## ❌ Cons

- Younger ecosystem than Node.js
- Some Node.js APIs not yet implemented
- Fewer production deployments and battle-testing
- Some npm packages with native bindings may not work
- JavaScriptCore differences from V8 can cause edge cases

---

## 🔧 Common Commands

```bash
# Run a file
bun run app.ts

# Install dependencies (reads package.json)
bun install

# Add a package
bun add express

# Remove a package
bun remove lodash

# Run tests
bun test

# Bundle for production
bun build ./src/index.ts --outdir ./dist

# Start a script from package.json
bun run dev
```

---

## 🔩 Compatible Items

- [[TypeScript]] - Native execution
- [[npm]] - Compatible package registry
- [[Node.js]] - API compatibility layer
- [[Express]] - Works with Bun runtime
- [[SQLite]] - Built-in driver
- [[Jest]] - Test API compatibility
- [[Webpack]] - Can replace for bundling

---

## 🔗 Related Concepts

- [[Node.js]] (Primary alternative)
- [[TypeScript]] (Native support)
- [[Deno]] (Similar modern runtime)
- [[JavaScript]] (Base language)
- [[Zig]] (Implementation language)

---

## 📚 External Resources

- [Bun Official Site](https://bun.sh/)
- [Bun Documentation](https://bun.sh/docs)
- [Bun GitHub](https://github.com/oven-sh/bun)
- [Bun Discord](https://bun.sh/discord)
