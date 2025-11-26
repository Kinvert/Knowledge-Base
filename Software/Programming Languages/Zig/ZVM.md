# ZVM (Zig Version Manager)

ZVM, the Zig Version Manager, is a tool designed to simplify managing multiple versions of the Zig programming language on a single system. Similar to version managers for Python (pyenv), Node.js (nvm), or Rust (rustup), ZVM allows developers to install, switch, and isolate Zig versions for different projects. This is particularly important for Reinforcement Learning engineers and systems developers who may need to target different Zig releases for stability, features, or compatibility with third-party libraries.

---

## 🧠 Overview

ZVM addresses a common pain point in systems programming: maintaining multiple Zig versions without polluting the system environment. Developers often face challenges when:
- A project depends on a specific Zig release
- Newer Zig releases introduce breaking changes
- Continuous integration or reproducible builds require controlled Zig versions

ZVM provides a user-centric interface for these scenarios:
- Install any Zig version easily
- Switch between versions globally or per-project
- Integrate seamlessly with shell environments
- Maintain reproducible builds across environments

---

## ⚙️ Core Concepts

### Version Installation
- Install specific Zig releases from official channels or GitHub releases.
- Optionally, install development builds or nightly versions.
- Handles architecture-specific binaries automatically.

### Switching Versions
- Global version: sets the default Zig compiler for all shells.
- Local version: per-directory version overrides for project-specific builds.
- Temporary version: session-specific version for testing or experimentation.

### Path Management
- Updates `PATH` environment variables transparently.
- Avoids conflicts with system-installed Zig binaries.
- Works with multiple shells (bash, zsh, fish).

### Isolation
- Ensures projects using different Zig versions do not interfere with one another.
- Helps enforce deterministic build environments for CI/CD pipelines.

---

## 🔁 Workflow

1. Install a Zig version: `zvm install 0.11.0`
2. Set global default: `zvm global 0.11.0`
3. Set project-specific version: `zvm local 0.10.1`
4. Verify active version: `zvm current`
5. Use temporary version for testing: `zvm use 0.12.0`

ZVM also integrates with build systems via environment variables, making it CI/CD-friendly.

---

## 🧩 Comparison to mise

mise is a minimalist Zig version manager designed for speed and simplicity. Here’s how ZVM compares:

| Feature | ZVM | [[mise]] |
|---------|-----|------|
| Version Installation | Yes, official and nightly | Yes, lightweight |
| Global/Local Versions | Yes | Only per-project or per-shell (simpler) |
| Shell Integration | Bash, zsh, fish | Bash, zsh |
| Nightly Support | Yes | Partial |
| CI/CD Friendly | Strong | Moderate |
| Architecture Detection | Automatic | Manual in some cases |
| Ease of Use | User-friendly | Ultra-minimal, fewer commands |
| Documentation | Full | Sparse |
| Active Maintenance | Moderate | Minimal |
| Community Adoption | Growing | Niche |

ZVM is more feature-complete and CI/CD-oriented, while mise targets minimalism and speed for developers who don’t need global version management.

---

## 🛠️ Developer Tools

- CLI: `zvm` command-line interface
- Shell integration scripts
- Version caching and download management
- Compatible with Zig Build System for version-specific builds

---

## 📚 Documentation and Support

- ZVM GitHub repository
- Community discussion channels
- Tutorials on version management and CI/CD integration
- Shell integration guides

---

## ✅ Key Highlights

- Simplifies multi-version Zig environments
- Supports global, local, and temporary versions
- Integrates with shells and CI/CD pipelines
- Reduces build conflicts and enhances reproducibility

---

## 🎯 Use Cases

- Managing projects with strict Zig version requirements
- Testing new Zig features without affecting stable environments
- CI/CD pipelines for RL environments, simulation engines, or embedded systems
- Developers contributing to Zig language or compiler infrastructure

---

## 📌 Related Concepts / Notes

- [[Zig]] (Programming Language)
- [[mise]] (Minimal Zig Version Manager)
- [[CI-CD]] (Continuous Integration and Continuous Delivery)
- [[Comptime]] (Compile-Time Execution)
- [[Build Systems]]
- [[Version Management]]
- [[Environment Isolation]]

---

## 📦 Compatible Items

- Linux
- macOS
- Windows (via WSL or compatible binaries)
- Zig supported architectures (x86_64, ARM, etc.)

---

## 🧭 Summary

ZVM brings structured, user-friendly version management to Zig, making it easier to maintain reproducible environments and switch between multiple versions for different projects. Compared to mise, ZVM is more feature-rich and CI/CD-ready, while mise is lightweight and minimal. For Reinforcement Learning, robotics, or systems programming projects where Zig versions may vary across environments or CI pipelines, ZVM ensures stability, flexibility, and developer productivity.
