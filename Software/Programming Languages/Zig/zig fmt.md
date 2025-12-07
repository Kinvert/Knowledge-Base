# zig fmt

`zig fmt` is Zig’s built-in automatic code formatter. It enforces a consistent, official, canonical style for all Zig code. Unlike formatters in many other languages, `zig fmt` is **not configurable**—there is only one correct formatting style for Zig programs. This design ensures uniform codebases across all projects, eliminates bikeshedding, and guarantees that Zig code is readable and stylistically predictable everywhere.

---

## 🧭 Overview

The `zig fmt` tool parses Zig source files using the official compiler frontend and rewrites them with standardized indentation, spacing, line wrapping, and layout. Because it uses the Zig parser, it is always correct with respect to Zig semantics (i.e., no heuristic-driven formatting). Developers run it manually, integrate it into CI, or rely on editor integrations for on-save formatting.

---

## 🧩 Core Concepts

- **Canonical Formatting**  
  Zig enforces one universal style. There are no `.editorconfig`, no knobs, no preferences—just the standard.

- **Compiler-Integrated**  
  Uses Zig’s actual parser, so formatting is stable and semantically aware.

- **Idempotent**  
  Running `zig fmt` multiple times yields identical results.

- **Zero Configuration**  
  Projects do not diverge stylistically.

- **Isolated Changes**  
  Formatter updates only occur on Zig version changes; this ensures reproducibility.

---

## ⚖️ Comparison Chart

| Tool | Language | Configurable? | Guarantees | Notes |
|------|----------|---------------|------------|-------|
| **zig fmt** | Zig | ❌ | Canonical, stable | Always correct, no style debates |
| gofmt | Go | ❌ | Canonical | Similar philosophy; inspiration |
| rustfmt | Rust | ⚠️ Semi-configurable | Mostly stable | Larger configuration surface |
| clang-format | C/C++ | ✔️ Highly configurable | Heuristic-based | Can lead to inconsistent styles |
| black (Python) | Python | ⚠️ Limited options | Mostly stable | Philosophy similar but not compiler-native |

---

## 🔧 How It Works

- The formatter reads one or more input `.zig` files.  
- It parses the file into Zig’s AST (Abstract Syntax Tree).  
- The AST is re-emitted using Zig’s canonical formatting rules.  
- The file is overwritten unless `--stdin` or `--stdout` modes are used.

Typical commands:

- `zig fmt .` formats all Zig files in the current directory.  
- `zig fmt src/` formats recursively within a directory.  
- Editor integrations often call: `zig fmt --stdin --stdout` for streaming.

---

## 🧪 Use Cases

- Enforcing style in CI for Zig projects  
- Removing formatting debates from teams  
- Keeping large Zig codebases clean  
- Ensuring diffs are meaningful (no arbitrary whitespace changes)  
- Autoformatting on save in IDEs/editors  

---

## 🏆 Strengths

- **Zero bikeshedding**: There is only one format.  
- **Compiler-accurate**: Uses the official parser.  
- **Predictable**: No random decisions; idempotent output.  
- **Fast**: Optimized and lightweight.  
- **Ubiquitous**: Every Zig project uses it.

---

## ⚠️ Weaknesses

- No customization (by design; may frustrate teams wanting their own style).  
- Updates tied to Zig versions may cause repo-wide formatting diffs.  
- Not applicable to languages outside Zig.

---

## 🧰 Developer Tools

- Editor integrations for VS Code, Vim, Neovim, Helix, Emacs  
- CI workflows using `zig fmt --check`  
- Pre-commit hooks  
- Integration in build systems (e.g., calling `zig fmt` before packaging)

---

## 🔗 Related Concepts

- [[Zig]]  
- [[ELF]] (Zig emits excellent DWARF/ELF metadata)  
- [[DWARF]]  
- [[Compilers]]  
- [[Formatting Tools]]  
- [[Static Analysis]]  

---

## 🌐 External Resources

- Zig Lang reference  
- Zig documentation (`zig help fmt`)  
- Official Zig repository and PRs related to formatting  
- Zig community style discussions (rare due to fixed format)

---

## 📌 Summary

`zig fmt` is an official, single-style, compiler-backed formatter that ensures every Zig codebase looks the same. It eliminates configuration overhead, guarantees correctness by parsing the true Zig AST, and promotes clarity and consistency across the entire Zig ecosystem. For any Zig project, `zig fmt` is not optional—it is the standard way to write Zig code.
