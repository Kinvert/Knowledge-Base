# IDA Pro

IDA Pro (Interactive DisAssembler Professional) is the industry-standard commercial disassembler and debugger developed by Hex-Rays. It transforms compiled binaries back into assembly code and, with the Hex-Rays decompiler add-on, produces human-readable pseudocode. IDA is the go-to tool for malware analysts, vulnerability researchers, and reverse engineers working across virtually any CPU architecture.

---

## ⚙️ Overview

IDA Pro supports 40+ processor architectures out of the box (x86, ARM, MIPS, PowerPC, RISC-V, etc.) and can be extended to support custom or obscure architectures via processor modules. The software maintains an **IDA Database (IDB)** that stores all analysis results, annotations, and user modifications, allowing collaborative and iterative reverse engineering sessions.

Originally developed by Ilfak Guilfanov in the early 1990s, IDA has evolved into a comprehensive binary analysis platform. Hex-Rays, the company behind IDA, also produces the Hex-Rays Decompiler which generates C-like pseudocode from disassembly.

---

## 🧠 Core Concepts

- **Disassembly:** Translates machine code into human-readable assembly instructions.
- **Decompilation:** The Hex-Rays decompiler converts assembly into C-like pseudocode (separate license).
- **IDA Database (IDB):** Stores all analysis data, comments, names, types, and structures.
- **FLIRT Signatures:** Fast Library Identification and Recognition Technology matches known library functions automatically.
- **Type Libraries (TIL):** Pre-built type information for standard libraries and APIs (Windows SDK, POSIX, etc.).
- **Processor Modules:** Plugins that add support for new CPU architectures.
- **Loaders:** Handle different executable formats (PE, ELF, Mach-O, raw binary, etc.).
- **IDAPython:** Python scripting interface for automation and plugin development.
- **IDC:** Legacy C-like scripting language (IDAPython preferred for new work).

---

## 🛠️ How It Works

1. **Load Binary:** Select the target file; IDA auto-detects format and architecture.
2. **Auto-Analysis:** IDA performs initial disassembly, identifies functions, applies signatures.
3. **Navigate:** Use the IDA View (disassembly), Hex View, Strings, Imports/Exports windows.
4. **Decompile:** Press F5 to invoke Hex-Rays decompiler on the current function.
5. **Annotate:** Rename functions/variables, add comments, define structures.
6. **Debug:** Attach to running process or use remote debugging for dynamic analysis.
7. **Export:** Generate reports, pseudocode files, or use scripts to extract data.

---

## ⚙️ Key Features

- **Multi-Architecture:** 40+ processors supported natively.
- **Hex-Rays Decompiler:** Industry-leading decompilation to C pseudocode.
- **Integrated Debugger:** Local and remote debugging (Windows, Linux, macOS, iOS, Android).
- **IDAPython:** Full Python 3 scripting for automation and plugins.
- **Graph View:** Visualize control flow graphs of functions.
- **Proximity View:** Explore function call relationships.
- **Collaborative Analysis:** Team server for shared IDB databases.
- **Lumina:** Cloud-based function signature sharing service.
- **Extensible:** Large plugin ecosystem and SDK for custom development.

---

## 📊 Comparison Chart

| Tool | Type | GUI | Cost | Decompiler | Architectures | Best For |
|------|------|-----|------|------------|---------------|----------|
| **IDA Pro** | Commercial | ✅ Yes | $$$ | ✅ Hex-Rays (paid add-on) | 40+ | Professional RE, malware analysis |
| **[[Ghidra]]** | Free/OSS | ✅ Yes | Free | ✅ Built-in | 30+ | General RE, budget-conscious |
| **[[Binary Ninja]]** | Commercial | ✅ Yes | $$ | ✅ Built-in | 20+ | Modern workflows, BNIL IR |
| **[[Radare2]]** | Free/OSS | CLI + GUI | Free | ⚙️ Via plugins | 40+ | Scriptable analysis, CTF |
| **[[Cutter]]** | Free/OSS | ✅ Yes | Free | ⚙️ Via Ghidra/r2dec | (via r2) | Radare2 with friendly GUI |
| **Hopper** | Commercial | ✅ Yes | $ | ✅ Built-in | x86/ARM | macOS/iOS reversing |
| **[[objdump]]** | Free | ❌ CLI | Free | ❌ No | (via BFD) | Quick disassembly, scripting |

---

## 💰 Licensing & Editions

| Edition | Price Range | Features |
|---------|-------------|----------|
| **IDA Free** | Free | x86/x64 only, no decompiler, limited features |
| **IDA Home** | ~$365/year | Personal use, most architectures, no commercial use |
| **IDA Pro** | ~$1,900+ | Full commercial license, all architectures |
| **Hex-Rays Decompiler** | +$2,700+ per arch | Add-on for pseudocode generation |
| **IDA Teams** | Enterprise pricing | Collaboration features, Lumina, team server |

*Prices approximate and subject to change.*

---

## 🎯 Use Cases

- **Malware Analysis:** Reverse engineer malware samples to understand behavior and extract IOCs.
- **Vulnerability Research:** Find security bugs in closed-source software.
- **CTF Competitions:** Solve reverse engineering challenges.
- **Firmware Analysis:** Reverse engineer embedded device firmware.
- **Software Auditing:** Analyze third-party binaries for security reviews.
- **Game Modding:** Understand game binaries for modification.
- **Compatibility Research:** Understand undocumented file formats or protocols.

---

## ✅ Strengths

- Industry standard with 30+ years of development
- Best-in-class decompiler output (Hex-Rays)
- Massive plugin ecosystem
- Extensive architecture support
- Mature and stable codebase
- Strong documentation and community
- Professional support from Hex-Rays

---

## ❌ Weaknesses

- Expensive (especially with decompiler add-ons)
- Steep learning curve for beginners
- License restrictions (node-locked, no cloud deployment)
- UI can feel dated compared to newer tools
- Free version is very limited
- Some features require additional purchases

---

## 🧩 Compatible Items & Plugins

- **[[GDB]]** / **[[LLDB]]** — Remote debugging backends
- **[[YARA]]** — Pattern matching integration
- **Findcrypt** — Identify cryptographic constants
- **Diaphora** — Binary diffing and comparison
- **Keypatch** — Patch binaries with Keystone assembler
- **LazyIDA** — Productivity shortcuts
- **IDA Signsrch** — Signature searching
- **BinDiff** — Google's binary comparison tool
- **FRIEND** — Improve decompiler output readability
- **[[Python]]** — Scripting via IDAPython

---

## 🔧 Cheatsheet — Common Hotkeys

| Key | Action |
|-----|--------|
| `G` | Go to address |
| `N` | Rename item |
| `Y` | Set type |
| `X` | Cross-references to |
| `Ctrl+X` | Cross-references from |
| `F5` | Decompile (Hex-Rays) |
| `Space` | Toggle graph/text view |
| `Esc` | Go back |
| `Ctrl+Enter` | Go forward |
| `C` | Convert to code |
| `D` | Convert to data |
| `U` | Undefine |
| `P` | Create function |
| `;` | Add comment |
| `Shift+;` | Add repeatable comment |
| `/` | Add comment in pseudocode |
| `Alt+T` | Text search |
| `Alt+B` | Binary search |
| `Ctrl+S` | Save database |
| `Ctrl+F` | Quick filter in lists |

---

## 🔗 Related Concepts

- [[Ghidra]] (NSA's free alternative)
- [[Radare2]] (Open-source RE framework)
- [[Binary Ninja]] (Modern commercial disassembler)
- [[Cutter]] (GUI for Radare2)
- [[GDB]] (Debugger, integrates with IDA)
- [[objdump]] (CLI disassembler)
- [[YARA]] (Pattern matching for malware)
- [[Reverse Engineering]] (General concept)
- [[Decompilation]] (Converting binary to source-like code)
- [[Malware Analysis]] (Primary use case)

---

## 🌐 External Resources

- **Official Website:** https://hex-rays.com/ida-pro/
- **Documentation:** https://hex-rays.com/documentation/
- **IDAPython Docs:** https://hex-rays.com/products/ida/support/idapython_docs/
- **Plugin Repository:** https://github.com/onethawt/idaplugins-list
- **Hex-Rays Blog:** https://hex-rays.com/blog/
- **IDA Support:** https://hex-rays.com/support/

---

## 📚 Summary

IDA Pro remains the gold standard for professional reverse engineering, offering unmatched architecture support, the industry's best decompiler (Hex-Rays), and a mature plugin ecosystem. While expensive and with a steep learning curve, it's the tool of choice for malware analysts, vulnerability researchers, and anyone doing serious binary analysis. For budget-conscious users or those learning RE, [[Ghidra]] offers a capable free alternative.
