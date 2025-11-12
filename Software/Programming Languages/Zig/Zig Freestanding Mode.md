# Zig — Freestanding Mode
A focused Obsidian reference about using Zig for **freestanding / bare-metal / no-OS** targets: what “freestanding” means in Zig, core concepts, build patterns, supported targets, common pitfalls, comparison to alternatives, and practical tips for embedded / OSDev work.

---

## ⚙️ Overview
Freestanding (aka bare-metal or `no-OS`) in Zig means targeting a platform where the runtime/OS abstractions (like POSIX, stdout, dynamic loaders, syscalls) are not available. Zig can compile to freestanding targets by selecting a `-target` triple that specifies `*-freestanding` (or a platform with no OS ABI), and by avoiding or replacing parts of `std` that assume an OS. Zig’s design (lazy top-level evaluation, allocator-param APIs, and explicit runtime control) makes freestanding development feasible and ergonomic.

---

## 🔎 Summary (tl;dr)
- Use `zig build`/`zig build-exe` with an appropriate `-target` (for example `riscv64-freestanding`, `x86_64-freestanding`, or `wasm32-freestanding`).
- Avoid `std` APIs that require an OS; prefer small parts of `std` that are OS-agnostic or provide your own substitutes (e.g., custom `Allocator`, simple console via UART).
- Many people use Zig for OS dev, Raspberry Pi / STM32 projects, and WebAssembly freestanding builds. Community libs aim to fill the libc gap but are evolving.

---

## 🧠 Core Concepts & Terminology
- **Target triple** — includes architecture, OS, and ABI (e.g., `aarch64-none-none` or `x86_64-freestanding` / Zig supports `*-freestanding` triples).
- **Freestanding** — no kernel syscalls, no standard I/O, no dynamic loader; you provide the entry point and hardware init.  
- **Allocator pattern** — many `std` containers accept an `Allocator` parameter so they can be used without a global OS allocator (important in freestanding).
- **Lazy top-level declarations** — Zig delays evaluating top-level code until used, which helps avoid accidental OS-dependent initialization in freestanding builds.

---

## 🛠 How It Works (practical steps)
1. **Choose target**: pass `-target <arch>-<os>-<abi>` where the OS/ABI indicates freestanding (examples: `riscv64-freestanding`, `x86_64-freestanding`, `wasm32-freestanding`).
2. **Entry point**: provide your own entry symbol (for example `pub export fn _start() callconv(.Naked) void { ... }`) and link script to place sections correctly.  
3. **Minimal runtime**: implement startup (stack, bss/data init), interrupt/exception vectors, and device init (UART, timers).  
4. **Avoid OS `std` features**: or reimplement required bits (you can still use many std utilities that are OS-agnostic).
5. **Linking**: typically use Zig’s built-in linker support or a custom `linker.ld`; for some toolchains you may combine Zig with `zig cc`/`clang` toolchains.

> Inline example commands (short, single-line examples only):  
> `zig build-exe src/main.zig -target riscv64-freestanding -O ReleaseSafe -mcpu=... -femit-bin`  
> `zig build-lib foo.zig -target wasm32-freestanding -dynamic`

---

## 🧩 Supported Targets & Use Cases
- **RISC-V / x86_64 / aarch64** — common for hobby OSes and small kernels.
- **ARM Cortex (bare metal Raspberry Pi / STM32)** — many community examples and tutorials exist.
- **WebAssembly freestanding** — compile Zig to `wasm32-freestanding` for sandboxed no-OS workloads.

---

## 📦 Standard Library & libc situation
- Zig’s `std` is partly usable in freestanding contexts: many containers/utilities are allocator-parameterized and can work without OS services; but parts of `std` expect I/O or OS primitives. Use only OS-agnostic modules or provide your own glue.
- There are community efforts (foundation libc, related repos) to provide a freestanding libc in Zig, but these projects are evolving and sometimes archived — check current status before relying on them.

---

## 🪲 Common Pitfalls & Troubleshooting
- **Accidental `std` usage** — linking will fail or bring in syscalls; audit imports and prefer explicit small helpers.
- **Linker scripts** — forgetting to define memory layout / stack and reset vectors causes runtime spin/crash. Always provide a tested linker script for your target.  
- **Toolchain/target mismatches** — double-check `-target` semantics (architecture bits, endianness, ABIs). Community threads show people mixing up `-target aarch64-none-none` vs `rpi`-style triples.

---

## ⚖ Comparison — Zig freestanding vs Others
| Aspect | Zig (freestanding) | C (`-ffreestanding` / bare-metal) | Rust (`no_std`) |
|---|---:|---|---|
| Language ergonomics | Modern syntax, comptime, safe-by-default opts | Ubiquitous, mature toolchain | Strong type safety, ecosystem for embedded |
| Stdlib support | Partial — allocator-parameterized APIs help | Minimal by default (libc optional) | Explicit `no_std`, large ecosystem (`embedded-hal`, `alloc`) |
| Tooling for cross-compilation | `zig` offers integrated cross-linking and `zig cc` helper | rely on `gcc/clang`, cross-toolchains | `rustc`+`cargo` with target specs |
| Community libs for bare-metal | Growing, fragmented; active discussion and issues | Massive (newlib, musl, etc) | Mature embedded crates (good tooling) |
Sources/examples: Zig docs, OSDev community posts.

---

## ✅ Strengths & Weaknesses
**Strengths**
- Powerful compile-time (`comptime`) features for generating data and avoiding runtime overhead.  
- Direct control over linking, no hidden runtime — good for minimal kernels and bootloaders.

**Weaknesses**
- Standard library is not a complete freestanding libc — you often need to implement basic OS primitives or use community crates which may be unstable.

---

## 🔧 Developer Tools & Workflow Tips
- Use `zig build` with explicit `-target` and `-mcpu` flags for cross-target builds.  
- Use `zig-ld`/built-in linker or provide a custom `linker.ld` for precise memory layout.  
- Combine Zig with QEMU for early testing (e.g., `qemu-system-*`) and use `gdb` with Zig’s debug info.  
- Follow community threads / GitHub issues for edge cases: the freestanding experience is actively being improved and discussed.

---

## 🔗 Related Notes (link style for Obsidian)
- - [[Cross-Compilation]] (tooling & triples)  
- - [[Linker Scripts]] (memory layout & vectors)  
- - [[Allocator Pattern]] (using/std containers in freestanding)  
- - [[RISC-V Bare Metal]] (example target)  
- - [[WebAssembly]] (wasm32-freestanding builds)  
- - [[Zig Build System]] (build.zig tips)

---

## 📚 Further Reading / References
- Zig language documentation & target reference.
- “Why Zig” / std design notes (allocator pattern & freestanding-friendly parts of std).
- Community OSDev / Bare Bones guides for Zig (practical examples & linker scripts).
- Recent GitHub issues discussing better freestanding support and libc efforts.

---
