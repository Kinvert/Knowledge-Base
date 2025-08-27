# GDB (GNU Debugger)

GDB (GNU Debugger) is the canonical command-line debugger for compiled languages on Unix-like systems (C, C++, Fortran, Rust, etc.). It lets you inspect program state (stack, variables, registers, memory), control execution (breakpoints, stepping, watchpoints), debug core dumps, attach to running processes, and script complex workflows. GDB is extensible (Python), integrates with `gdbserver` for remote/embedded targets, and is the backbone for many GUI/debugger front-ends and IDE integrations.

---

## 🔭 Overview

GDB attaches to an "inferior" (the program being debugged) and controls it via OS facilities (typically `ptrace`), debug symbols (DWARF), and dynamic loader hooks. It supports source-level debugging, assembly-level inspection, multi-threaded programs, remote debugging, and scripting. Modern workflows often combine GDB with helpers like `gdbserver`, `pwndbg` / `gef`, `gdb-dashboard`, or record-and-replay tools like `rr` for reverse debugging.

---

## 🧠 Core Concepts

- **Inferior:** the program being debugged (local, remote via `gdbserver`, or a core file).  
- **Breakpoints:** stop execution at a location; can be conditional.  
- **Watchpoints:** stop when a memory location/value changes (hardware vs software watchpoints).  
- **Frames & backtrace:** function call stack; `frame`, `bt`, `up`, `down`.  
- **Registers & memory:** inspect via `info registers`, `x/` commands and modify with `set $reg=` / `set var`.  
- **Symbols & debug info:** DWARF symbols (`-g` at compile time) enable variable names, types, and line mappings.  
- **Non-stop & multi-thread:** control per-thread execution.  
- **Remote debugging:** `gdbserver` or remote remote targets (`target remote <host>:port`).  
- **Scripting & pretty-printers:** embed Python for automation and custom pretty-printers (useful for STL containers, etc.).  
- **Reverse debugging / recording:** either GDB's internal `record` (limited) or `rr` for robust reverse execution.

---

## 🛠️ How it works (short)

- GDB uses OS debug syscalls (`ptrace` on Linux) to stop the inferior, read/write its memory and registers, and resume execution.  
- Breakpoints are normally implemented by writing a trap instruction (software breakpoint) at the target address; hardware breakpoints use CPU debug registers.  
- Source mapping requires debug symbols; without them you operate on addresses and disassembly.  
- For remote/embedded targets, `gdbserver` runs on the target and GDB connects over TCP/serial to control it.

---

## ⚙️ Key Features

- Source- and assembly-level stepping (`step`, `next`, `finish`).  
- Conditional & function breakpoints (`break foo.c:42 if x>5`).  
- Watchpoints (data breakpoints) `watch var`.  
- Thread control (`thread`, `info threads`, `thread apply all bt`).  
- Core-file debugging (`gdb ./a.out core`), postmortem analysis.  
- Remote debugging via `gdbserver` and `target remote`.  
- Python scripting and plugin ecosystem (`pwndbg`, `gef`, `gdb-dashboard`).  
- Symbol loading/unloading, shared library handling (`set solib-search-path`).  
- Reverse debugging with `record` (and `rr` integration for robust record/replay).  
- Support for cross-architecture builds via `gdb-multiarch` and target-specific GDBs (`arm-none-eabi-gdb`).

---

## 📊 Comparison Chart — GDB vs common alternatives

| Tool | Type | Interactive? | Primary focus | Strengths | Weaknesses |
|---|---:|:---:|---|---|---|
| **[[GDB]]** | Native debugger | Yes | General-purpose source + assembly debugging | Ubiquitous, extensible, remote/embedded support, DWARF-aware | UI is CLI; can be slower for complex reverse debugging without `rr` |
| **[[LLDB]]** | LLVM debugger | Yes | Modern debugger with better integration to LLVM toolchain | Faster startup for some tasks, modern APIs, good Xcode integration | Different command syntax; less mature ecosystem on Linux historically |
| **[[rr]]** | Record & replay tool | Mostly CLI | Deterministic replay & reverse debugging | Robust reverse debugging, deterministic replay | Requires recording session; only Linux x86_64 supported well |
| **[[valgrind]] (memcheck)** | Dynamic analysis tool | No / limited | Memory errors/leak detection | Finds memaccess bugs, leaks, undefined behavior | Slow (heavy instrumentation), not a step debugger |
| **[[strace]] / ltrace** | Syscall / library call tracers | No | Observing syscalls / library calls | Lightweight, easy to see syscalls & args | Not for source-level debugging |
| **[[perf]] / oprofile** | Profiler | No | Sampling & performance tracing | Low-overhead profiling, hardware counters | Not interactive debugger; complementary |
| **gdbgui / cgdb / ddd / edb** | Graphical front-ends | Yes | GUI-based debugging | Easier visualization, breakpoints clicking | Extra layer — may lag behind gdb features |
| **kgdb / kgdboc** | Kernel debugger | Yes | Kernel debugging | Debug kernel via serial or network | Requires special kernel config and setup |

---

## 🧩 Variants & Compatible Items

- **GDB variants:** `gdb-multiarch`, `arm-none-eabi-gdb`, `mipsel-gdb`, etc.  
- **Remote helpers:** `gdbserver`, `openocd` (for JTAG/SWD), `qemu` with `-s -S`.  
- **Pretty-printers & plugins:** `pwndbg`, `gef`, `gdb-dashboard`, custom Python pretty-printers.  
- **Front-ends / IDEs:** VSCode (`Native Debug` / `C/C++`), CLion, Eclipse CDT, gdbgui, DDD, cgdb.  
- **Related tools:** `objdump`, `readelf`, `nm`, `addr2line`, `strace`, `ltrace`, `valgrind`, `perf`, `rr`.

---

## 🧾 Use Cases

- Interactive debugging of native C/C++/Rust programs.  
- Post-mortem core-dump analysis.  
- Remote debugging embedded devices via `gdbserver` / OpenOCD.  
- Security research (exploitation, reverse engineering) with `pwndbg`.  
- Debugging multi-threaded deadlocks & race conditions (with thread-aware commands).  
- Reproducing and bisecting hard-to-reproduce bugs using `rr` + reverse execution.

---

## ✅ Strengths

- Extremely flexible and scriptable (Python).  
- Works on nearly every POSIX system and many architectures.  
- Deep feature set: remote debugging, core analysis, multi-thread support.  
- Large ecosystem — many plugins and front-ends.  
- Integrates with toolchains and CI pipelines.

---

## ❌ Weaknesses

- Steep learning curve for newcomers.  
- CLI-only by default (GUIs help but add layers).  
- Reverse debugging built-in is limited on some builds — `rr` is often preferred.  
- Debugging stripped binaries or those without DWARF is harder (more assembly-level work).

---

## 🔧 Developer Tools & Integrations

- `gdbserver` — run on target for remote debugging.  
- `gdb-multiarch` / `arm-none-eabi-gdb` — cross-architecture GDB binaries.  
- `pwndbg`, `gef`, `gdb-dashboard` — enhance UX and add commands.  
- IDEs: VSCode (Native Debug / cpptools), CLion, Eclipse — use GDB as backend.  
- `gdbgui`, `ddd`, `cgdb`, `edb` — GUI wrappers for GDB.  
- Profilers & tracers: integrate `perf`, `valgrind`, `strace` for complementary analysis.

---

## 📚 Documentation & Support

- Official GDB project page: `https://www.gnu.org/software/gdb/`  
- GDB manual / online docs: `https://sourceware.org/gdb/current/onlinedocs/gdb/`  
- Tutorials & cheat-sheets: many community cheat-sheets (search "GDB cheat sheet")  
- Community: Stack Overflow, mailing lists (`gdb@sourceware.org`), GitHub repos for plugins.

---

## 🧾 Cheatsheet — One-Liners (ONLY single-tick ` like this `; commands are one-liners)

- `gdb ./a.out` — start GDB with the binary.  
- `gdb -q ./a.out` — start quietly (no intro).  
- `gdb -tui ./a.out` — start GDB with TUI (split source/console).  
- `gdb -ex "break main" -ex run --args ./a.out arg1 arg2` — set breakpoint then run with args.  
- `gdb -p 1234` — start GDB and attach to PID `1234`.  
- `gdb ./a.out core` — open `core` for postmortem analysis of `a.out`.  
- `gdbserver :1234 ./a.out` — run target under `gdbserver` listening on port `1234`.  
- `gdb ./a.out` then `target remote :1234` — connect GDB to remote `gdbserver` (can be combined with `-ex`).  
- `break main` — set breakpoint at function `main` (GDB internal command).  
- `break file.c:123` — breakpoint at `file.c` line `123`.  
- `break *0x400123` — breakpoint at absolute address `0x400123`.  
- `break foo.c:42 if x==0` — conditional breakpoint (stop only if condition true).  
- `watch var` — break when `var` is written to.  
- `rwatch var` — break when `var` is read.  
- `awatch var` — break on read/write (access watchpoint).  
- `run` — start program (inside GDB).  
- `continue` or `c` — continue execution after a stop.  
- `next` or `n` — step over (source-level).  
- `step` or `s` — step into (source-level).  
- `finish` — run until current function returns.  
- `stepi` / `si` — single-step one instruction (assembly).  
- `nexti` / `ni` — step one instruction, stepping over calls.  
- `bt` — backtrace (stack trace).  
- `frame 2` — select frame number `2`.  
- `up` / `down` — move up/down stack frames.  
- `info threads` — list threads.  
- `thread 3` — switch to thread `3`.  
- `info locals` — show local variables in current frame.  
- `info args` — show function arguments.  
- `print x` or `p x` — print variable `x`.  
- `p/x x` — print `x` in hexadecimal.  
- `ptype var` — show type of `var`.  
- `display i` — automatically print `i` each stop.  
- `undisplay 1` — remove display number `1`.  
- `set var x = 5` — change variable `x` to `5`.  
- `set $pc = 0x400123` — set program counter to address `0x400123`.  
- `info registers` or `i r` — show CPU registers.  
- `x/16xb &var` — examine 16 bytes at address of `var` (byte format).  
- `x/8gx 0x7ffd...` — examine 8 giant words at an address (64-bit hex).  
- `disassemble` — disassemble current function.  
- `x/20i $pc` — show 20 instructions at the current PC.  
- `info sharedlibrary` — list loaded shared libraries and their symbol status.  
- `set solib-search-path /path/to/libs` — set path to search for shared libraries.  
- `set follow-fork-mode child` — when program forks, follow the child.  
- `set follow-fork-mode parent` — follow the parent after fork.  
- `catch throw` — stop when a C++ exception is thrown.  
- `catch syscall` — catch syscalls (platform-dependent).  
- `set pagination off` — disable paging of long outputs.  
- `set logging file gdb.log` — set log file for GDB session.  
- `set logging on` — enable logging to the log file.  
- `maint info sections` — low-level info about binary sections.  
- `add-symbol-file ./libfoo.debug 0x400000` — load extra symbol file at address.  
- `source ~/.gdbinit` — load commands from a file.  
- `record` — begin recording program execution (if supported).  
- `record stop` — stop recording.  
- `reverse-continue` — continue execution backwards (requires recording/rr).  
- `attach 4321` — attach to process `4321`.  
- `detach` — detach from the inferior and let it continue.  
- `kill` — stop and kill the inferior.  
- `help` — show help topics (e.g., `help breakpoints`).  
- `quit` or `q` — exit GDB.

---

## 🔍 Practical Tips & Patterns

- Always compile with debug symbols: `gcc -g -O0` for easiest debugging.  
- Use `-Og` for a compromise between optimization and debuggability.  
- Prefer `gdb-multiarch` or a target-specific GDB for cross-architecture debugging.  
- Use `rr` to record a failing run and then use reverse execution to find the root cause.  
- Use pretty-printers for complex types (STL, Rust, etc.) via Python plugins.  
- Strip debug symbols from deployed binaries to protect IP, keep `.debug` files to load in GDB.

---

## 🔗 Related Concepts / Notes

- [[LLDB]] (LLVM Debugger) — alternative frontend for LLVM toolchain  
- [[rr]] (Record & Replay Debugging) — deterministic replay + reverse debugging  
- [[Valgrind]] — memory/debugging tool (memcheck, helgrind)  
- [[strace]] / [[ltrace]] — syscall / library-call tracing  
- [[perf]] — performance profiling and hardware counters  
- [[gdbserver]] — remote debugging server for embedded/remote targets  
- [[OpenOCD]] — JTAG/SWD interface for microcontrollers  
- [[DWARF]] — debug information format used by GDB

---

## 📚 Further Reading & Links

- GDB homepage: `https://www.gnu.org/software/gdb/`  
- GDB manual (online): `https://sourceware.org/gdb/current/onlinedocs/gdb/`  
- `gdb` quick reference / cheat sheets — many community variants; search “gdb cheat sheet”.  
- `pwndbg` (GDB plugin): `https://github.com/pwndbg/pwndbg`  
- `gef` (GDB Enhanced Features): `https://github.com/hugsy/gef`  
- `gdb-dashboard`: `https://github.com/cyrus-and/gdb-dashboard`  
- `rr` record & replay: `https://rr-project.org/`

---

## 🏁 Summary

GDB remains a fundamental tool for low-level debugging, forensic analysis, and embedded/remote debugging. Pair it with plugins (`pwndbg`, `gef`) and record/replay tools (`rr`) for advanced workflows, and use `gdbserver` or cross-compiled GDBs for embedded targets. Mastering the one-liners above will cover 95% of day-to-day debugging tasks; the rest is using Python scripting and plugins to automate the other 5%.

