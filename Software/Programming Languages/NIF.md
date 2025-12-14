# NIF (Native Implemented Function)

A *Native Implemented Function (NIF)* is a mechanism in the BEAM VM (Erlang/Elixir) that allows native code—typically written in C or C-compatible languages—to be called directly from Erlang or Elixir. NIFs provide extremely low-latency integration with native libraries but come with strict safety and scheduling constraints. They sit at the intersection of **FFI**, **ABI stability**, and **VM scheduling**, making them powerful but dangerous tools.

This topic is especially relevant for high-performance paths in reinforcement learning (RL), numerical kernels, cryptography, compression, and interoperability with existing C/C++/Zig/Python ecosystems.

---

## 📚 Overview

NIFs allow Elixir or Erlang code to invoke native functions *inside the BEAM VM process*. Unlike ports, NIFs do not involve IPC—they are direct function calls. This means:
- Near-zero call overhead
- Direct access to BEAM terms
- Severe consequences if misused (VM crash, scheduler starvation)

Because BEAM is a soft real-time VM, NIFs must be carefully designed to avoid blocking schedulers.

---

## 🧠 Core Concepts

- **NIF Module**: A shared library loaded by the BEAM VM.
- **BEAM Scheduler**: Executes both Elixir code and NIFs.
- **Dirty NIFs**: Special NIFs that run on separate schedulers.
- **Term Encoding**: Native code manipulates BEAM terms via APIs.
- **ABI Compatibility**: Native binaries must match BEAM’s expectations.
- **FFI Boundary**: Crossing from managed BEAM world to native code.

---

## 🔧 How NIFs Work (BEAM Perspective)

1. Elixir module declares NIF-backed functions.
2. At runtime, BEAM loads a shared library (`.so`, `.dylib`, `.dll`).
3. Native functions are registered with the VM.
4. Calls from Elixir jump directly into native code.
5. Native code receives BEAM terms and returns BEAM terms.

There is **no sandbox**. A segfault in a NIF crashes the entire VM.

---

## 🧠 BEAM Scheduling & Safety

- **Normal NIFs**
  - Run on standard schedulers
  - Must complete in microseconds
  - Blocking causes system-wide latency

- **Dirty CPU NIFs**
  - Run on dirty schedulers
  - Safe for longer CPU-bound work

- **Dirty IO NIFs**
  - Designed for blocking IO

Misclassifying a NIF can cause catastrophic performance degradation.

---

## 🔗 Relationship to FFI and ABI

NIFs are a *specialized FFI* tightly coupled to the BEAM VM.

| Layer | Role |
|-----|-----|
| **FFI** | Language-to-language boundary |
| **ABI** | Binary compatibility contract |
| **NIF API** | BEAM-specific FFI |
| **Scheduler Model** | Determines safety constraints |

NIFs rely on a stable C ABI exposed by the Erlang VM. Any language that can emit C-compatible binaries can theoretically implement NIFs.

---

## 🧩 Language Interoperability Deep Dive

### 🟦 C ↔ Elixir (Canonical NIF)

- C is the reference implementation language for NIFs
- Uses Erlang NIF API (`erl_nif.h`)
- Full access to BEAM term construction
- Manual memory management
- Highest risk, highest control

**Best for**: Performance-critical primitives, crypto, compression

---

### 🟨 Zig ↔ Elixir (Modern NIF Approach)

Zig is particularly well-suited for NIFs because:
- First-class C ABI compatibility
- No runtime or GC
- Explicit memory control
- Excellent safety compared to C

Zig can:
- Export C ABI functions
- Include `erl_nif.h`
- Produce `.so` files identical to C output
- Avoid undefined behavior common in C

**Pattern**:
- Zig implements the native logic
- Zig exposes C ABI functions
- Elixir loads the compiled NIF

**Best for**: Safer high-performance NIFs, RL kernels, math-heavy code

---

### 🟩 Python ↔ Elixir (Indirect via C ABI)

Python cannot directly implement NIFs.

Instead:
- Python ↔ C via CPython C API
- Elixir ↔ C via NIF
- C acts as the bridge

This is usually a **bad idea** for performance and safety.

Problems:
- Two runtimes
- Two GCs
- Scheduler interference
- Complex lifetime management

**Better Alternatives**:
- Ports
- gRPC
- Shared memory via OS primitives

**Use only if**: Reusing an existing C library that already embeds Python

---

### 🟥 Python ↔ Elixir (Preferred Alternatives)

Instead of NIFs:
- Use **Ports** for isolation
- Use **ZeroMQ / gRPC**
- Use **shared files or memory**
- Use **Rustler + Rust** instead of Python

---

## 📊 Comparison Chart

| Mechanism | Overhead | Safety | Crash Isolation | Best Use Case |
|--------|----------|--------|-----------------|--------------|
| **NIF** | Lowest | Lowest | None | Hot paths |
| **Dirty NIF** | Low | Medium | None | CPU/IO heavy |
| **Port** | Medium | High | Yes | External processes |
| **CNode** | Medium | High | Yes | Distributed systems |
| **Python FFI** | Medium | Medium | Partial | Python-native libs |
| **gRPC** | High | Very High | Yes | Cross-language systems |

---

## 🛠️ Use Cases

- RL inference kernels
- Numeric computation
- Compression and encoding
- Cryptographic primitives
- Signal processing
- Hardware drivers
- Fast parsing and serialization

---

## ⚠️ Weaknesses & Risks

- VM-wide crash on fault
- Scheduler starvation
- Memory corruption
- ABI breakage across OTP upgrades
- Difficult debugging
- Hard to hot-reload safely

---

## 🛠️ Developer Tools

- `erl_nif.h`
- `rebar3` / `mix`
- `valgrind`
- `gdb` / `lldb`
- `Rustler` (Rust-based NIF framework)
- Zig build system
- Sanitizers (ASAN, UBSAN)

---

## 📑 Documentation and Support

- Erlang NIF Documentation
- Elixir NIF Guidelines
- OTP Design Principles
- Rustler Documentation
- Zig Language Reference

---

## 🔗 Related Concepts / Notes

- [[C FFI]]
- [[FFI]]
- [[ABI]]
- [[Elixir]]
- [[Erlang]]
- [[BEAM]]
- [[Rustler]]
- [[Ports]]
- [[CNode]]
- [[Zig Programming Language]]
- [[Python]]
- [[Zig]]
- [[C]]

---

## 📝 Summary

NIFs are the sharpest tool available for integrating native code into Elixir. They bypass IPC and deliver unmatched performance, but at the cost of safety and isolation. C and Zig are the most appropriate languages for writing NIFs, with Zig offering a compelling modern alternative due to its explicitness and C ABI compatibility. Python should generally be kept out of the NIF boundary and integrated via safer mechanisms.

For RL systems and performance-critical pipelines, NIFs are invaluable—but only when used sparingly, carefully, and with full understanding of BEAM’s scheduling and failure model.
