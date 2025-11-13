# callconv (Calling Convention) 🔄

A **calling convention** defines how functions receive parameters and return values at the machine level — specifying stack layout, register usage, argument passing, and cleanup responsibilities. It ensures consistent interaction between compiled code, libraries, and languages — critical for FFI (Foreign Function Interface) and ABI (Application Binary Interface) compatibility across C, Python (C-API, ctypes), Zig, and Elixir (via NIFs).

---

## ⚙️ Overview
Calling conventions define *how* and *where* function parameters are passed (registers or stack), *who* cleans up the stack (caller or callee), and *how* names are mangled. Consistency is essential for linking compiled code across languages and modules — mismatched conventions cause crashes or data corruption.  

---

## 📚 Summary
When integrating multiple languages (e.g., C ↔ Zig, Python ↔ C, Elixir ↔ C), matching the calling convention ensures binary compatibility. Each compiler and architecture can default to a different convention (`cdecl`, `stdcall`, `fastcall`, `sysv`, `win64`), but they all must agree when sharing a function boundary.

---

## 🧩 Core Concepts
- **ABI (Application Binary Interface)** — contract between caller and callee on binary level.  
- **Stack discipline** — defines order and cleanup of arguments.  
- **Register allocation** — determines which registers hold arguments and return values.  
- **Name mangling** — determines function symbol representation.  
- **FFI boundaries** — inter-language function calls depend on compatible callconv.  

---

## 🔍 Comparison Chart (Common Calling Conventions)

| Convention | Platform | Cleanup Responsibility | Argument Passing | Typical Use |
|-------------|-----------|------------------------|------------------|--------------|
| `cdecl` | x86 (C compilers) | Caller cleans stack | Right-to-left on stack | Default in C/C++ |
| `stdcall` | Windows API | Callee cleans stack | Stack | Windows system libraries |
| `fastcall` | Windows / MSVC | Caller | Registers (ECX, EDX) | MS performance opt |
| `sysv_abi` | Linux / Unix | Caller | Registers (RDI, RSI, etc.) | GCC/Clang default on x86_64 |
| `win64` | Windows x64 | Caller | RCX, RDX, R8, R9 | Windows x64 ABI |
| `thiscall` | C++ (x86) | Callee | `this` in ECX | Member functions |
| `vectorcall` | x86/x64 | Caller | Vector registers | SIMD optimization |

---

## 🧠 How It Works
When a function is called:
1. Arguments are pushed or moved to registers as defined by callconv.  
2. Control transfers via a `call` instruction.  
3. The callee executes and returns a value (register or memory).  
4. Either the caller or callee adjusts the stack.  
5. Return address restored; control resumes.  

Each architecture (ARM, x86, RISC-V) defines its own ABI, and compilers generate code accordingly.

---

## 🧑‍💻 One-Liners — By Language

### 🧰 C
- `__attribute__((cdecl))` / `__attribute__((stdcall))` / `__vectorcall` define conventions explicitly.  
- Default: `cdecl` (GCC/Clang/Linux), `__stdcall` (WinAPI).  
- Interfacing with Python C-API and system libraries requires matching the platform ABI.  

### 🧰 Python
- Python’s **C API** assumes the platform’s default C calling convention.  
- `ctypes.CFUNCTYPE(None, c_int, use_errno=True, use_last_error=True)` can specify callconv (`stdcall` or `cdecl`).  
- `cffi` and `cython` automatically use system defaults unless overridden.  
- Mismatch causes segmentation faults or corrupted arguments.  

### ⚡ Zig
- Declares conventions via `callconv(.C)`, `callconv(.Stdcall)`, etc.  
- Zig automatically adapts when targeting specific ABIs (`.C` ensures C-ABI compatibility).  
- Example: `fn foo(a: i32) callconv(.C) void {}` ensures linkage with C or Python FFI.  
- Zig’s LLVM backend supports platform ABIs, enabling safe interop with C, Python, and even Rust.  

### 🔥 Elixir (BEAM VM)
- Native code interfaces (NIFs) are **C shared libraries** loaded by the BEAM.  
- Must use the platform C ABI (`ERL_NIF_INIT` follows `cdecl` on x86).  
- Incorrect conventions crash the VM.  
- Calling C functions from NIFs or Rustler must match the system’s ABI.  

---

## ⚙️ Use Cases
- FFI (Foreign Function Interface) between C and higher-level languages.  
- Cross-language bindings (e.g., Zig or Rust to Python via shared libs).  
- System calls (kernel expects specific ABI).  
- Performance tuning (vectorcall, fastcall).  
- Embedded systems where assembly-level linkage matters.  

---

## ✅ Strengths
- Enables interoperability between languages and compilers.  
- Optimized variants improve performance (e.g., `fastcall`).  
- Enforces predictable binary-level function behavior.  

---

## ⚠️ Weaknesses
- ABI mismatches cause undefined behavior or crashes.  
- Platform-specific — Windows/Linux ABIs differ.  
- Not always portable across compilers or architectures.  

---

## 🔗 Related Concepts / Notes
- - [[ABI]] (Application Binary Interface)  
- - [[FFI]] (Foreign Function Interface)  
- - [[Python C API]]  
- - [[Zig FFI]]  
- - [[Elixir NIF]]  
- - [[LLVM]] (compiler backend managing callconv)  
- - [[C ABI]] (Platform conventions for C interop)  

---

## 🧰 Developer Tools
- **objdump / nm / readelf** — inspect symbol names and attributes.  
- **clang -cc1 -fdump-record-layouts** — view ABI info.  
- **gdb** — debug stack/register state.  
- **Zig `@call()` builtin** — allows dynamic convention calls.  
- **Python `ctypes`** — manual ABI definition.  

---

## 📖 Documentation & Support
- System V ABI spec (x86_64, ARM, RISC-V).  
- Microsoft x64 Calling Convention (MSDN).  
- Zig language reference (`callconv`).  
- Python C API and `ctypes` docs.  
- Elixir/BEAM NIF guide.  

---

## 📚 Further Reading
- *System V ABI AMD64 Specification* — definitive Linux calling convention reference.  
- *Microsoft x64 Calling Convention* — official doc for Windows ABI.  
- *Zig Language Reference* — FFI and callconv examples.  
- *Elixir NIFs* — BEAM VM native interface rules.  

---

## 🧭 Key Highlights
1. callconv = rulebook for how functions call each other at binary level.  
2. Crucial for inter-language integration and FFI stability.  
3. C sets the baseline ABI for most ecosystems (Python, Zig, Elixir).  
4. Always match conventions across modules to avoid crashes.  
