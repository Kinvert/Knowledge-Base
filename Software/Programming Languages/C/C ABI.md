# C ABI (Application Binary Interface)

The **C Application Binary Interface (ABI)** defines how binary code modules — such as compiled functions and libraries — interact at the machine level. It governs low-level details like data representation, function calling conventions, register usage, and name mangling.  
In short, the **ABI ensures compiled code from different compilers or languages can work together**, making it fundamental to [[C FFI]] and interoperability across systems.

---

## ⚙️ Overview

An ABI is the *contract* between the compiler, linker, and runtime environment.  
Whereas the **API (Application Programming Interface)** specifies *what* functions and data structures are available, the **ABI** defines *how* they are represented and called at the binary level.

In robotics and embedded systems, maintaining ABI compatibility is critical — firmware, drivers, or shared libraries compiled with different compilers or versions can fail catastrophically if their ABIs differ.

---

## 🧠 Core Concepts

- **Calling Conventions:** Rules for how parameters and return values are passed between functions (e.g., via stack or registers).
- **Data Layout:** Defines how structs, enums, and primitive types are aligned and padded in memory.
- **Name Mangling:** Symbol naming conventions in compiled binaries; C uses unmangled names, while C++ requires `extern "C"` to maintain compatibility.
- **Register Usage:** Which CPU registers are preserved or clobbered during function calls.
- **Stack Frame Organization:** How local variables, return addresses, and saved registers are stored.
- **Exception Handling and Stack Unwinding:** Defines how control flow behaves under failure or during cleanup.
- **System V ABI vs Microsoft ABI:** The two dominant families for UNIX-like and Windows systems respectively.

---

## 🧩 Comparison Chart

| ABI Type | Platform / OS | Calling Convention | Common Use Case | Notable Features |
|-----------|----------------|--------------------|------------------|------------------|
| **System V AMD64 ABI** | Linux, macOS, BSD | Registers for first 6 args | Standard for Unix-like x86_64 | Used by GCC & Clang |
| **Microsoft x64 ABI** | Windows | Registers for first 4 args | Win32/Win64 systems | Uses different stack alignment |
| **ARM EABI** | ARM / Embedded | Varies by compiler | Embedded systems, mobile devices | Tailored for efficiency & determinism |
| **AAPCS** | ARM Procedure Call Standard | Register-based | Robotics controllers, Cortex MCUs | Common in real-time robotics code |
| **Itanium C++ ABI** | Cross-platform | Name mangling & vtables | C++ interoperability | Defines RTTI and exception model |

---

## 🧩 Language ABI Compliance Comparison
|Language |	Default ABI Compliance | Interop Mechanism |	Name Mangling |	Notable Traits|
|--|-------------|---------|--------|------------------------|
| C |	Native ABI |	Direct |	None	| Canonical reference ABI|
| C++ |	Partial (via `extern "C"`) |	Linker/export directives |	Yes |	Requires explicit C linkage|
| Rust |	Partial (`extern "C"`) |	FFI layer |	Optional |	Enforces explicit ABI safety|
| Go |	Custom Go ABI	| cgo bridge |	No |	Uses runtime mediation|
| Zig |	Full C ABI compliance |	`@cImport, extern, export` |	None (C-compatible) |	Direct binary compatibility and compile-time header parsing|

---

## 🦎 Zig and the C ABI
Zig fully complies with the C ABI on all supported targets, allowing seamless interoperability with existing C libraries and system interfaces. Its compiler emits machine code adhering to the same conventions as Clang and GCC for the target platform, including System V AMD64 ABI on UNIX-like systems and Microsoft x64 ABI on Windows.
Zig’s tooling natively understands and preserves ABI alignment, calling conventions, and symbol names without additional wrappers or headers. The @cImport builtin automatically parses C headers using Clang, ensuring ABI-accurate type definitions and linkage.

When exporting functions or linking with C code, Zig’s export and extern keywords map directly to C ABI expectations — no name mangling, hidden struct padding, or altered calling convention occurs unless explicitly changed by the developer.
Because Zig avoids runtime dependencies and hidden metadata, its binaries maintain strict ABI fidelity — ideal for systems programming, kernel modules, or shared library development that must match C-level expectations.

When a function is compiled, the compiler emits machine code conforming to a specific ABI.  
If another module — written in C, [[Rust]], or [[Python]] — expects the same ABI, their compiled forms can directly interact at runtime.

Example flow:
1. Source code is compiled into object files following an ABI.
2. The linker merges symbols and addresses using the same ABI conventions.
3. The runtime loader resolves and executes these binary interfaces seamlessly.

In [[C FFI]] contexts, both sides (host language and C code) must agree on the ABI to ensure parameters and memory layouts align correctly.

---

## 🧰 Use Cases

- Creating binary-compatible shared libraries (`.so`, `.dll`, `.a`).
- Interfacing with OS-level APIs or firmware drivers.
- Ensuring ABI stability in SDKs distributed to third-party developers.
- Building cross-language integrations (e.g., Python → C → Rust).
- Debugging low-level assembly or reverse engineering.

---

## ✅ Strengths

- Enables cross-language and cross-compiler interoperability.
- Defines portable, stable binary contracts.
- Vital for driver, firmware, and library compatibility.
- Facilitates efficient system-level development.

---

## ❌ Weaknesses

- Platform and compiler specific (portability issues).
- ABI mismatches can cause silent crashes or memory corruption.
- Changing compiler flags (e.g., struct alignment) can break ABI compatibility.
- No unified ABI across all architectures or operating systems.

---

## 🧩 Related Concepts / Notes

- [[C FFI]] (Foreign Function Interface)
- [[Calling Convention]] (Function parameter passing rules)
- [[Linker]] (Binary symbol resolution)
- [[ELF]] (Executable and Linkable Format)
- [[System V ABI]] (UNIX x86_64 standard)
- [[EABI]] (Embedded ABI for ARM)
- [[C++ ABI]] (Name mangling and class layout rules)
- [[Memory Alignment]] (Data placement rules)
- [[Assembly]] (Machine-level code representation)
- [[Compiler]] (Generates code conforming to ABI)

---

## 🧮 Compatible Items

- Compilers: GCC, Clang, MSVC, ARMCC
- Tools: `nm`, `objdump`, `readelf`, `ldd`
- Build systems: CMake, Make, Bazel
- Languages: [[C]], [[C++]], [[Rust]], [[Zig]], [[Go]]
- Architectures: x86, x86_64, ARM, RISC-V, MIPS

---

## 🔧 Developer Tools

- `readelf -h binary` to inspect architecture and ABI.
- `nm -C library.so` to check symbol names.
- `objdump -d binary` for disassembly and ABI verification.
- `clang -target` flag to specify ABI during cross-compilation.
- `bindgen` for generating ABI-compatible bindings in Rust.

---

## 📚 External Resources

- [System V AMD64 ABI Specification (PDF)](https://uclibc.org/docs/psABI-x86_64.pdf)
- [ARM ABI Documentation](https://developer.arm.com/documentation/ihi0042/latest)
- [Microsoft x64 Calling Convention Docs](https://learn.microsoft.com/en-us/cpp/build/x64-software-conventions)
- [Itanium C++ ABI Specification](https://itanium-cxx-abi.github.io/cxx-abi/)
- [GCC ABI Notes](https://gcc.gnu.org/onlinedocs/gcc/Interface.html)

---

## 🧭 Summary

The **C ABI** forms the invisible backbone of binary interoperability across programming languages, compilers, and hardware platforms.  
It ensures that independently compiled code modules can communicate — a critical property in systems where performance, real-time reliability, and modularity matter most, such as in robotics, embedded development, and OS-level programming.

---
