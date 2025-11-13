# Dynamic Linking 🔗

Dynamic Linking refers to the process of resolving and loading external code modules (shared libraries) at runtime rather than at compile-time or static link-time. It is a cornerstone of modern Unix-like operating systems, allowing multiple executables to share common libraries, saving memory, and enabling modular updates. In Linux, this process is built around the [[ELF]] (Executable and Linkable Format) and handled by the system’s dynamic linker (`ld-linux.so`).

---
## ⚙️ Overview
Dynamic linking allows executables to defer the inclusion of external code until runtime. Instead of embedding all required functions directly into the executable, references to shared objects (`.so` files) are stored in the ELF dynamic section (`.dynamic`). When the program launches, the dynamic linker locates and loads these shared libraries into memory and resolves their symbol references.

This design allows:
- Smaller binaries
- Shared memory for common libraries (like `libc`)
- Hot-swappable library updates without recompiling applications

---
## 🧠 Core Concepts
- **Shared Object (`.so`)** — Compiled library that can be loaded by multiple programs at once.
- **PLT (Procedure Linkage Table)** — Used by the executable to call functions whose addresses are unknown until runtime.
- **GOT (Global Offset Table)** — Stores resolved addresses of external functions/data.
- **`dlopen`, `dlsym`, `dlclose`** — Runtime API (from `libdl`) for dynamic loading and symbol lookup.
- **RPATH / RUNPATH** — ELF dynamic tags defining search paths for shared libraries.
- **SONAME** — Symbolic name of a shared object, allowing versioned compatibility.
- **Relocation** — Process of patching addresses in the executable once shared objects are loaded.
- **Lazy Binding** — Defers symbol resolution until first use for faster startup.
- **`LD_PRELOAD`** — Environment variable that forces the loader to load custom libraries before others (useful for debugging, hooking, or overriding symbols).

---
## 🧩 How It Works (Step-by-Step)
1. The **ELF executable** includes a `.dynamic` section listing `DT_NEEDED` dependencies.
2. When executed, the **kernel** maps the ELF file and invokes the **dynamic linker** (`ld-linux.so`).
3. The linker searches for required libraries (in paths from `DT_RPATH`, `LD_LIBRARY_PATH`, `/etc/ld.so.conf`, etc.).
4. Each `.so` is loaded into process memory and relocated.
5. The linker updates the GOT and PLT so all function and data references point to the correct memory addresses.
6. Control transfers to the program’s entry point (usually `_start`).

---
## 🧾 Comparison Chart — Dynamic vs Static Linking

| Feature | Dynamic Linking | Static Linking |
|---|---|---|
| Linking time | Runtime | Compile-time |
| Output size | Smaller | Larger |
| Memory usage | Shared between processes | Duplicated per process |
| Update mechanism | Replace `.so` file | Rebuild/relink binary |
| Performance | Slight overhead (PLT/GOT, indirection) | Slightly faster (direct calls) |
| Portability | Requires same library versions | Fully self-contained |
| Example extension | `.so` | `.a` |

---
## 🧮 Comparison Chart — Language Handling

| Language | Uses Dynamic Linking By Default? | Mechanism | Common Dynamic Libraries |
|---|---|---|---|
| C / C++ | Often | ELF `DT_NEEDED`, `dlopen`, `libdl` | `libc.so`, `libm.so` |
| Zig | Yes (configurable) | Direct control over linker and `-dynamic` build | `libc.so`, custom `.so` |
| Python | Yes (via `dlopen`) | Loads native extensions dynamically | `_ctypes.so`, extension modules |
| Elixir (BEAM) | Yes | NIFs (`.so` shared libraries) loaded by VM | Hardware drivers, C/Rust interop |

---
## 🔍 Use Cases
- Loading performance-critical C modules in Python (`import` triggers `dlopen`).
- Robotics: dynamically load sensor drivers or communication backends without rebuilding the core runtime.
- Plugin systems for simulation or control pipelines.
- Hot-swapping shared libraries for on-the-fly firmware or software updates.
- Dynamic extension loading in BEAM/Elixir using NIFs or Port Drivers.

---
## 💪 Strengths
- Reduces duplication in memory and disk usage.
- Allows independent updating of libraries.
- Enables modularity and extensibility.
- Supports plugin architectures (e.g., Gazebo plugins, Python C extensions).
- Provides fine-grained runtime control (`dlopen`/`dlsym`).

---
## ⚠️ Weaknesses
- ABI incompatibilities can cause runtime crashes or symbol mismatches.
- Harder debugging (symbols resolved at runtime).
- Library version conflicts (“dependency hell”).
- Small runtime overhead (GOT/PLT lookups).
- Security risks from `LD_PRELOAD` misuse.

---
## 🔧 Developer Tools
- `ldd` — shows dynamic dependencies.
- `readelf -d` — displays `.dynamic` entries.
- `nm -D` — lists dynamic symbols.
- `objdump -p` — prints program headers including dynamic sections.
- `strace` — traces calls to `open`, `mmap`, `dlopen`.
- `ldconfig` — manages `/etc/ld.so.cache` for fast library lookup.
- Libraries: `libdl` (C), `ctypes` (Python), `:erlang.load_nif` (Elixir).

---
## 🧩 Compatible Items
- ELF executables and shared objects.
- POSIX-compliant systems (Linux, BSD, Solaris).
- Linkers: `ld`, `lld`, `gold`.
- Debuggers: `gdb`, `lldb`.
- Build systems: `cmake`, `make`, `zig build`, `mix compile` (via NIF linkage).

---
## 🔗 Related Concepts / Notes
- - [[ELF]] (Executable and Linkable Format)
- - [[Linkers]] (Static and dynamic symbol resolution)
- - [[Shared Libraries]] (`.so`, `.dll`, `.dylib`)
- - [[PIC and PIE]] (Position Independence)
- - [[Symbol Versioning]] (ABI management)
- - [[RPATH vs RUNPATH]] (Library search behavior)
- - [[LD_PRELOAD]] (Preloading libraries)
- - [[NIFs]] (Elixir native interface)
- - [[pyelftools]] (Inspect ELF `.so` structure)
- - [[Zig]] (Can produce or statically link `.so` files)
- - [[ABI]] (Application Binary Interface)

---
## 📚 External Resources
- `man ld.so`, `man dlopen`
- ELF Dynamic Linking specification (System V ABI)
- Ulrich Drepper’s *How To Write Shared Libraries*
- `ldd`, `readelf`, `objdump` documentation
- `pyelftools` and `LIEF` projects for ELF parsing
- Elixir documentation: `:erlang.load_nif/2`

---
## 🧩 Summary
Dynamic linking is the mechanism that enables modular, efficient, and maintainable software ecosystems on Unix-like systems. It underpins how Python loads C extensions, how Elixir integrates with NIFs, and how C, C++, and Zig share core libraries like `libc`. Understanding dynamic linking is critical for systems engineers, robotics developers, and anyone writing or distributing native code in multi-language environments.

---
## 💡 Quick Commands
- View dependencies: `ldd ./program`
- Inspect dynamic section: `readelf -d ./program`
- Load a library manually in C: `dlopen("libfoo.so", RTLD_NOW)`
- Check RPATHs: `readelf -d ./program | grep PATH`
- Preload custom libs: `LD_PRELOAD=./override.so ./program`
