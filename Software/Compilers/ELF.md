# ELF — Executable and Linkable Format 📦

ELF (Executable and Linkable Format) is the standard binary file format for Unix-like systems (Linux, many BSDs, Solaris). It describes how machine code, data, symbol tables, relocation info, dynamic linking metadata, and debug information are laid out so linkers, loaders, debuggers, and other tools can work with them. ELF is used for: executables, relocatable object files (`.o`), shared libraries (`.so`), core dumps, and more.

---
## 🔍 Overview
ELF is a container format (not a language-specific object model). It cleanly separates two views:
- **Sections** (link-time view) — `.text`, `.data`, `.bss`, `.rodata`, `.symtab`, `.rela.*`, `.debug_*` etc. Useful for linkers, debuggers, and static analysis.
- **Segments** (runtime/program view) — contiguous memory mapping for the loader; defined by Program Headers (PT_LOAD, PT_DYNAMIC, PT_TLS, …).

Key on-disk pieces:
- **ELF Header**: magic, class (32/64), endianness, entry point.
- **Program Header Table**: tells loader what to map into memory.
- **Section Header Table**: gives per-section metadata; optional for stripped binaries.
- **Symbol Tables** (`.symtab`, `.dynsym`), Relocations (`.rel`/`.rela`), Dynamic Section (`.dynamic`), GOT/PLT for dynamic linking.
- **Note**: A `.so` file on Linux *is* an ELF shared object; `.o` object files are ELF relocatable objects.

---
## 🧠 Core Concepts
- **Relocatable vs. Executable vs. Shared**:
  - *Relocatable object (`ET_REL`)* — produced by compilers (`.o`), contains relocations and section info.
  - *Executable (`ET_EXEC`) / PIE (`ET_DYN` with executable flags)* — program with entry point.
  - *Shared object (`ET_DYN`)* — position-independent and loadable into other processes (`.so`).
- **Position Independent Code (PIC/PIE)** — required for shared libs and for full ASLR-friendly executables.
- **GOT/PLT** — runtime mechanism to lazily resolve symbols from shared libraries.
- **Dynamic Linker (ld-linux.so)** — reads `DT_*` entries (e.g. `DT_NEEDED`, `DT_RPATH`, `DT_RUNPATH`) to locate dependencies.
- **Symbol resolution & versioning** — `SONAME`, versioned symbols (`GLIBC_2.2.5` style), symbol visibility.
- **Relocation types** — architecture-specific (x86_64, aarch64, RISC-V each have different relocation semantics).
- **Thread-Local Storage (TLS)** — ELF supports TLS segments (PT_TLS) and platform-specific TLS relocations.
- **Debug info** — DWARF lives in `.debug_*` sections; can be split to separate files.

---
## ⚙️ How It Works (high level)
1. Compiler emits object (`.o`) ELF (relocatable).
2. Linker (`ld`, `gold`, `lld`) combines `.o` files and libraries into an executable or `.so`.
3. Runtime loader (kernel + dynamic linker) maps ELF segments and resolves dynamic symbols.
4. When a program loads a native extension (e.g., Python `.so`), the process calls `dlopen()` which parses ELF dynamic section and links symbols.

---
## 🛠️ Developer Tools (what reads/writes ELF)
- **Compilers/Linkers**: `gcc`, `clang`, `zig`, `ld`, `lld`, `gold`
- **Binutils**: `objdump`, `nm`, `readelf`, `objcopy`, `ldd`, `strip`
- **Debuggers**: `gdb`, `lldb`
- **Runtime**: kernel ELF loader, `ld-linux.so` (dynamic linker)
- **Libraries/parsers**:
  - Python: `pyelftools` (read/write parsing), `lief` (binary editing)
  - C: `libelf`, `elf.h` APIs
  - Zig: standard library exposes ELF-writing/reading helpers and can produce ELF outputs
  - Elixir/Erlang NIFs: loaded via `:erlang.load_nif/2` which ultimately uses `dlopen()`
- **Analyzers/static**: `file`, `readelf -h`, `readelf -l`, `readelf -S`, `objdump -d`, `eu-readelf` (elfutils)

---
## 🧾 Comparison Chart — Languages & How They Produce / Use ELF

| Language | Primary Build Artifact | How ELF enters the picture | Typical ELF artifacts produced | Native interop pattern |
|---|---:|---|---|---|
| C / C++ | `gcc/clang` → `.o` → linked `.so`/executable | Native: compiler emits ELF object/ELF executable | `.o`, `a` (archive of `.o`s), `.so`, executables | Direct: link with `-l`, `dlopen`, symbol exports |
| Zig | `zig build` | Zig's compiler emits ELF for supported targets; explicit control over linker | `.o`, `.so`, executables; Zig can call linker or produce static/dynamic libs | Direct (C ABI compatible), can produce `-fPIC`/`SONAME` |
| Python | `.py` → `.pyc` (VM) ; extensions via C | Python itself is an ELF executable on Linux; C extensions are ELF shared objects | `.so` extension modules (ELF shared objects) | `import` of extension modules triggers `dlopen()` |
| Elixir (BEAM) | `.ex` → `.beam` (BEAM bytecode) | BEAM VM is typically an ELF binary; native NIFs are ELF `.so` files loaded by VM | `.beam` (VM bytecode), `.so` (NIF libs) | NIFs loaded via `:erlang.load_nif` → `dlopen()` of ELF `.so` |

---
## ⚖️ Comparison Chart — ELF vs Other Binary Formats

| Feature | ELF (Linux/BSD) | PE (Windows) | Mach-O (macOS) |
|---|---:|---:|---:|
| Platform | Unix-like | Windows | macOS / iOS |
| Dynamic loader | `ld-linux.so` | Windows loader | dyld |
| Shared lib extension | `.so` | `.dll` | `.dylib` |
| Sections vs Segments | Sections + Program Headers | Sections + Data directories | Segments + Sections |
| Widely used in embedded Linux | Yes | No | No |
| Tooling | `readelf`, `objdump`, `ldd` | `dumpbin`, `depends` | `otool`, `dyldinfo` |

---
## ✅ Use Cases (where ELF matters)
- Building native robotics control loops in C/C++/Zig producing real-time capable executables.
- Python extension modules for performance-critical sensor processing (`.so` modules).
- Creating NIFs in C/Rust for Elixir to do low-level hardware interfacing.
- Static analysis, reverse engineering, and firmware inspection on embedded Linux targets.
- Packaging drivers / kernel modules (special ELF variants for kernel modules).

---
## 💪 Strengths
- Platform standard on Unix-like systems — great tooling ecosystem.
- Flexible: supports many object types, debug info, symbol/versioning and TLS.
- Extensible for architectures: relocation and ABI details per-arch.
- Well-understood by linkers, debuggers; mature binary editing libraries exist (`lief`, `pyelftools`).

---
## ⚠️ Weaknesses
- Complexity: many fields, architecture-specific relocation types.
- ABI/Loader behavior can vary across distros (dynamic linker versions, glibc symbol versioning).
- Stripped binaries can be hard to reverse engineer; debug info separate.
- Not portable across OS families (need PE/Mach-O conversion for Windows/macOS).

---
## 🧩 Variants & Related Concepts
- **Position-Independent Executable (PIE)** — ELF executable built with `-fPIE -pie` for ASLR.
- **Static linking** — linking libraries into executable (`-static`) removing runtime dependencies.
- **Shared objects** — `.so` files with `SONAME` and `DT_NEEDED` entries.
- **Kernel modules** — ELF-like but special linking/loading into kernel space.
- **Static/dynamic split debug** — `objcopy --only-keep-debug` / `eu-strip` workflows.

---
## 🔧 Compatible Items (tools, libs, runtimes)
- `gcc`, `clang`, `ld`, `lld`, `zig`
- `readelf`, `objdump`, `nm`, `ldd`, `strip`, `objcopy`
- `pyelftools`, `lief`, `elfutils`, `libelf`
- Python runtime (`python3` ELF executable), BEAM VM (Erlang runtime) as ELF processes
- Cross-compilers for cross-target ELF (e.g., `aarch64-linux-gnu-gcc`)

---
## 🧾 Key Features (quick list)
- Multi-architecture support (x86_64, aarch64, RISC-V, MIPS, etc.)
- Dynamic linking metadata (`DT_NEEDED`, `DT_SONAME`)
- Symbol tables, relocations, versioning
- Separate debug sections (DWARF)
- TLS support, PLT/GOT lazy binding

---
## 🔬 Strengths/Weaknesses (pros/cons quick)
- **Pros**: standard, tool-rich, flexible, cross-arch
- **Cons**: complex, distro-dependent loader behavior, not cross-OS

---
## 🔗 Related Concepts / Notes
- - [[Linkers]] (how `ld`/`lld` resolves symbols)
- - [[Shared Libraries]] (`.so`, SONAME, `DT_NEEDED`)
- - [[Dynamic Linking]] (GOT, PLT, lazy binding)
- - [[DWARF]] (debug format commonly stored in ELF)
- - [[Cross Compilation]] (producing ELF for embedded targets)
- - [[NIFs]] (Native Implemented Functions for BEAM/Elixir)
- - [[PIC and PIE]] (position independence)
- - [[RPATH vs RUNPATH]] (loader search path behavior)
- - [[Symbol Versioning]] (GLIBC style symbol versions)
- - [[pyelftools]] (Python library to inspect/edit ELF)
- - [[LIEF]] (binary editing library)

---
## 🧾 External Resources / Further Reading
- The ELF specification (System V ABI, “Executable and Linking Format”) — read official spec for details
- `man elf`, `man ld.so`, `readelf` and `objdump` man pages
- `pyelftools` docs for programmatic reading of ELF in Python
- LIEF project documentation (binary modification)
- "Linkers and Loaders" book (John R. Levine) — classic reference
- `ELF and Program Loading` articles (kernel/kernel docs)

---
## 🧪 Developer Examples & Patterns (short notes)
- To inspect headers: `readelf -h mybinary`
- To see dynamic deps: `readelf -d mybinary` or `ldd mybinary`
- To dump symbols: `nm -C mybinary` or `objdump -t`
- To strip debug: `strip --strip-debug mybinary` (or use split debug with `objcopy`)
- To build PIC: compile with `-fPIC`, link with `-shared` to produce `.so`
- To create a SONAME: `gcc -shared -Wl,-soname,libfoo.so.1 -o libfoo.so.1.0 ...`

---
## 🔭 How Languages Interact With ELF (summary)
- **C/C++**: Native first-class—compiler emits ELF objects and final ELF artifacts.
- **Zig**: Compiles/links to native ELF; aims for fine control over produced ELF artifacts and good C-ABI compatibility.
- **Python**: Interpreter often an ELF binary; performance extensions are ELF `.so` modules loaded with `dlopen`.
- **Elixir / Erlang**: Runs on BEAM VM (an ELF executable on Linux). Native code integration uses NIFs as ELF `.so` modules (or C nodes via sockets).

---
## 🏁 Summary
ELF is the lingua franca of native binaries on Unix-like systems. For robotics engineering it’s central: firmware, native libraries for sensor/actuator drivers, Python extension modules for numerics, Zig/C performance-critical pieces, and even runtime (BEAM) are all unioned by ELF. Mastering ELF concepts (sections vs segments, dynamic section, relocations, symbol versioning) unlocks cross-language integration, debugging, and safe packaging of native components.

---
## 📚 External Resources (short)
- `readelf` / `objdump` documentation
- `pyelftools` (Python)
- LIEF (binary manipulation)
- System V ABI — ELF Spec
