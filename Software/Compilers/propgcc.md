# 🟡 propgcc

**propgcc** is Parallax's port of [[GCC]] to the Parallax Propeller microcontroller. It provides a C/C++ toolchain, binutils, GDB-related pieces, Propeller libraries, loaders, demos, and related tools for writing compiled code for [[Parallax Propeller 1]] and related workflows.

---

## 📚 Overview

The Propeller ecosystem historically centered on [[Spin]] and PASM assembly. `propgcc` makes the chip accessible from C and C++ by providing a `propeller-elf` cross-compiler toolchain. It is especially relevant when mixing low-level embedded work, Propeller boards, serial loaders, and Linux or WSL-based development environments.

The upstream repository describes it as a GCC port for the Parallax Propeller and recommends building it with the project Makefile. Output defaults to `/opt/parallax`, with cross-build output variants such as `/opt/parallax-win32` and `/opt/parallax-rpi`.

---

## 🧠 Core Concepts

- **Cross Compiler**: Runs on a host PC but emits Propeller-targeted binaries.
- **`propeller-elf-gcc`**: GCC executable targeting the Propeller ELF environment.
- **binutils**: Assembler, linker, object tools, and related binary utilities.
- **Propeller Libraries**: Runtime and board-support libraries for Propeller programs.
- **`propeller-load`**: Serial loader and communications program for loading Propeller binaries.
- **[[propeller-elf-gcc]]**: GCC compiler driver that builds Propeller ELF binaries.
- **COG**: Propeller hardware execution core.
- **XMM**: External memory model support used by some Propeller programs.
- **GDB Support**: Debugger-related tooling and stubs in the repository.

---

## ⚙️ Build Workflow

Basic source build:

```bash
git clone https://github.com/parallaxinc/propgcc.git
cd propgcc
make
```

Default install/output location:

```bash
/opt/parallax
```

Add the toolchain to `PATH`:

```bash
export PATH=/opt/parallax/bin:$PATH
```

Cross-build examples:

```bash
make CROSS=win32
make CROSS=rpi
```

Change install prefix:

```bash
make PREFIX=/path/to/prefix
```

---

## 📊 Comparison Chart

| Tool / Language | Target | Strength | Weakness | Best Use |
|---|---|---|---|---|
| **propgcc** | Propeller C/C++ | GCC-style compiled workflow | Older toolchain base | C/C++ on Propeller |
| [[Spin]] | Propeller | Native Parallax language | Less portable than C | Classic Propeller code |
| PASM | Propeller assembly | Maximum hardware control | More manual work | Timing-critical COG code |
| SimpleIDE | Propeller IDE | Easier beginner workflow | Less transparent than CLI | Education and demos |
| FlexProp | Propeller 1/2 | Modern multi-language toolchain | Separate ecosystem | Modern Propeller dev |
| Generic [[GCC]] | Many targets | Mature compiler ecosystem | Needs target port | General embedded C |

---

## ✅ Pros

- Brings GCC-style C/C++ development to Propeller.
- Includes binutils, libraries, loader tools, demos, and debugger-related pieces.
- Useful for developers already comfortable with [[C]], Makefiles, and cross-compilers.
- Supports Linux-hosted builds and cross-compilation workflows.
- Makes Propeller development fit better into Unix-style tooling.

---

## ❌ Cons

- Based on older GCC/binutils/GDB components.
- Build process can be heavy compared with using a packaged IDE.
- Propeller architecture is unusual, so normal embedded assumptions may not apply.
- Documentation is spread across repo files and generated docs.
- Toolchain setup can be fiddly on modern systems.

---

## 🧰 Repository Layout

| Directory | Purpose |
|---|---|
| `binutils` | Propeller-modified GNU binary utilities |
| `gcc` | Propeller GCC port |
| `gdb` | Propeller-modified GDB pieces |
| `gdbstub` | Interface between GDB and the Propeller |
| `lib` | Propeller libraries |
| `loader` | `propeller-load` serial loader and communications tool |
| `demos` | Example projects |
| `spin2cpp` | Spin-to-C/C++ conversion tool |
| `spinsim` | Propeller simulator |
| `p2load` | Loader-related tooling for Propeller 2 workflows |

---

## 🔧 Practical Notes

- Start with the demos after building the toolchain.
- Add `/opt/parallax/bin` to `PATH` before invoking compiler or loader tools.
- USB serial access may require Linux permissions, [[udevadm]] rules, or [[usbipd]] when using WSL.
- Use `propeller-load` for loading binaries onto boards over serial.
- Keep [[C ABI]], [[C Memory Layout]], and embedded memory limits in mind.

---

## 🔗 Related Notes

- [[Parallax Propeller 1]]
- [[Parallax Propeller 2]]
- [[Spin]]
- [[C]]
- [[GCC]]
- [[propeller-elf-gcc]]
- [[GDB]]
- [[C ABI]]
- [[usbipd]]
- [[UART]]

---

## 🌐 External Resources

- propgcc GitHub: https://github.com/parallaxinc/propgcc
- Parallax: https://www.parallax.com/

---

## 📝 Summary

`propgcc` is the GCC-based path into C/C++ development for the Parallax Propeller ecosystem. It is most useful when you want a Unix-style embedded toolchain instead of a purely Spin-oriented workflow.
