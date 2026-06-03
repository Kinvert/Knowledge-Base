# 🟡 propeller-elf-gcc

**`propeller-elf-gcc`** is the GCC cross-compiler executable installed by [[propgcc]]. It compiles C and C++ code on a host machine into ELF binaries for the Parallax Propeller toolchain.

---

## 📚 Overview

`propeller-elf-gcc` is the compiler driver in the PropGCC toolchain. Like other GCC cross-compilers, it calls the preprocessor, compiler, assembler, and linker with a target-specific prefix. The `propeller-elf` prefix means the toolchain emits binaries for the Propeller ELF environment rather than for the host operating system.

Typical workflow:

1. Write C or C++ source.
2. Compile with `propeller-elf-gcc`.
3. Load the resulting ELF with `propeller-load`.
4. Monitor serial output or debug with the available PropGCC tools.

---

## 🧠 Core Concepts

- **Cross Compiler**: Runs on Linux, Windows, or another host but emits Propeller-targeted code.
- **GCC Driver**: Front-end command that coordinates preprocessing, compiling, assembling, and linking.
- **ELF Output**: Executable and linkable format used by the PropGCC toolchain.
- **Memory Model**: Propeller-specific code placement/execution model such as COG, LMM, CMM, or XMM variants.
- **`propeller-load`**: Tool used to load the generated ELF onto a Propeller board.
- **Target Prefix**: Related tools use names like `propeller-elf-as`, `propeller-elf-ld`, `propeller-elf-objdump`, and `propeller-elf-g++`.

---

## ⚙️ Basic Commands

Check the compiler:

```bash
propeller-elf-gcc --version
propeller-elf-gcc --target-help
```

Compile a simple C file:

```bash
propeller-elf-gcc -Os -o hello.elf hello.c
```

Compile with common PropGCC memory model flags:

```bash
propeller-elf-gcc -Os -mlmm -o hello.elf hello.c
propeller-elf-gcc -Os -mcmm -o hello.elf hello.c
propeller-elf-gcc -Os -mcog -o hello.elf hello.c
```

Load the program:

```bash
propeller-load -r hello.elf
```

Inspect output:

```bash
propeller-elf-objdump -d hello.elf
propeller-elf-size hello.elf
```

---

## 📊 Comparison Chart

| Tool | Role | Target | Strength | Weakness |
|---|---|---|---|---|
| **`propeller-elf-gcc`** | C compiler driver | Propeller ELF | Familiar GCC workflow | Propeller-specific flags |
| `propeller-elf-g++` | C++ compiler driver | Propeller ELF | C++ support | Embedded C++ limits |
| `propeller-elf-as` | Assembler | Propeller ELF | Low-level assembly | More manual workflow |
| `propeller-elf-ld` | Linker | Propeller ELF | Linker-script control | Easier to misuse directly |
| `propeller-load` | Loader | Propeller boards | Loads ELF over serial | Needs board/port setup |
| [[Spin]] compiler | Spin language | Propeller | Native Propeller style | Not standard C |

---

## 🧩 Memory Model Notes

Propeller GCC programs must fit the Propeller's unusual memory architecture. Common model flags seen in PropGCC workflows include:

| Flag | Meaning | Typical Use |
|---|---|---|
| `-mcog` | COG memory model | Very small, fast code that fits in COG memory |
| `-mlmm` | Large Memory Model | Code executes from hub memory through an interpreter loop |
| `-mcmm` | Compact Memory Model | Smaller code size than LMM in many cases |
| `-mxmm-*` | External memory variants | Boards with external memory support |

Use `propeller-elf-gcc --target-help` on the installed toolchain as the source of truth for exact flags and supported variants.

---

## ✅ Pros

- Brings normal GCC-style C compilation to the Propeller.
- Works with standard GCC concepts such as optimization levels, include paths, and object files.
- Integrates with PropGCC libraries and loader tools.
- Easier to automate with Makefiles than GUI-only workflows.
- Useful when mixing C, assembly, and low-level embedded patterns.

---

## ❌ Cons

- Toolchain is older and target-specific.
- Propeller memory models are unusual compared with ARM or AVR.
- Code size and runtime behavior depend heavily on selected memory model.
- Loader and serial-port setup are separate concerns.
- Some modern GCC options or libraries may not apply.

---

## 🔧 Practical Flags

| Flag | Purpose |
|---|---|
| `-Os` | Optimize for size, often important on microcontrollers |
| `-O2` | General optimization |
| `-g` | Include debug information |
| `-I <dir>` | Add include directory |
| `-L <dir>` | Add library search directory |
| `-l<name>` | Link a library |
| `-std=c99` | Compile as C99 |
| `-Wall -Wextra` | Enable useful warnings |
| `-S` | Emit assembly |
| `-c` | Compile to object file without linking |

---

## 🧰 Makefile Pattern

```make
CC := propeller-elf-gcc
LOAD := propeller-load

CFLAGS := -Os -mlmm -Wall -Wextra -std=c99
TARGET := main.elf
SRC := main.c

$(TARGET): $(SRC)
	$(CC) $(CFLAGS) -o $@ $^

run: $(TARGET)
	$(LOAD) -r $(TARGET)

clean:
	rm -f $(TARGET) *.o
```

---

## 🔗 Related Notes

- [[propgcc]]
- [[GCC]]
- [[C]]
- [[C99]]
- [[ELF]]
- [[C ABI]]
- [[Parallax Propeller 1]]
- [[Spin]]
- [[UART]]

---

## 🌐 External Resources

- propgcc GitHub: https://github.com/parallaxinc/propgcc
- PropGCC cross-compilation README: https://github.com/parallaxinc/propgcc/blob/master/README.cross
- Parallax: https://www.parallax.com/

---

## 📝 Summary

`propeller-elf-gcc` is the main compiler command for C development in the PropGCC ecosystem. Learn it like any other GCC cross-compiler, but pay special attention to Propeller memory models and the separate `propeller-load` step.
