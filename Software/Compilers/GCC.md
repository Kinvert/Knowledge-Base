# GCC (GNU Compiler Collection)

**GCC (GNU Compiler Collection)** is a free and open-source suite of compilers developed by the [[Free Software Foundation]] as part of the GNU Project. It supports a wide range of programming languages and hardware architectures, forming the foundation of much of today’s embedded and systems software, including Linux and many robotics toolchains.

---

## ⚙️ Overview

GCC is one of the most mature compiler systems in existence, supporting languages like **C**, **C++**, **Fortran**, **Ada**, and **Go**.  
It provides frontends for these languages and backends for dozens of **ISAs (Instruction Set Architectures)**, including x86, ARM, MIPS, RISC-V, and PowerPC.  

Unlike [[LLVM]], GCC performs most of its optimizations internally, without producing an intermediate representation (IR) accessible to other tools. It’s especially common in **embedded systems**, where hardware vendors provide `arm-none-eabi-gcc`, `avr-gcc`, etc.

---

## 🧠 Core Concepts

- **Frontends**: Parse and translate high-level code (e.g., C or C++) into an internal representation.
- **Middle-end**: Optimizes code across languages and architectures.
- **Backends**: Generate assembly and machine code for specific ISAs.
- **Cross-compilation**: Build code for targets different from the host machine using a cross-toolchain (e.g., `arm-none-eabi-gcc`).
- **Linking and Assembling**: Integrates with `binutils` tools like `ld`, `as`, and `objcopy`.

---

## ⚖️ Comparison Chart

| Feature | GCC | [[LLVM]] | [[Zig]] |
|----------|-----|----------|---------|
| License | GPL | Apache 2.0 | MIT |
| Architecture Support | Extremely Broad | Broad but selective | LLVM-dependent |
| Compilation Model | Monolithic (internal passes) | Modular (IR + passes) | LLVM frontend |
| Cross Compilation | Mature toolchains | Supported | Easier via built-in cross |
| Embedded Use | Dominant | Growing | Emerging |
| Speed of Compilation | Generally slower | Faster incremental builds | Similar to LLVM |
| Code Optimization | Very mature | State-of-the-art | Same as LLVM |

---

## 🔩 Developer Tools

- **`gcc`** – Core compiler for C, C++ etc.  
- **`g++`** – C++-specific compiler driver.  
- **`ld`**, **`as`**, **`objcopy`** – Part of [[GNU Binutils]].  
- **`make`** – Often used in conjunction with GCC for build automation.  
- **`newlib`** or **`musl`** – Common standard C libraries for embedded GCC targets.  
- **GDB (GNU Debugger)** – Integrates tightly with GCC for debugging compiled binaries.

---

## 🧰 Use in Robotics and Embedded Systems

GCC is the **default compiler** for most microcontroller SDKs and real-time operating systems used in robotics:

- **STM32 (ARM Cortex-M)**: Uses `arm-none-eabi-gcc` via ST’s CubeIDE or [[PlatformIO]].
- **AVR (Arduino)**: `avr-gcc` is the standard toolchain.
- **RISC-V Boards**: Supported through `riscv64-unknown-elf-gcc`.
- **ESP32 / Xtensa**: Uses GCC-based toolchains from Espressif.
- **Linux Robotics Frameworks (e.g., [[ROS]])**: Often compiled using GCC on x86 or ARM64.

---

## ✅ Strengths

- Extremely stable and well-tested across decades.  
- Broadest ISA and language support of any compiler.  
- Fully open-source with no external dependencies.  
- Mature cross-compilation toolchains for embedded devices.  
- Strong ecosystem (GDB, Make, CMake, Binutils).  

---

## ❌ Weaknesses

- Compilation can be slower compared to LLVM-based compilers.  
- No easy-to-use intermediate representation (IR) for analysis tools.  
- Code generation sometimes produces slightly larger binaries than LLVM.  
- Licensing (GPL) can complicate integration in proprietary toolchains.  

---

## 🧭 Related Concepts

- [[LLVM]] (Low-Level Virtual Machine)
- [[Zig]] (Programming Language)
- [[ISA]] (Instruction Set Architecture)
- [[ARM Architecture]]
- [[RISC-V]] (Open ISA)
- [[Binutils]]
- [[Make]]
- [[GDB]] (GNU Debugger)
- [[STM32]]
- [[Cross Compilation]]

---

## 🔗 External Resources

- GCC Official Site: https://gcc.gnu.org  
- GCC ARM Embedded: https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain  
- RISC-V GCC Port: https://github.com/riscv-collab/riscv-gnu-toolchain  
- ESP32 GCC Docs: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/compiler.html  
- GCC Internals Manual: https://gcc.gnu.org/onlinedocs/gccint/  

---

## 🏁 Summary

GCC remains the **gold standard** for compiling code across countless architectures, especially in embedded and robotics applications.  
While [[LLVM]] and [[Zig]] provide modern modular alternatives, GCC’s long-standing support, ecosystem maturity, and extensive hardware coverage make it indispensable for microcontrollers like STM32, AVR, and RISC-V.  
