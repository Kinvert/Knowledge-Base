# ISA (Instruction Set Architecture)

The **Instruction Set Architecture (ISA)** defines the abstract model of a computer's processor — the set of instructions it can execute, how it accesses memory, and how software interacts with the hardware. It determines software compatibility between processors and directly impacts compiler and language support, including whether languages like [[Zig]] or toolchains based on [[LLVM]] can target a given chip.  

---

## ⚙️ Overview

The ISA acts as the **interface between hardware and software**, specifying the available registers, instruction formats, addressing modes, and binary encoding. Software such as operating systems, compilers, and firmware must conform to this definition to run on a processor.

Zig and LLVM depend heavily on compiler backends that translate high-level code into machine instructions for a specific ISA. If an ISA is unsupported by LLVM (and thus by Zig), the language cannot directly target that chip without developing a new backend.

---

## 🧠 Core Concepts

- **Instruction Set**: Defines operations (add, load, store, branch, etc.) supported by the CPU.
- **Registers**: The small, fast memory cells used during instruction execution.
- **Endianness**: Defines byte order (little-endian or big-endian).
- **Addressing Modes**: How memory locations are accessed.
- **Privilege Levels**: Determines user vs. kernel mode instructions.
- **Compatibility Layer**: Defines whether an ISA can emulate or translate to another (e.g., ARM → x86).

---

## ⚖️ Comparison Chart

| Architecture | Example Chips | Supported by LLVM | Can Zig Target It? | Notes |
|---------------|----------------|------------------|--------------------|-------|
| **x86 / x86_64** | Intel, AMD | ✅ | ✅ | Widely supported; default Zig backend |
| **ARM / ARM64 (AArch64)** | STM32 (Cortex-M), Raspberry Pi | ✅ | ✅ | Common in embedded and mobile |
| **RISC-V** | SiFive, Kendryte | ✅ | ✅ | Open standard; rapidly growing |
| **Propeller (Parallax)** | Propeller 1 | ❌ | ❌ | Custom multi-core design, not LLVM-based |
| **Propeller 2 (Parallax)** | P2X8C4M64P | ⚠️ Partial (experimental) | ⚠️ Possible with custom backend | Requires LLVM backend or assembler integration |
| **MIPS** | Older routers, embedded | ✅ | ✅ | Legacy but still supported in LLVM |
| **ESP32 (Xtensa)** | ESP32, ESP8266 | ⚠️ Limited | ⚠️ Possible via external toolchain | Xtensa partially supported by LLVM patches |
| **PowerPC** | Legacy Macs, some robotics | ✅ | ✅ | Supported but fading |

---

## 🔩 Developer Tools and Compilers

- **[[LLVM]] (Low-Level Virtual Machine)**: Provides backends for x86, ARM, RISC-V, and more. Zig leverages LLVM for code generation.
- **[[Zig]]**: Uses LLVM backends, so it can only target ISAs LLVM supports.
- **[[GCC]] (GNU Compiler Collection)**: Supports a broad range of ISAs, sometimes earlier than LLVM.
- **Parallax Toolchains**: Use `Spin` or `Propeller Assembly (PASM)` for Propeller 1 and 2; not LLVM-based.
- **STMicroelectronics Toolchains**: Typically use `arm-none-eabi-gcc` for STM32 microcontrollers.

---

## 🧰 Use Cases in Robotics

- **STM32 (ARM Cortex-M)**: Common in motor controllers, sensors, and low-level embedded systems. Compatible with Zig via LLVM ARM backend.
- **RISC-V Boards**: Emerging option for open-source robotics and AI accelerators.
- **Parallax Propeller 2**: Excellent for parallel sensor reading or signal generation, but limited compiler support makes high-level languages difficult.

---

## ✅ Strengths

- Standardized ISA enables broad compiler and OS support.
- LLVM-based ISAs (x86, ARM, RISC-V) benefit from powerful language ecosystems.
- Simplifies cross-platform development.

---

## ❌ Weaknesses

- Proprietary or custom ISAs (like Propeller) isolate developers from modern languages.
- Unsupported ISAs require writing new LLVM backends — complex and time-consuming.
- Toolchains are often vendor-specific and fragmented in embedded contexts.

---

## 🧭 Related Concepts

- [[Zig]] (Programming Language)
- [[LLVM]] (Low-Level Virtual Machine)
- [[GCC]] (GNU Compiler Collection)
- [[STM32]] (Microcontroller Family)
- [[RISC-V]] (Open ISA)
- [[ARM Architecture]]
- [[Assembly Language]]
- [[Embedded Systems]]
- [[Parallax Propeller 1]]
- [[Parallax Propeller 2]]

---

## 🔗 External Resources

- LLVM Target List: https://llvm.org/docs/CodeGenerator.html  
- Zig Targets Reference: https://ziglang.org/documentation/master/#Supported-Targets  
- Parallax Propeller 2 Documentation: https://docs.parallax.com/propeller2  
- STM32 Developer Resources: https://www.st.com/en/development-tools.html  
- RISC-V Foundation: https://riscv.org/  

---

## 🗂️ Summary

The ISA determines whether modern toolchains like LLVM and Zig can target a given chip.  
While STM32 and RISC-V are fully supported due to their ARM and open-standard architectures, the Parallax Propeller series uses custom ISAs that fall outside LLVM’s ecosystem. To enable Zig support, a dedicated LLVM backend (or alternate translation layer) would need to be written — a major engineering task typically justified only for widely used platforms.

