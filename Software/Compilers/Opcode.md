# 🧮 Opcode

An **opcode** (operation code) is the smallest unit of encoded instruction in a machine’s instruction set architecture (ISA). It tells the CPU *what* operation to perform (e.g., `ADD`, `LOAD`, `JMP`). Combined with operands (registers, immediate values, memory addresses), opcodes form the foundation of all machine-level execution.

Opcodes are essential for understanding compilers, decompilers, reverse engineering, embedded systems, VM design, and low-level performance work.

---

## 📘 Overview

Opcodes sit at the core of every processor architecture—from x86 and ARM to RISC-V, GPU shader ISAs, and custom VM bytecodes. They represent the operation type encoded inside machine instructions.

Every instruction’s binary encoding includes:
- The **opcode**
- Operand descriptors
- Flags, addressing modes, immediates, etc.

Opcodes define the CPU’s “vocabulary,” telling it what instruction to execute.

In reverse engineering and game modding (e.g., Pokémon, Nintendo systems), manually reading opcodes is often required to understand a system's underlying behavior or to rewrite low-level logic in a language like Zig.

---

## 🧠 Core Concepts

- **ISA (Instruction Set Architecture)**: Defines opcode values and instruction formats.  
- **Mnemonic**: Human-readable symbolic name mapped to an opcode (`ADD`, `MOV`, `LDR`).  
- **Encoding Format**: How the bits of an opcode and its operands are arranged in machine code.  
- **Operands**: Registers, immediates, memory addresses.  
- **Endianness**: Byte ordering affects how opcodes appear in memory.  
- **Variable vs Fixed Length Instructions**:  
  - x86 → variable length  
  - ARM Thumb → 16-bit  
  - RISC-V → fixed 32-bit (with extensions)

Working with opcodes often means understanding the **binary → mnemonic** mapping.

---

## 📊 Comparison Chart

| ISA / System                | Opcode Length | Encoding Style       | Typical Use Case           | Notes                        |
|-----------------------------|---------------|-----------------------|----------------------------|------------------------------|
| x86 / x86-64                | Variable      | Highly complex       | Desktop CPUs               | Long legacy, many prefixes   |
| ARM (ARMv4–ARMv8)          | Mostly fixed  | RISC-like            | Mobile, consoles, embedded | GBA uses ARMv4T (Thumb)      |
| ARM Thumb                  | 16-bit        | Compact              | GBA, DS                    | Many Pokémon engine routines |
| RISC-V                     | 32-bit        | Clean, orthogonal    | Modern embedded systems    | Great for custom toolchains  |
| Game Boy (SM83)            | 8-bit         | Simple               | Pokémon Gen 1/2            | Very modding-friendly        |
| WASM                       | Variable LEB  | Stack-based          | Browsers, sandboxing       | No traditional registers     |

---

## 🔬 Opcodes in Reverse Engineering Pokémon Games

Pokémon titles use different architectures depending on generation:

- **Gen 1–2** → Game Boy (SM83, similar to Z80)  
- **Gen 3** → GBA (ARM7TDMI, ARMv4T with ARM + Thumb instructions)  
- **Gen 4–5** → Nintendo DS (ARM9/ARM7)  

If you want to rewrite Pokémon code in Zig, knowing the opcodes is vital.

### Why?

1. **Understanding Disassembly**  
   Disassembly output is driven entirely by opcodes.  
   For example, ARM Thumb instruction `0x4770` decodes to `BX LR`.

2. **Correct Semantics in Zig**  
   You must replicate the instruction’s behavior exactly.  
   Example:
   - `LDR r0, [r1, #4]` → In Zig: pointer math + load + type casting.

3. **Reconstructing Control Flow**  
   Jumps, branches, and calls (`BL`, `BNE`, `B`, `BX`) determine:
   - Function boundaries  
   - State machines  
   - Game engine behavior  

4. **Manual Optimization or Matching ROMs**  
   Pokémon rebuilding communities aim for *matching builds* where the output binary exactly matches the original.  
   This requires opcode-perfect understanding.

---

## 🧩 Opcodes When Rewriting to Pure Zig

Zig does not expose high-level equivalents to every CPU instruction. Instead, you express intent in safe, portable operations, and Zig's compiler emits the correct target opcodes.

However, when behavior **must match** an original ROM:

- You may need inline assembly (`asm`), but in pure Zig ports, you usually replace logic with equivalent Zig.
- Opcodes involving hardware registers (GBA I/O) are replaced with:
  - `volatile` pointers  
  - explicit memory accesses  
  - structured data types defined in Zig  

### Example Mappings

| Opcode (GBA)       | Behavior            | Zig Equivalent Concept |
|--------------------|---------------------|-------------------------|
| `LDR`              | Load from memory    | Pointer deref          |
| `STR`              | Store to memory     | Pointer assign          |
| `ADD`, `SUB`       | Arithmetic          | Normal Zig operators    |
| `BL`, `BX`         | Calls / returns     | Function calls          |
| `CMP`, `BNE`, `BEQ`| Branching           | `if`, `switch`          |
| `SWI`              | BIOS call           | Inline assembly or Zig reimplementation |

Precise mapping often requires consulting documentation like GBAtek.

---

## 📚 Use Cases

- Designing virtual machines and custom bytecodes  
- Reverse engineering firmware or ROMs  
- Understanding control flow and memory behavior of legacy systems  
- Writing compilers or interpreters  
- Translating assembly → Zig or Rust  

---

## 🏆 Strengths

- Provides lowest-level insight into CPU behavior  
- Enables precise reproduction of original program logic  
- Critical for debugging at machine level  
- Enables custom toolchains, emulators, transpilers  

---

## ❌ Weaknesses

- Extremely architecture-specific  
- Hard to maintain manually  
- Requires strong understanding of binary formats  
- Disassembly may be ambiguous without context  

---

## 🛠️ Developer Tools

- Ghidra (opcode decoding + lifting)
- IDA Pro
- Radare2
- LLVM MC layer (for many ISAs)
- Zig inline assembly and `@intFromPtr` utilities
- Emulator debugging tools (mgba, BGB, no$gba)

---

## 🔗 Related Concepts/Notes

- [[ISA]] (Instruction Set Architecture)
- [[Assembly]] (Human mnemonics for opcodes)
- [[ARM Architecture]]
- [[Reverse Engineering]]
- [[Zig]] (For rewriting logic)
- [[Decompilation]]
- [[GBA Hardware]] (For Gen 3 Pokémon)
- [[Virtual Machines]] (Opcodes define VM behavior)
- [[Bytecode]] (Opcode-based virtual ISAs)

---

## 🔗 External Resources

- ARM ARM (Architecture Reference Manual)
- GBAtek (comprehensive GBA opcode and hardware doc)
- Game Boy CPU manual
- RISC-V spec  
- Zig inline assembly documentation

---
