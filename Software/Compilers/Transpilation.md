# 🔁 Transpilation

Transpilation refers to the process of converting source code from one high-level language to another while preserving the program’s structure, behavior, and semantics. Unlike compilation—which typically lowers code into machine or bytecode—transpilation focuses on language-to-language transformation, often to improve portability, readability, or maintainability.

This technique is widely used in systems programming, legacy code modernization, web development, and game modding. In contexts like researching older games (e.g., Pokémon), transpilation provides a powerful method for understanding original behavior and migrating the entire codebase into a safer, modern language such as Zig.

---

## 📘 Overview

A **transpiler** reads code written in Language A, constructs an intermediate representation (IR), and outputs equivalent code in Language B. This is useful when:

- Migrating from an older or unsupported language  
- Rewriting disassembled code into a maintainable form  
- Creating safer implementations in languages with stronger guarantees (Zig, Rust, Ada)  
- Porting software across platforms where the original language lacks support  

Transpiling often follows a pipeline similar to a compiler but without lowering to machine code.

---

## 🧠 Core Concepts

- **Lexing & Parsing**: Converting raw text into tokens and then an AST.
- **AST (Abstract Syntax Tree)**: Structured representation of a program’s logic.
- **Semantic Analysis**: Resolving types, variables, control flow, and data dependencies.
- **Code Generation**: Emitting equivalent code in the target language.
- **Equivalence Guarantee**: The transpiler must preserve semantics even if the syntax differs greatly.
- **Decompilation vs Disassembly vs Transpilation**:  
  - Disassembly → machine code → assembly  
  - Decompilation → assembly/machine code → pseudo high-level  
  - Transpilation → high-level → high-level

---

## 🧩 Transpiling Disassembled Pokémon Code to Zig

Game Boy Advance (GBA) Pokémon games have well-documented disassemblies (e.g., `pokeemerald`, `pokefirered`). These provide **C-like representations** of the original assembly, often including reconstructed functions and data tables.

To convert such code into pure Zig, developers typically follow these steps:

### 1. **Acquire the Disassembly**
Projects like `pret/pokeemerald` provide:
- Readable C code  
- Metadata  
- Build scripts mimicking the original ROM layout  

This gives you a near-complete high-level starting point.

### 2. **Create a Mapping Between C Constructs and Zig Constructs**
Examples:
- `typedef` → `const name = type;`
- `struct {...}` → Zig `struct { ... }`
- `enum` → Zig `enum(u32) { ... }` (explicit tag size is often required)
- `#define` → `const`
- Raw pointers → `[*c]T` or `[ *]T` depending on semantics
- GBA hardware registers → `extern` declarations or `volatile` access

Care must be taken because Zig is stricter than C:
- No implicit casts  
- Explicit integer sizes  
- Defined pointer alias rules  

This makes the Zig port safer but requires more upfront work.

### 3. **Handle Inline Assembly or Hardware Access**
GBA code uses:
- Memory-mapped I/O  
- BIOS routines  
- Highly optimized assembly blocks  

In Zig:
- Use `@ptrFromInt` for hardware addresses  
- Use `volatile` types  
- Rewrite assembly manually or provide Zig equivalents  

### 4. **Replace Build System with `zig build`**
Using Zig's cross-compilation capabilities:
- GBA target = ARM7TDMI (ARMv4T)  
- Build using `-Dtarget=arm-none-eabi`  
- ROM layout can be reconstructed via linker scripts or Zig’s link options  

### 5. **Ensure Behavioral Fidelity**
To maintain identical game logic:
- Test against the original ROM using hashes  
- Reuse known test ROMs or emulator test harnesses  
- Gradually replace C files with Zig equivalents  

### 6. **Automate the Transpilation**
For large projects:
- Write a small transpiler or AST converter  
- Tools like tree-sitter can help parse C  
- Emit Zig code through templates

This becomes essential for thousands of functions.

---

## 📊 Comparison Chart

| Aspect                          | Transpilation          | Compilation               | Decompilation          | Manual Rewrite        |
|----------------------------------|-------------------------|----------------------------|------------------------|------------------------|
| Input                            | High-level code         | High-level code            | Machine/assembly       | Any                   |
| Output                           | High-level code         | Machine/bytecode           | High-level pseudocode  | High-level code        |
| Best For                         | Porting                 | Execution                  | Reverse engineering    | Clean redesign         |
| Semantic Guarantees              | Strong (if done well)   | Strong                     | Weak                   | Very strong           |
| Automation Level                 | High                    | High                       | Medium                 | Low                   |

---

## 🧰 Use Cases

- Porting legacy C code to Zig  
- Understanding game logic and structures  
- Making modding tools for older games  
- Improving readability of reverse-engineered code  
- Migrating embedded systems into a modern environment  

---

## 🏆 Strengths

- Preserves behavior while modernizing code  
- Produces readable, idiomatic high-level output  
- Useful for reverse-engineering and archival work  
- Helps integrate old systems with modern ecosystems  

---

## ❌ Weaknesses

- Complex systems require heavy semantic analysis  
- Implicit behaviors (undef behavior) must be made explicit  
- Disassembled code may be incomplete or incorrect  
- Transformation can be extremely time consuming  

---

## 🔧 Developer Tools

- tree-sitter parsers (C/Zig)
- LLVM/Clang tooling (AST extraction)
- wasm-bind based inspection for WASM transpilers
- Ghidra, IDA, Radare2 (for reverse engineering)
- Custom scripts to transform C → Zig ASTs
- Zig’s `std.zig.Ast` for emitting Zig code programmatically

---

## 🧭 How It Works

1. Parse the source language into an AST.  
2. Optionally normalize or desugar constructs.  
3. Perform semantic and type analysis.  
4. Translate AST nodes into the target language’s AST.  
5. Print the transformed AST as formatted source.  

The process resembles a compiler pipeline but stops before lowering to machine code.

---

## 📎 Related Concepts/Notes

- [[Compilers]] (General compiler architecture)
- [[Reverse Engineering]] (Process for recovering structure)
- [[Zig]] (Modern systems language used for ports)
- [[AST]] (Abstract Syntax Tree)
- [[Decompilation]] (Related but different goal)
- [[C]] (Source of most Pokémon disassemblies)
- [[ARM Architecture]] (GBA CPU target)
- [[Ghidra]] (Reverse engineering platform)

---

## 📚 External Resources

- PRET Pokémon disassemblies (`pret/pokeemerald`, `pret/pokefirered`)
- Zig documentation for cross-compiling
- GBAtek hardware reference
- Resources on reverse engineering and ROM hacking
- Tree-sitter C parser documentation

---

## 🧩 Capabilities

- Converts large codebases to a more modern language  
- Helps preserve video game history by making code readable  
- Improves debugging and mod support  
- Enables safe reimplementation using Zig’s guarantees  

---
