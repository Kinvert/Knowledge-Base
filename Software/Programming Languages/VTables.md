# VTables

**VTables** (Virtual Method Tables) are a core mechanism used by many object-oriented languages and runtimes—particularly **C++**—to support **dynamic dispatch** (runtime polymorphism). When a class contains virtual methods, the compiler generates a table of function pointers (the vtable) and stores a pointer to that table (the vptr) in each object instance. This allows the correct override of a method to be chosen at runtime.

---

## 🛰️ Overview

A vtable is essentially a lookup table mapping virtual method indices to actual function implementations. When a virtual method is invoked, the object’s vptr directs the call through the vtable to the proper override.

This mechanism is fundamental to implementing polymorphism in C++ and influenced how virtual dispatch works in languages like Rust (trait objects), Zig (manual tables), and COM interfaces. Understanding vtables is crucial for systems programming, ABI stability, and low-level debugging.

---

## 🧠 Core Concepts

- **Virtual Methods**: Functions resolved at runtime rather than compile time.
- **vptr (Virtual Pointer)**: Hidden pointer in objects pointing to that class's vtable.
- **VTable**: A table of function pointers corresponding to each virtual method.
- **Dynamic Dispatch**: Selecting the correct method implementation at runtime.
- **ABI (Application Binary Interface)**: Defines vtable layout and calling conventions.
- **Multiple Inheritance**: Requires multiple vptrs and complex offset adjustments.
- **Optimization**: Compilers may devirtualize calls when type is known.

---

## 📊 Comparison Chart

| Feature / Language | C++ (Classic VTables) | Rust (Trait Objects) | Go (Interfaces) | Zig (Manual Tables) | Java (JVM vtables) |
|--------------------|------------------------|------------------------|------------------|-----------------------|---------------------|
| Dispatch Model     | vtables                | fat pointers + vtables | itab structures | explicit tables       | vtables in JVM      |
| Memory Overhead    | vptr in each object    | fat pointer (data + vtable) | interface table | user-controlled      | JVM-managed         |
| Multiple Inheritance | Supported (complex)  | No                     | Yes (clean)      | N/A                   | Single inheritance  |
| ABI Stability      | Unstable across compilers | Stable within Rust | Stable | User-defined | Stable within JVM versions |
| Dynamic Dispatch Speed | Very fast          | Fast                  | Medium           | Depends              | Fast                |
| Primary Use Case   | Runtime polymorphism   | Trait polymorphism     | Interface calls  | Manual polymorphism   | OO dispatch         |

---

## 🔧 How It Works

- The compiler emits a **vtable** for each polymorphic class.
- This table contains pointers to:
  - Virtual method implementations
  - RTTI (Run-Time Type Information) in many ABIs
- Each object includes a **vptr** pointing to its class’s vtable.
- When calling a virtual method, code does:
  - Load vptr from object
  - Index into vtable by method slot
  - Indirect call through function pointer

This indirection enables runtime polymorphism.

---

## 🚀 Use Cases

- Class hierarchies in C++ where runtime behavior varies by derived type
- Plugin systems and ABI-stable extension points
- COM and WinRT interfaces (vtable-based)
- Low-level engine/graphics systems needing dynamic dispatch
- Debugging memory corruption or undefined behavior involving vptr misuse
- Devirtualization optimizations in performance-critical applications

---

## ⭐ Strengths

- Extremely fast runtime dispatch (single pointer indirection)
- Memory layout is predictable and compact
- Well-supported by static analyzers and debuggers
- Crucial for classic object-oriented designs
- Enables polymorphism even when type is unknown until runtime

---

## ⚠️ Weaknesses

- Adds a hidden pointer to objects (slightly larger footprint)
- Breaks strict POD layout in C++
- ABI varies between compilers (GCC vs MSVC vs Clang)
- Harder to reason about for cache and branch prediction
- Vulnerable to vtable pointer corruption (security implications in exploits)

---

## 🔩 Variants and Related Mechanisms

- **Itable / itab (Go)**: Similar but struct-based
- **Trait Objects (Rust)**: Fat pointer containing data ptr + vtable ptr
- **Java Virtual Tables**: Managed by the JVM
- **C++ Multiple Inheritance VTables**: Multi-pointer layouts with offsets
- **Manual Polymorphism** (Zig, C): Struct of function pointers

---

## 🛠️ Developer Tools

- `objdump` or `readelf` to inspect vtables in compiled binaries
- GDB/LLDB with `ptype` or memory inspection
- Compiler explorer (Godbolt) to view emitted vtable structures
- Clang sanitizers for catching virtual call issues (`-fsanitize=undefined`)

---

## 📚 Related Concepts

- [[Cpp]] (Programming language with vtables)
- [[OOP]] (Object-oriented programming)
- [[ABI]] (Binary interface rules)
- [[RTTI]] (Runtime Type Information)
- [[Polymorphism]] (Dynamic method selection)
- [[Memory Layout]] (Object structures)
- [[Function Pointer]] (Core mechanism behind vtables)
- [[Zig]] (Manual polymorphism approaches)
- [[Rust]] (Trait object vtables)

---

## 🔗 External Resources

- Itanium C++ ABI documentation on vtable layout
- MSVC ABI documentation
- C++ Object Model (Lippman)
- Compiler Explorer for real vtable inspection

---

## 🏁 Summary

VTables are the runtime infrastructure enabling virtual dispatch in languages like C++. They map method slots to concrete function implementations via a table of function pointers, with each object storing a pointer to its class’s vtable. This mechanism provides fast runtime polymorphism with predictable performance, at the cost of complexity in multiple inheritance and ABI variance.
