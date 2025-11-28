# Zig Struct

Zig `struct` is a compact, expressive composite type that covers many of the same use cases as C `struct` — grouping heterogeneous fields — but with a different set of guarantees, defaults, and language features. Zig intentionally separates *type-level* guarantees (alignment, layout, and ABI) from the developer's intent, giving you explicit tools (`extern`, `packed`, `@cImport`, `comptime`) when you need C-like behavior and freedom otherwise. This note dives deep into Zig structs, their semantics, how they differ from C structs, pitfalls, and practical guidance for systems and RL engineers.

---

## 🔎 Overview

- Zig `struct` is a first-class type that can contain fields, `const`/`var` declarations, functions (methods), `comptime` declarations, and nested types.
- By default, Zig **does not guarantee in-memory field order or total size** for a plain `struct` — the compiler may rearrange or optimize the layout unless you request a specific ABI/layout.
- When you need a deterministic memory layout for FFI, serialization, or hardware/packet formats, use `extern struct` (C ABI layout) or `packed struct` (bit/byte-packed backing with guaranteed order).

---

## 🧩 Core Concepts & Differences vs C

- **Default layout**:  
  - Zig: no guaranteed field ordering or padding for `struct`. Compiler may reorder for optimization. Use `extern` or `packed` to force layout.
  - C: field order and padding are defined by the ABI; the compiler keeps declared order (subject to ABI alignment rules).
- **Explicit ABI vs flexible layout**:  
  - Zig gives you `extern struct` to match C ABI exactly; otherwise Zig is free to optimize layout.
- **Packed structs**:  
  - Zig's `packed struct` gives deterministic bit/byte packing and field ordering (least significant to most significant). `packed` types are represented with a backing integer. Some restrictions apply (e.g., historically issues with arrays inside packed structs).
- **Methods & functions**:  
  - Zig allows functions declared inside a `struct` (methods) and can take `self` as `*T` or `T`. There is no built-in mechanism to add methods to a type from another file later (extension methods outside the type were historically requested but not the default).
- **Comptime & reflection**:  
  - Zig supports `comptime` execution and compile-time reflection over types and struct fields; you can generate and manipulate struct-related code at compile time. This is far more powerful than C's preprocessor macros.

---

## 🧠 Memory Layout & ABI

**Plain `struct` (Zig)**  
- No strict guarantee about field ordering or total size. The compiler may choose any layout that preserves the abstract semantics. Use plain structs when layout determinism is *not* required, which lets Zig perform optimizations.

**`extern struct`**  
- Forces memory layout to match the target C ABI. Use this for FFI with C libraries, syscalls, or binary interfaces. Example inline: `const S = extern struct { a: i32, b: u8 };`.

**`packed struct`**  
- Fields have defined order and are packed into the smallest possible ABI-aligned integers consistent with target endianness. Useful for binary protocols and hardware registers. Example inline: `const P = packed struct { a: u3, b: u5 };`. Be mindful of restrictions (e.g., arrays and some complex types have historically caused issues).

**Why Zig chooses this design**  
- By defaulting to flexible layout, Zig trades off deterministic binary layout for potential performance/size gains while giving explicit opt-in for deterministic ABI semantics.

---

## 🛠 Initialization, Defaults & Mutability

- Struct literals use the familiar `Type{ .field = value, ... }` or positional-like initialization when available. Inline example: `const v = Vec3{ .x = 1.0, .y = 2.0, .z = 3.0 };`.
- Fields may be declared with types only; values are given in instances. Zig enforces initialization rules at compile time.
- Zig's pointer and mutability semantics: a `const` value of a struct makes the entire instance immutable; pointers are typed and mutability is explicit (`*T` vs `*const T` semantics are explicit in how you pass `self`).

---

## 🧩 Comptime Fields & Metaprogramming

- Zig supports `comptime` declarations and `comptime` evaluation that can interact with struct definitions. You can use `comptime` to generate fields, compute sizes, or perform validation at compile time.
- There are nuanced behaviors with `comptime` fields: they're conceptually different from runtime fields and have stricter initialization/assignment semantics. In some cases a `comptime` struct field is effectively a compile-time constant (and cannot be mutated at runtime). Use global `const` values when appropriate.

---

## 🧭 Methods, Associated Functions & Interfaces

- Zig structs can contain functions which act like methods; you explicitly take `self` when desired:
  - Inline example: `fn add(self: *Vec3, other: Vec3) void { ... }`
- There is no built-in trait/interface dispatch tied to structs the way C++ has member functions with virtual tables — Zig prefers explicitness and zero-cost abstractions.
- You cannot (today) declare methods for a struct *outside* its declaration in a separate file as a language-wide extension (this has been discussed in the community and has trade-offs).

---

## 🔍 Reflection & Introspection

- Zig exposes compile-time reflection primitives (`@typeInfo`, `@field`, `@sizeOf`, etc.) that let you introspect struct fields, names, types, and attributes at `comptime`. This enables code generation patterns (e.g., automatic serializers, debug printers) without runtime cost. Example inline: `const info = @typeInfo(MyStruct);`.
- Reflection + `comptime` makes it easy to write generic utilities for RL state serialization, zero-copy views, and deterministic marshaling.

---

## 🧪 Common Pitfalls & Gotchas

1. **Assuming field order** — plain Zig `struct` may reorder fields; if you depend on order for serialization/FFI, use `extern` or `packed`.
2. **Packed struct restrictions** — packed structs are backed by integers and certain constructs (e.g., arrays) have historically produced errors or limitations; test packed types carefully on your target Zig version.
3. **Comptime fields confusion** — comptime fields are evaluated differently; don’t expect them to behave as runtime mutable fields. Prefer global `const` when you need a symbolic constant.
4. **Method scoping** — you can’t (easily) add methods to a type from another file; plan module boundaries accordingly.

---

## 🧾 Comparison Chart — Zig Struct vs C Struct

| Concern | Zig `struct` (default) | Zig `extern`/`packed` | C `struct` |
|---|---:|---:|---|
| Field order guarantee | No (compiler may reorder). | Yes (`extern` matches C ABI; `packed` enforces declared order). | Yes (declared order; ABI dictates padding) |
| ABI compatibility | Not guaranteed | Guaranteed for `extern` | Guaranteed |
| Bit-level packing | Use `packed` | `packed` with backing integer | Manual bitfields (implementation-defined behavior) |
| Reflection / Comptime | Strong via `@typeInfo`, `comptime` | Same; useful to validate layout at compile time | None natively (macros only) |
| Method extension outside type | Not supported (by design) | Same | Supported (functions can operate on structs; C has no methods but you can define helpers) |
| Tooling for serialization | `comptime` + reflection simplifies | Works, but check backing representation | Manual, error-prone |
| FFI friendliness | Use `extern` | Use `extern` | Native |

Key notes: Zig intentionally separates *flexible* default behavior from *deterministic* ABI semantics; C assumes determinism by default.

---

## ✅ Best Practices (Practical Guidance)

- For FFI with C libraries or kernel/syscall structs: **use `extern struct`**. Always verify `@sizeOf` and `@alignOf` at `comptime` for CI checks.
- For binary protocols / bit-packed layouts: **use `packed struct`** but test across targets and Zig versions due to historical limitations.
- For performance where layout doesn't matter: prefer plain `struct` and let the compiler optimize; use `@sizeOf` and profiling to confirm behavior.
- Use `comptime` reflection to auto-generate serializers/deserializers, validation code, and unit tests.
- Keep methods close to struct declarations (module-local) to avoid cross-file extension issues.

---

## 🔬 Examples (single-line inline snippets)

- Declare typical struct: `const Vec3 = struct { x: f32, y: f32, z: f32 };`
- Extern struct for FFI: `const CHeader = extern struct { id: u32, flags: u8 };`
- Packed struct example: `const Packed = packed struct { a: u3, b: u5 };`
- Comptime introspection: `const info = @typeInfo(MyStruct);` (then iterate fields at comptime)

> Note: examples are inline here per your format request. For runnable examples test with `zig` and validate `@sizeOf`/`@alignOf`.

---

## 🧾 Further Reading & References

- Zig Language Reference (master): general docs and struct reference.
- Zig Guide — Structs: language-basics and practical advice.
- Zig Guide — Extern Structs (FFI).
- Packed Structs explanation and examples.
- Community discussions / issues on comptime fields and packed array limitations (useful for edge cases).

---

## 🔗 Related Notes

- [[Zig]] (language overview)  
- [[Comptime]] (compile-time programming)  
- [[FFI]] (Foreign Function Interface)  
- [[Packed Structs]] (binary protocols)  
- [[extern struct]] (C ABI compatibility)

---

## 🧭 Summary

Zig structs are powerful and designed for explicitness: defaults favor compiler freedom and performance, while `extern` and `packed` give you deterministic layout when you need it. The real strengths are the tight integration with `comptime` reflection and the ability to validate or generate struct-related code at compile time — capabilities that are typically cumbersome or impossible in C. At the same time, be cautious: because Zig defaults to flexible layout, you must be deliberate when you require binary compatibility or precise packing.
