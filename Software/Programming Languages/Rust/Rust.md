# Rust

**Rust** is a systems programming language focused on safety, speed, and concurrency. Developed by Mozilla and now maintained by the Rust Foundation, it uses a unique ownership model with compile-time borrow checking to guarantee memory safety without a garbage collector. Rust is popular for systems programming, [[WebAssembly]], CLI tools, and anywhere performance and reliability matter.

---

## 📚 Overview

Rust aims to provide the performance of [[C]] and [[C++]] while eliminating entire classes of bugs—null pointer dereferences, buffer overflows, data races—at compile time. Its ownership system enforces strict rules about how memory is accessed and shared.

Key highlights:
- Memory safety without garbage collection
- Zero-cost abstractions
- Fearless concurrency via ownership rules
- Rich type system with pattern matching
- Cargo package manager and build system
- Strong [[WebAssembly]] support

---

## 🧠 Core Concepts

- **Ownership**
  Every value has a single owner; when the owner goes out of scope, the value is dropped

- **Borrowing**
  References can borrow values immutably (`&T`) or mutably (`&mut T`), but not both simultaneously

- **Lifetimes**
  Annotations that tell the compiler how long references are valid

- **Traits**
  Rust's interface system for shared behavior (like interfaces or typeclasses)

- **Pattern Matching**
  Exhaustive `match` expressions for control flow

- **Result & Option**
  No null—use `Option<T>` for optional values, `Result<T, E>` for errors

- **Cargo**
  Build system and package manager; `Cargo.toml` defines dependencies

---

## 📊 Comparison Chart

| Aspect | Rust | C | C++ | Go | Zig |
|--------|------|---|-----|----|----|
| Memory Safety | Compile-time (borrow checker) | Manual | Manual | GC | Optional runtime checks |
| GC | None | None | None | Yes | None |
| Concurrency | Ownership prevents data races | Manual | Manual | Goroutines + channels | Manual |
| Compile Speed | Slow | Fast | Slow | Fast | Fast |
| Binary Size | Moderate | Small | Moderate | Larger | Small |
| Learning Curve | Steep | Moderate | Steep | Easy | Moderate |
| C Interop | FFI | Native | Native | cgo | Seamless |
| Ecosystem | Large (crates.io) | Fragmented | Large | Growing | Small |

---

## 🔧 Use Cases

- Systems programming (OS, drivers, embedded)
- [[WebAssembly]] modules (wasm-pack, wasm-bindgen)
- CLI tools (ripgrep, fd, bat, exa)
- Game engines and game development
- Network services and async runtimes (Tokio, async-std)
- Cryptography and security-critical code
- Browser engines (Servo, parts of Firefox)
- [[Elixir]] native extensions via [[Rustler]]

---

## ✅ Pros

- Memory safety guaranteed at compile time
- No garbage collector pauses
- Excellent performance comparable to C/C++
- Prevents data races in concurrent code
- Powerful type system catches many bugs early
- Cargo is one of the best package managers
- Strong community and documentation
- First-class WebAssembly support

---

## ❌ Cons

- Steep learning curve (ownership, lifetimes)
- Slow compile times for large projects
- Fighting the borrow checker can be frustrating
- Less flexibility than C for low-level tricks
- Async Rust has complexity (Pin, lifetimes in futures)
- Smaller ecosystem than C/C++ for some domains

---

## 🔧 Common Commands

```bash
# Create new project
cargo new my_project

# Build project
cargo build

# Build optimized release
cargo build --release

# Run project
cargo run

# Run tests
cargo test

# Check without building
cargo check

# Format code
cargo fmt

# Lint with Clippy
cargo clippy

# Add dependency
cargo add serde
```

---

## 🔧 Basic Example

```rust
use std::collections::HashMap;

fn main() {
    let mut scores: HashMap<String, i32> = HashMap::new();

    scores.insert(String::from("Blue"), 10);
    scores.insert(String::from("Red"), 50);

    for (team, score) in &scores {
        println!("{}: {}", team, score);
    }

    // Pattern matching with Option
    match scores.get("Blue") {
        Some(score) => println!("Blue team: {}", score),
        None => println!("Blue team not found"),
    }
}
```

---

## 🔩 Compatible Items

- [[WebAssembly]] - First-class WASM target
- [[Cargo]] - Package manager
- [[Tokio]] - Async runtime
- [[Serde]] - Serialization framework
- [[Actix]] / [[Axum]] - Web frameworks
- [[Rustler]] - Elixir NIF bindings
- [[PyO3]] - Python bindings
- [[LLVM]] - Compiler backend

---

## 🔗 Related Concepts

- [[C]] (Performance comparison, FFI)
- [[C++]] (Similar domain, different safety model)
- [[Zig]] (Alternative systems language)
- [[Go]] (Different tradeoffs for systems work)
- [[WebAssembly]] (Common compile target)
- [[Rustler]] (Elixir integration)
- [[LLVM]] (Compiler infrastructure)

---

## 📚 External Resources

- [Rust Official Site](https://www.rust-lang.org/)
- [The Rust Book](https://doc.rust-lang.org/book/)
- [Rust by Example](https://doc.rust-lang.org/rust-by-example/)
- [crates.io](https://crates.io/) - Package registry
- [Rustlings](https://github.com/rust-lang/rustlings) - Small exercises
- [Are We Web Yet?](https://www.arewewebyet.org/)
