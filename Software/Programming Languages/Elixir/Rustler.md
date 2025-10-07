# Rustler

Rustler is a tool that enables seamless integration between [[Elixir]] and [[Rust]], allowing developers to write fast, memory-safe native code for use inside Elixir applications. It automates much of the complexity involved in creating NIFs (Native Implemented Functions), while preserving the fault-tolerance and safety of the [[BEAM VM]]. For robotics engineers, Rustler provides a powerful way to extend Elixir with high-performance computation, hardware interfacing, and real-time algorithms.

---

## ⚙️ Overview

Rustler bridges the gap between Elixir’s high-level concurrency model and Rust’s low-level performance and safety. By leveraging Rust’s ownership model and type safety, Rustler minimizes the risk of crashes or undefined behavior inside the BEAM. It’s ideal for performance-critical tasks where Elixir’s pure functional style might not be efficient enough — such as control systems, signal processing, or computer vision routines.

Rustler automatically handles NIF registration, compilation via Cargo, and integration into `mix` builds, making native extensions feel like first-class citizens in Elixir projects.

---

## 🧠 Core Concepts

- **NIF (Native Implemented Function)**: A function implemented in native code (Rust, C, or Zig) and callable from Elixir.
- **Safe NIFs**: Rustler ensures Rust code doesn’t crash the BEAM by preventing panics and memory errors.
- **Term Encoding/Decoding**: Rustler provides macros for converting between Elixir and Rust data types.
- **Cargo Integration**: Rustler uses Rust’s build system to compile extensions automatically.
- **Mix Tasks**: The `mix rustler.new` task scaffolds new NIF projects with minimal setup.

---

## 📊 Comparison Chart

| Feature / Tool             | **Rustler (Elixir + Rust)** | **Zigler (Elixir + Zig)** | **C NIFs** | **Port Drivers** | **CNodes** |
|-----------------------------|------------------------------|----------------------------|-------------|------------------|-------------|
| Primary Language            | Rust                         | Zig                        | C           | C / Elixir       | Elixir / C  |
| Memory Safety               | Excellent (ownership model)   | High (manual + safe APIs)  | Low         | Medium           | High        |
| Compile Integration         | Cargo + `mix` bridge          | `mix` with Zig build       | Manual      | Manual           | Manual      |
| Performance                 | High                         | High                       | High        | Medium           | Low         |
| Complexity                  | Moderate                     | Low                        | High        | Medium           | High        |
| Ecosystem Maturity          | Mature                       | Growing                    | Legacy      | Legacy           | Legacy      |
| Ideal Use Case              | Performance, safety, FFI      | Embedded, cross-compile    | Legacy code | Isolation needed | Remote IPC  |

---

## 🔩 Use Cases

- High-performance robotics computations (e.g., matrix math, SLAM preprocessing)
- Integrating Elixir with Rust robotics crates
- Writing real-time data pipelines or sensor fusion modules
- Extending [[Phoenix Framework]] APIs with optimized routines
- Creating safe hardware drivers for embedded robotics through [[Nerves]]

---

## ✅ Strengths

- Memory-safe by design (prevents most segmentation faults)
- Automatic error handling — Rust panics don’t crash BEAM
- Great ecosystem and tooling (Cargo, crates.io)
- Rich type conversions between Elixir and Rust
- Excellent documentation and active community

---

## ⚠️ Weaknesses

- Longer compile times compared to Zigler or C
- Requires understanding of both Elixir and Rust
- Slightly higher overhead for data exchange than C
- Less ideal for very small embedded targets (Zig may fit better)
- Async integration with BEAM is non-trivial without care

---

## 🧰 Developer Tools

- `mix rustler.new` — create a new Rustler NIF project  
- `mix compile` — compiles both Elixir and Rust components  
- `cargo build --release` — manual build for performance testing  
- `iex -S mix` — test Rust NIFs in the interactive shell  

---

## 🔌 Compatible Items

- [[Elixir]] (host language)
- [[Rust]] (systems programming language)
- [[BEAM VM]] (runtime environment)
- [[Phoenix Framework]] (web + real-time applications)
- [[Livebook]] (interactive experimentation with Rust NIFs)
- [[Nerves]] (embedded Elixir platform)
- [[Zigler]] (alternative approach using Zig)

---

## 🔗 Related Concepts / Notes

- [[Elixir]] (Functional, concurrent language)
- [[Rust]] (Safe, performant systems language)
- [[Zigler]] (Elixir-Zig NIF bridge)
- [[Nerves]] (Embedded Elixir)
- [[BEAM VM]] (Elixir runtime)
- [[Livebook]] (Elixir notebook interface)
- [[Phoenix Framework]] (Web + real-time UI)
- [[FFI]] (Foreign Function Interface)

---

## 📚 External Resources

- [https://github.com/rusterlium/rustler](https://github.com/rusterlium/rustler) — Official GitHub repository  
- [https://hexdocs.pm/rustler](https://hexdocs.pm/rustler) — Documentation on HexDocs  
- [https://docs.rs/rustler/latest/rustler/](https://docs.rs/rustler/latest/rustler/) — Rust crate docs  
- [Elixir Forum: Rustler Threads](https://elixirforum.com/tags/rustler) — Community discussion and guides  
- [Rust Language](https://www.rust-lang.org) — Official Rust site  

---

## 🏁 Summary

Rustler brings the performance and safety of [[Rust]] into the [[Elixir]] ecosystem, letting developers write native code without risking the stability of the [[BEAM VM]]. It’s an excellent choice for robotics engineers building real-time systems, sensor interfaces, or computational modules that need both reliability and speed. When combined with [[Phoenix Framework]], [[Nerves]], or [[Livebook]], Rustler enables a full-stack approach to high-performance robotics development.
