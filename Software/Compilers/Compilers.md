# 🧠 Compilers

This note provides a **high-level overview** of compilers, what they do, the major types, how they compare, and links out to more detailed notes. This is meant to serve as a hub for understanding the ecosystem of **modern compiler technologies**, both general-purpose and specialized.

---

## 🚀 What is a Compiler?

A **compiler** is a program that translates source code written in a high-level programming language into **machine code**, **intermediate code**, or **another language**. The key goals are performance, correctness, portability, and toolchain integration.

### 🧱 Types of Compilers

- **Ahead-of-Time (AOT)**: Translates code before execution (e.g. GCC, Clang)
    
- **Just-in-Time ([[JIT]])**: Translates code during execution (e.g. JVM, V8)
    
- **Transpilers**: Convert code from one high-level language to another (e.g. TypeScript to JavaScript)
    

---

## 🔧 Major Compilers & Toolchains

### 🐧 GCC / G++

- **Maintainer**: GNU Project
    
- **Languages**: C, C++, Objective-C, Fortran, Ada, etc.
    
- **Strengths**: Mature, portable, highly optimized for many platforms
    

### 💡 [[Clang]] / [[LLVM]]

- **Maintainer**: LLVM Project
    
- **Languages**: C, C++, Objective-C, Swift, Rust (via rustc backend), etc.
    
- **Strengths**: Modular, faster compiles, modern error messages
    
- **Used In**: macOS toolchain, Chrome, Android
    

### 🔧 [[NVCC]]

- **Maintainer**: NVIDIA
    
- **Languages**: CUDA C/C++
    
- **Strengths**: Compiles CUDA kernels for GPU execution
    
- **Ties In With**: GCC, Clang, MSVC
    

### 📦 MSVC

- **Maintainer**: Microsoft
    
- **Languages**: C, C++
    
- **Strengths**: Deep Windows integration, Visual Studio support
    

### 🦀 rustc

- **Maintainer**: Rust Project
    
- **Languages**: Rust
    
- **Backend**: LLVM
    
- **Strengths**: Strict safety checks, concurrency
    

### 🐍 CPython / Cython

- **Maintainer**: Python Software Foundation / Cython team
    
- **Languages**: Python + C extensions
    
- **Notes**: CPython is an interpreter; Cython is a compiler
    

### ☕ Javac

- **Maintainer**: Oracle
    
- **Languages**: Java
    
- **Target**: Java Bytecode for the JVM

### 🧩 TypeScript Compiler (tsc)



---

## 📊 Comparison Table

| Compiler | Lang(s) | Backend | Notable For             | Open Source?  | Target OS |
| -------- | ------- | ------- | ----------------------- | ------------- | --------- |
| GCC/G++  | C/C++   | Native  | Stability, performance  | ✅             | All       |
| Clang    | C/C++   | LLVM    | Tooling, speed          | ✅             | All       |
| NVCC     | CUDA    | Custom  | GPU compilation         | ❌ (partially) | All       |
| MSVC     | C/C++   | Native  | Windows IDE integration | ❌             | Windows   |
| rustc    | Rust    | LLVM    | Memory safety           | ✅             | All       |
| Cython   | Python  | C       | Speedups for Python     | ✅             | All       |
| Javac    | Java    | JVM     | Java ecosystems         | ✅             | All       |

---

## ✅ Pros and ❌ Cons

|   |   |   |
|---|---|---|
|Compiler|✅ Pros|❌ Cons|
|GCC/G++|Wide support, fast, portable|Verbose errors, slower to compile than Clang|
|Clang|Modular, great tooling, fast|Slightly less optimized output in some cases|
|NVCC|GPU programming, CUDA SDK|Limited to NVIDIA hardware|
|MSVC|Visual Studio integration|Windows-only, closed source|
|rustc|Strong type safety, concurrency|Compile times can be high|
|Cython|Easy Python optimization|Complexity, build step|
|Javac|Mature JVM integration|Java-only|

---

## 🔍 Use Cases

|   |   |
|---|---|
|Use Case|Recommended Compiler(s)|
|High-performance native apps|GCC, Clang|
|Cross-platform GUI apps|Clang, MSVC|
|CUDA GPU compute|NVCC|
|Rust development|rustc|
|Fast Python extensions|Cython|
|Java enterprise software|Javac|

---

## 💬 Other Related Concepts

- [[JIT vs AOT]]
    
- [[Linkers and Loaders]]
    
- [[Build Systems (CMake, Meson)]]
    
- [[LLVM IR]]
