# WebAssembly (WASM)

WebAssembly (abbreviated Wasm) is a binary instruction format for a stack-based virtual machine designed as a portable compilation target for programming languages, enabling deployment on the web for client and server applications. It provides near-native performance, memory-safe sandboxed execution, and runs on all major browsers and numerous standalone runtimes. Since becoming a W3C Recommendation in December 2019, WebAssembly has evolved beyond the browser to edge computing, serverless platforms, and embedded systems.

## 🎯 Overview

WebAssembly is a low-level virtual instruction set architecture (ISA) that executes at near-native speed while maintaining security through sandboxing. Originally designed to enable high-performance applications in web browsers, Wasm has become a universal runtime for executing code from multiple languages across diverse platforms. The specification is language-agnostic and platform-independent, making it ideal for cross-platform applications, plugin systems, and containerless deployment. WebAssembly modules are validated before execution and run in a memory-safe environment, preventing data corruption and security breaches.

---

## ⚙️ Core Concepts

### Stack-Based Virtual Machine

WebAssembly uses a stack-based execution model:
- **Operands pushed onto stack:** Function arguments and intermediate values
- **Operations pop from stack:** Execute operations on values
- **Results pushed back:** Computation results returned to stack
- **LIFO execution:** Last-In, First-Out (stack discipline)
- **Structured control flow:** Blocks, loops, and conditionals

### Binary and Text Format

- **Binary format (.wasm):** Compact, efficient representation (~200 opcodes in MVP)
- **Text format (.wat):** Human-readable S-expression syntax for debugging
- **Size-efficient:** Significantly smaller than equivalent JavaScript
- **Load-time optimized:** Fast parsing and validation
- **Version-less:** Designed for backward compatibility

### Module System

A WebAssembly module contains:
- **Functions:** Imported or exported code
- **Memory:** Linear memory arrays (expandable)
- **Tables:** Typed arrays of references (e.g., function tables)
- **Globals:** Global variables (mutable or immutable)
- **Imports/Exports:** Interface definitions for host environment

### Memory Model

- **Linear memory:** Contiguous byte array, expandable in 64KB pages
- **Memory-safe:** Bounds checking on all accesses
- **Shared memory:** Optional support for multi-threading (WebAssembly 2.0+)
- **No direct system access:** All I/O through host environment
- **Sandboxed:** Cannot access memory outside allocated space

### Security Model

- **Validation before execution:** Type checking and verification
- **Memory isolation:** Each module has isolated linear memory
- **Capability-based security:** Can only call imported functions
- **No ambient authority:** Must explicitly import all system access
- **Sandboxed execution:** Cannot corrupt host environment

---

## 🌐 WebAssembly System Interface (WASI)

### Overview

WASI is a modular system interface specification for running WebAssembly outside the browser:
- **Standardized system calls:** File I/O, networking, clocks, random numbers
- **Capability-based security:** Explicit permissions required
- **Platform-independent:** Works across Windows, Linux, macOS, embedded systems
- **Influenced by POSIX and CloudABI:** But designed for security-first approach

### WASI Versions

**WASI Preview 1 (WASIp1):**
- Original specification with witx IDL
- Basic system facilities: filesystem, clock, environment variables, random
- Widely implemented across runtimes

**WASI Preview 2 (WASIp2):**
- Launched 2024, based on Component Model
- HTTP support (wasi-http), Key-Value stores (wasi-keyvalue)
- Uses WIT (WebAssembly Interface Types) for interface definitions
- Enables component composition across languages

**WASI Preview 3 (WASIp3):**
- Nearing completion as of 2025
- Async/await support across components
- Language-neutral asynchronous communication
- Enables sophisticated cross-component composition

### Component Model

- **WIT (WebAssembly Interface Types):** IDL for defining component interfaces
- **Composability:** Link WebAssembly components together
- **Language interoperability:** Components from different languages communicate
- **Type-safe interfaces:** Strongly typed function signatures

---

## 🆚 Comparison Chart

| Runtime/Platform | Type | Performance | Memory | Use Case | Language | AOT Support | JIT Support |
|------------------|------|-------------|--------|----------|----------|-------------|-------------|
| **Wasmtime** | Standalone | 85-90% native | 20-22 MB | Serverless, general purpose | Rust | ✅ Yes | ✅ Yes (Cranelift) |
| **Wasmer** | Standalone | 80-85% native | 18 MB | Blockchain, edge, desktop | Rust | ✅ Yes | ✅ Yes (multiple backends) |
| **WasmEdge** | Standalone | 85-90% native | 20 MB | AI/ML, edge computing | C++ | ✅ Yes | ✅ Yes (LLVM) |
| **WAMR** | Embedded | 75-80% native | 15 MB | IoT, embedded systems | C | ✅ Yes | ✅ Yes |
| **Wasm3** | Interpreter | 70-75% native | 10 MB | Resource-constrained devices | C | ❌ No | ❌ Interpreter only |
| **V8 (Node.js/Chrome)** | Browser/Server | 80-85% native | 30 MB | Web, server-side JS | C++ | ✅ Yes | ✅ Yes (TurboFan) |
| **SpiderMonkey (Firefox)** | Browser | 80-85% native | 25 MB | Web applications | C++ | ✅ Yes | ✅ Yes |
| **JavaScriptCore (Safari)** | Browser | 75-80% native | 25 MB | Web applications (Apple) | C++ | ✅ Yes | ✅ Yes |
| **Lucet** | Standalone (EOL) | 85-90% native | 22 MB | Serverless (Fastly edge) | Rust | ✅ Yes | ✅ Yes (Cranelift) |

### Key Distinctions

**Wasmtime vs Wasmer:**
- Wasmtime: Bytecode Alliance reference implementation, Cranelift JIT
- Wasmer: Multiple backend support (LLVM, Cranelift, Singlepass), faster startup
- Both written in Rust, similar performance
- Wasmer has more stable C API

**WasmEdge vs Others:**
- Focused on AI/ML and edge computing workloads
- CNCF sandbox project
- Extensions for TensorFlow, PyTorch
- Strong Kubernetes integration

**WAMR vs Full Runtimes:**
- Minimal footprint (15 MB vs 20+ MB)
- Written in C (security considerations)
- Designed for embedded systems and IoT
- Part of Bytecode Alliance

**Browser Runtimes vs Standalone:**
- Browser: Integrated with JavaScript, DOM access
- Standalone: WASI support, system-level operations
- Browser: Larger memory footprint
- Standalone: Better for server/edge workloads

---

## 💪 Strengths

- **Near-Native Performance:** 80-90% of native C/C++ speed typical
- **Memory Safety:** Sandboxed execution prevents corruption
- **Language Agnostic:** Compile from C, C++, Rust, Go, AssemblyScript, and 40+ languages
- **Portable Binary:** Same binary runs on any platform/architecture
- **Small Binary Size:** Compact format, faster downloads
- **Fast Startup:** Millisecond cold starts vs seconds for containers
- **Deterministic Execution:** Reproducible results across platforms
- **Security by Default:** Capability-based permissions model
- **Cross-Platform:** Write once, run anywhere (browsers, servers, edge, IoT)
- **No Runtime Installation:** Browsers have built-in support
- **Standards-Based:** W3C recommendation with active development

---

## ⚠️ Weaknesses

- **Limited System Access:** Requires explicit imports for all I/O
- **Debugging Complexity:** Limited debugging tools compared to JavaScript
- **Ecosystem Maturity:** Smaller library ecosystem than JavaScript/Python
- **DOM Access Complexity:** Must use JavaScript glue code for web APIs
- **No Direct Threading (MVP):** Threads added in WebAssembly 2.0
- **Build Toolchain Complexity:** Requires Emscripten, wasm-pack, or similar tools
- **Memory Allocation Overhead:** Linear memory model has limitations
- **Limited Exception Handling:** Basic compared to native languages
- **Package Management Immaturity:** No npm equivalent (yet)
- **Learning Curve:** Low-level concepts unfamiliar to web developers
- **Browser Security Policies:** CSP restrictions in some environments
- **Mobile Memory Limits:** ~300MB allocation limit on some mobile browsers

---

## 🎮 Use Cases

### Web Applications

- **Performance-Critical Code:** Video/audio processing, image manipulation
- **Games:** Unity, Unreal Engine, custom game engines
- **CAD/Design Tools:** AutoCAD, Figma-like applications
- **Scientific Visualization:** 3D rendering, data visualization
- **Cryptography:** Client-side encryption, blockchain wallets
- **Video Conferencing:** Real-time encoding/decoding

### Server-Side Applications

- **Serverless Functions:** AWS Lambda, Cloudflare Workers, Fastly Compute@Edge
- **Microservices:** Lightweight, fast-starting services
- **API Endpoints:** High-performance request handling
- **Edge Computing:** CDN edge processing, IoT gateways
- **Plugin Systems:** Safe, sandboxed extensions (e.g., Envoy, Nginx)
- **Database Extensions:** User-defined functions in databases

### Desktop Applications

- **Cross-Platform Apps:** Single binary for all platforms
- **Plugin Architectures:** VSCode extensions, Figma plugins
- **Emulators:** Run legacy software in browser/desktop
- **Media Applications:** Audio/video editing tools

### Embedded/IoT

- **Firmware Updates:** Safe, sandboxed code updates
- **Smart Contracts:** Blockchain execution environments
- **Industrial Control:** Safe, isolated control logic
- **IoT Devices:** Resource-constrained devices with WAMR/Wasm3

### Poor Fit Use Cases

- **Simple Web Pages:** Overhead not justified
- **Heavy DOM Manipulation:** Better with JavaScript frameworks
- **Node.js-Style I/O Heavy Apps:** Limited async I/O in current WASI
- **Applications Requiring Native Threads:** Limited threading support (improving)
- **Rapid Prototyping:** Toolchain overhead vs JavaScript

---

## 🔧 Compilation Toolchains

### Emscripten (C/C++)

- Most mature toolchain for C/C++ to WebAssembly
- Generates Wasm + JavaScript glue code + HTML
- LLVM-based: `clang → LLVM → Wasm`
- Provides libc implementation (musl-based)
- OpenGL → WebGL translation

### Rust

- First-class WebAssembly support via `wasm32-unknown-unknown` target
- `wasm-pack` for building and packaging
- `wasm-bindgen` for JavaScript interop
- Excellent for web and WASI targets

### AssemblyScript

- TypeScript-like syntax compiles to WebAssembly
- Familiar to JavaScript developers
- Lower performance than C/Rust but easier learning curve
- Good for web-first applications

### Go

- TinyGo for WebAssembly (smaller binaries than standard Go)
- WASI support for server-side applications
- Standard library limitations

### Other Languages

- **Python:** Pyodide (CPython compiled to Wasm)
- **C#/.NET:** Blazor WebAssembly
- **Java:** TeaVM, JWebAssembly
- **Kotlin:** Kotlin/Wasm
- **Swift:** SwiftWasm
- **Zig:** Native WebAssembly support

---

## 📊 WebAssembly Versions

### WebAssembly 1.0 (December 2019)

- W3C Recommendation
- MVP feature set
- Core instruction set (~200 opcodes)
- Single linear memory
- Basic import/export

### WebAssembly 2.0 (Draft, April 2022)

- SIMD instructions (v128 datatype)
- Multiple return values
- Bulk memory operations
- Reference types
- Sign-extension operators

### WebAssembly 3.0 (Released September 2025)

- **Garbage Collection (GC):** Managed memory for high-level languages
- **Typed references:** Rich type system for GC
- **Relaxed SIMD:** Faster vector operations with deterministic fallbacks
- **Deterministic execution profile:** Optional for reproducibility
- **Custom annotations:** Text format annotations for tooling
- **JavaScript string builtins:** Direct string manipulation in Wasm
- **Better language support:** Java, OCaml, Scala, Kotlin, Scheme, Dart

---

## 📚 Related Concepts/Notes

- [[WASI]] (WebAssembly System Interface)
- [[Emscripten]]
- [[LLVM]]
- [[Wasmtime]]
- [[Wasmer]]
- [[WasmEdge]]
- [[Component Model]]
- [[Virtual Machines]]
- [[JIT Compilation]]
- [[AOT Compilation]]
- [[Serverless Computing]]
- [[Edge Computing]]
- [[Sandboxing]]
- [[Memory Safety]]
- [[Bytecode]]
- [[Stack Machine]]
- [[ISA]] (Instruction Set Architecture)
- [[RISC-V]]
- [[Container Runtime]]
- [[Cloudflare Workers]]
- [[Fastly Compute]]

---

## 🔗 External Resources

### Official Specifications

- [WebAssembly Core Specification](https://webassembly.github.io/spec/core/)
- [WebAssembly 3.0 Announcement](https://webassembly.org/news/2025-09-17-wasm-3.0/)
- [WASI Specifications](https://wasi.dev/)
- [WebAssembly GitHub Organization](https://github.com/WebAssembly)

### Learning Resources

- [MDN WebAssembly Concepts](https://developer.mozilla.org/en-US/docs/WebAssembly/Guides/Concepts)
- [WebAssembly.org](https://webassembly.org/)
- [Wasm By Example](https://wasmbyexample.dev/)
- [The WebAssembly Book](https://github.com/bytecodealliance/wasmtime/blob/main/docs/WASI-overview.md)

### Runtimes

- [Wasmtime](https://wasmtime.dev/)
- [Wasmer](https://wasmer.io/)
- [WasmEdge](https://wasmedge.org/)
- [WAMR (WebAssembly Micro Runtime)](https://github.com/bytecodealliance/wasm-micro-runtime)

### Tooling

- [Emscripten](https://emscripten.org/)
- [wasm-pack (Rust)](https://rustwasm.github.io/wasm-pack/)
- [AssemblyScript](https://www.assemblyscript.org/)
- [TinyGo](https://tinygo.org/)

### Articles and Comparisons

- [Standardizing WASI (Mozilla)](https://hacks.mozilla.org/2019/03/standardizing-wasi-a-webassembly-system-interface/)
- [WebAssembly Runtimes Compared](https://blog.logrocket.com/webassembly-runtimes-compared/)
- [Wasm, WASI, Wagi Explanation](https://www.fermyon.com/blog/wasm-wasi-wagi)
- [Performance Benchmarks](https://00f.net/2023/01/04/webassembly-benchmark-2023/)

---

## 🏢 Industry Adoption

### Companies Using WebAssembly

- **Figma:** Design tool runs in browser via Wasm
- **AutoCAD:** CAD software ported to web with Wasm
- **Disney+:** Video streaming optimizations
- **Google Earth:** 3D rendering in browser
- **Cloudflare:** Workers platform uses V8 isolates + Wasm
- **Fastly:** Compute@Edge runs user code as Wasm
- **Adobe:** Photoshop on the web
- **Unity:** WebGL builds use WebAssembly
- **Shopify:** Checkout performance optimizations

### Platform Support

- **All major browsers:** Chrome, Firefox, Safari, Edge (>95% global coverage)
- **Node.js:** Native WebAssembly support
- **Deno:** First-class WASM support
- **Cloudflare Workers:** Wasm-based serverless platform
- **AWS Lambda:** Limited WASM support (via custom runtimes)
- **Kubernetes:** WASM as container alternative (experimental)

---

## 🎯 Famous Quote

> "If WASM+WASI existed in 2008, we wouldn't have needed to create Docker. That's how important it is. WebAssembly on the server is the future of computing."
> 
> — Solomon Hykes, co-founder of Docker (2019)

---

## 🔮 Future Directions

### Active Proposals

- **Exception Handling:** Structured exception propagation
- **Threads and Atomics:** Full multi-threading support
- **SIMD:** Extended SIMD operations
- **Tail Calls:** Optimization for functional languages
- **Module Linking:** Compose multiple Wasm modules
- **Interface Types:** Better language interoperability

### Ecosystem Development

- **WASI Preview 3:** Async/await, advanced I/O
- **Package Management:** Native package registry (emerging)
- **Debugging Tools:** Improved source maps, debuggers
- **Profiling:** Better performance analysis tools
- **Registry:** Standard package distribution (WAPM, etc.)
