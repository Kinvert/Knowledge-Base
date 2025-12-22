# BEAM VM

The BEAM (Bogdan/Björn's Erlang Abstract Machine) is the virtual machine at the core of the Erlang Open Telecom Platform (OTP). It is a highly optimized runtime system designed for building massively scalable, fault-tolerant, soft real-time systems with built-in support for concurrency, distribution, and hot code reloading. The BEAM executes bytecode compiled from various languages including Erlang, Elixir, Gleam, and LFE.

## 🎯 Overview

BEAM is a register-based virtual machine that provides the foundation for the Erlang ecosystem. Unlike most virtual machines that run within a single thread, BEAM creates one scheduler per CPU core (by default), each running on its own OS thread with dedicated run queues. This architecture enables true parallelism while maintaining the lightweight process model that makes BEAM-based systems capable of handling millions of concurrent processes. The VM manages scheduling, memory allocation, garbage collection, and inter-process communication, all designed for high reliability and fault tolerance in distributed systems.

---

## ⚙️ Core Concepts

### Lightweight Processes

BEAM processes are fundamentally different from OS processes or threads. Each BEAM process is an isolated unit of execution with:
- **Private memory space:** ~200-300 bytes initial overhead, ~1KB for heap and stack
- **Independent execution:** No shared memory between processes
- **Message-based communication:** All inter-process communication via message passing
- **Process ID (PID):** Unique identifier that won't repeat for months after process death
- **Isolated failure:** Process crashes don't affect other processes
- **Theoretical limit:** Up to 134 million processes on a single VM instance

### Actor Model

BEAM implements the actor model where:
- Each process is an independent actor
- Actors communicate exclusively through asynchronous message passing
- Each actor has a private mailbox for receiving messages
- No shared state between actors
- Messages are immutable and copied when sent between processes

### Preemptive Scheduling

BEAM uses preemptive scheduling based on **reductions** rather than time slices:
- **Reduction:** Unit of work (function call, arithmetic operation, message passing)
- **Reduction count:** Each process allocated a certain number of reductions (default ~4000)
- **Preemption:** Process suspended when reduction count exhausted
- **Fair scheduling:** All processes get equal CPU time regardless of workload
- **O(1) process switching:** Constant time complexity for context switching
- **One scheduler per core:** Default configuration creates one scheduler thread per CPU core
- **Dedicated run queues:** Each scheduler has its own run queue (since OTP R13B)
- **Work stealing:** Schedulers can migrate processes from busy to idle queues

### Per-Process Garbage Collection

Unlike "stop-the-world" garbage collectors, BEAM runs garbage collection:
- **Per-process basis:** Only affects individual processes
- **Never stops the VM:** Other processes continue executing
- **Triggers on need:** GC runs when process needs more memory
- **Microsecond pauses:** Individual process suspensions are very brief
- **Small memory footprint:** Processes typically allocate small amounts
- **Automatic cleanup:** Short-lived processes free all memory on termination

### Memory Model

- **Isolated heaps:** Each process has its own heap
- **Stack and heap together:** Allocated as one contiguous memory area
- **Message copying:** Messages deep-copied between processes (no sharing)
- **Immutable data:** All data structures are immutable
- **Structural sharing:** Efficient copying of immutable structures

### Hot Code Reloading

BEAM supports updating code without stopping the system:
- **Two code versions:** Old and new versions coexist during upgrade
- **Seamless migration:** Running processes switch to new code at function calls
- **No downtime:** System continues serving requests during upgrade
- **Essential for telecom:** Designed for 24/7 availability requirements
- **Development benefit:** Faster iteration without system restarts

---

## 🔧 Technical Architecture

### Register Machine

BEAM is a register-based VM (not stack-based):
- **X registers:** Temporary data and function arguments
- **Y registers:** Local variables that need to survive function calls
- **Named registers:** Instructions operate on named registers like variables
- **Efficient execution:** Fewer instructions than stack-based VMs

### Instruction Set

BEAM uses a two-level instruction set:
- **Generic instructions:** High-level portable instruction set
- **Specific instructions:** Optimized concrete instructions for execution
- **Loader transformation:** Generic bytecode transformed to specific at load time
- **Pattern recognition:** Common sequences replaced with optimized single opcodes
- **Dynamic optimization:** Bytecode rewritten for performance during loading

### Compilation Pipeline
```
Source Code (.erl/.ex/.gleam)
    ↓
Abstract Syntax Tree
    ↓
Expanded AST
    ↓
Core Erlang (intermediate representation)
    ↓
Kernel Erlang (pattern matching compiled)
    ↓
BEAM Assembly
    ↓
Generic BEAM Bytecode
    ↓ (loader)
Specific BEAM Bytecode (.beam files)
```

### Process Lifecycle

1. **Creation:** `spawn` allocates ~1KB memory, assigns PID
2. **Scheduling:** Process placed in run queue
3. **Execution:** Scheduler picks process, grants reductions
4. **Preemption:** Process suspended when reductions exhausted or blocked
5. **Message handling:** Process wakes on message arrival
6. **Termination:** All process memory immediately freed

---

## 🆚 Comparison Chart

| Virtual Machine | Process Model | Scheduling | GC Strategy | Memory Model | Hot Reload | Primary Language | Max Processes |
|-----------------|---------------|------------|-------------|--------------|------------|------------------|---------------|
| **BEAM** | Lightweight actors | Preemptive (per-core) | Per-process | Isolated heaps | ✅ Yes | Erlang/Elixir | Millions |
| **JVM** | OS threads | OS-based | Stop-the-world | Shared heap | ❌ No | Java | Thousands |
| **JVM (Virtual Threads)** | Green threads | JVM-managed | Stop-the-world | Shared heap | ❌ No | Java 21+ | Millions |
| **Python (CPython)** | OS threads | OS-based (GIL) | Stop-the-world | Shared heap | ❌ No | Python | Hundreds |
| **Node.js** | Event loop | Single-threaded | Incremental | Shared heap | Limited | JavaScript | One main + workers |
| **Go Runtime** | Goroutines | Work-stealing | Concurrent mark-sweep | Per-goroutine stack | ❌ No | Go | Millions |
| **CLR (.NET)** | OS threads | OS-based | Generational | Shared heap | Limited | C#/F# | Thousands |
| **V8 (JavaScript)** | Event loop | Single-threaded | Generational | Shared heap | ❌ No | JavaScript | One main thread |

### Key Distinctions

**BEAM vs JVM:**
- BEAM: Per-process GC (no stop-the-world), isolated memory, message passing
- JVM: Shared heap with locking, stop-the-world GC, traditional threading
- BEAM: Designed for distribution from ground up
- JVM: Distribution added later (RMI, etc.)

**BEAM vs Go:**
- Both use lightweight concurrency primitives (processes vs goroutines)
- BEAM: Message passing only, no shared memory
- Go: Channels plus shared memory with mutexes
- BEAM: Hot code reloading built-in
- Go: Requires restart for updates

**BEAM vs Node.js:**
- BEAM: True parallelism across cores
- Node.js: Single-threaded event loop (worker threads available)
- BEAM: Millions of processes
- Node.js: Async I/O with callbacks/promises

---

## 💪 Strengths

- **Massive Concurrency:** Handle millions of concurrent processes efficiently
- **True Parallelism:** One scheduler per core, no GIL or global locks
- **Fault Isolation:** Process failures don't cascade to other processes
- **Soft Real-Time:** Predictable latency due to preemptive scheduling
- **No Stop-the-World GC:** Per-process garbage collection
- **Hot Code Reloading:** Update code without downtime
- **Built-in Distribution:** Transparent clustering across network nodes
- **Battle-Tested:** Decades of production use in telecom (WhatsApp, Discord)
- **Fair Scheduling:** No process starvation
- **Low Latency:** Microsecond-level GC pauses
- **Excellent Error Handling:** "Let it crash" philosophy with supervisors
- **Proven Scalability:** Handles millions of concurrent connections per node

---

## ⚠️ Weaknesses

- **CPU-Bound Performance:** Slower than compiled languages for intensive computation
- **Single-Threaded Process Execution:** Each process runs on one scheduler at a time
- **Message Copying Overhead:** Deep copy on message send between processes
- **Limited CPU Optimization:** Not as optimized as V8 JIT or JVM HotSpot
- **Learning Curve:** Functional programming and actor model unfamiliar to OOP developers
- **Numeric Performance:** Not ideal for heavy mathematical computations
- **Ecosystem Size:** Smaller library ecosystem than JVM or npm
- **Startup Time:** Slower cold start than native executables
- **Memory Overhead:** Each process has memory overhead (though minimal)
- **Type System (Erlang):** Dynamic typing can lead to runtime errors
- **Hot Reload Complexity:** Type safety not guaranteed during code upgrades

---

## 🎮 Use Cases

### Ideal Applications

- **Telecommunications Systems:** Phone switches, messaging platforms (original use case)
- **Chat Applications:** WhatsApp, Discord (millions of concurrent users)
- **Real-Time Web Applications:** Phoenix LiveView, multiplayer games
- **IoT Platforms:** Handling thousands of connected devices
- **Financial Systems:** Trading platforms, payment processing (soft real-time)
- **Backend APIs:** High-concurrency REST/GraphQL servers
- **Message Brokers:** RabbitMQ (written in Erlang)
- **Distributed Databases:** Riak, CouchDB
- **Stream Processing:** Event-driven systems, data pipelines
- **Monitoring Systems:** Telemetry collection and aggregation
- **Embedded Systems:** Networked devices, industrial control

### Poor Fit Applications

- **CPU-Intensive Computation:** Heavy numeric processing, machine learning training
- **Single-User Desktop Applications:** Overhead not justified
- **High-Performance Computing:** Scientific computing, simulations
- **Game Engines:** Real-time 3D graphics rendering
- **Video Encoding:** CPU-bound media processing
- **Batch Processing:** Large data transformations (better with Spark, etc.)

---

## 🌐 Languages Running on BEAM

### Production-Ready Languages

- **Erlang:** Original language, functional, dynamically typed
- **Elixir:** Modern syntax (Ruby-inspired), powerful macros, excellent tooling
- **Gleam:** Statically typed, compiles to Erlang or JavaScript, Rust-like syntax
- **LFE (Lisp Flavoured Erlang):** Lisp dialect, 100% compatible with Core Erlang
- **Alpaca:** ML-inspired statically typed language (less active)
- **Caramel:** OCaml-based statically typed language for BEAM

### Experimental/Niche Languages

- **Clojerl:** Clojure implementation on BEAM
- **Hamler:** Haskell-style strongly typed language
- **Purerl:** PureScript backend for BEAM
- **Efene:** Python/JavaScript-like syntax
- **Luerl:** Lua implementation in Erlang
- **OTPCL:** Tcl-like scripting language
- **Plus 40+ others in various stages of development**

### Cross-Language Interoperability

All BEAM languages compile to `.beam` bytecode and can:
- Call each other's functions seamlessly
- Share OTP libraries and behaviors
- Use the same package ecosystem (Hex)
- Run in the same VM instance

---

## 📊 Performance Characteristics

### Process Creation/Destruction

- **Creation time:** Microseconds
- **Memory overhead:** ~1-2KB per process
- **Context switch:** O(1) constant time
- **Destruction:** Instant memory reclamation

### Scheduling Fairness

- **Reduction-based:** ~4000 reductions per time slice
- **Preemptive:** No process can monopolize CPU
- **Work stealing:** Load balancing across schedulers
- **Compaction mode:** Consolidates work on fewer schedulers when idle

### Message Passing

- **Latency:** Microseconds for local processes
- **Throughput:** Millions of messages per second
- **Network transparency:** Same API for local and remote processes
- **Copying overhead:** Deep copy on send (immutability guarantee)

---

## 🛠️ Runtime Configuration

### Scheduler Control

- `+S N:M` - Set N schedulers online, M max schedulers
- `+SDcpu N` - Set N dirty CPU schedulers
- `+SDio N` - Set N dirty I/O schedulers
- Default: One scheduler per logical core

### Process Limits

- `+P Max` - Max number of processes (default 262,144)
- Can be set up to 134,217,727 processes

### Memory Management

- `+MBas aobf` - Set memory allocation strategy
- `+hms Size` - Set minimum heap size
- `+hmbs Size` - Set minimum binary virtual heap size

---

## 📚 Related Concepts/Notes

- [[Erlang]]
- [[Elixir]]
- [[Gleam]]
- [[LFE]] (Lisp Flavoured Erlang)
- [[OTP]] (Open Telecom Platform)
- [[Actor Model]]
- [[Concurrency]]
- [[Parallelism]]
- [[Message Passing]]
- [[Preemptive Scheduling]]
- [[Garbage Collection]]
- [[Process Isolation]]
- [[Fault Tolerance]]
- [[Distributed Systems]]
- [[Hot Code Reloading]]
- [[GenServer]]
- [[Supervisor]]
- [[Phoenix Framework]]
- [[WhatsApp]]
- [[Discord]]
- [[RabbitMQ]]
- [[Functional Programming]]
- [[Immutability]]
- [[Green Threads]]
- [[Virtual Machines]]
- [[BEAM]]

---

## 🔗 External Resources

### Official Documentation

- [Erlang/OTP Documentation](https://www.erlang.org/doc/)
- [The BEAM Book](https://blog.stenmans.org/theBeamBook/)
- [Brief Introduction to BEAM](https://www.erlang.org/blog/a-brief-beam-primer/)
- [Erlang FAQ - Implementations](https://www.erlang.org/faq/implementations.html)

### Deep Dives

- [BEAM Instruction Set (Historical)](https://www.cs-lab.org/historical_beam_instruction_set.html)
- [BEAM Wisdoms](https://github.com/kvakvs/beam-wisdoms)
- [The BEAM Book GitHub](https://github.com/happi/theBeamBook)
- [Erlang BEAM Links](https://github.com/AlexanderKaraberov/Erlang-BEAM-Links)

### Tutorials and Articles

- [Elixir and The Beam: How Concurrency Really Works](https://medium.com/flatiron-labs/elixir-and-the-beam-how-concurrency-really-works-3cc151cddd61)
- [Deep Diving Into the Erlang Scheduler](https://blog.appsignal.com/2024/04/23/deep-diving-into-the-erlang-scheduler.html)
- [BEAM and JVM Virtual Machines Comparison](https://www.erlang-solutions.com/blog/beam-jvm-virtual-machines-comparing-and-contrasting/)
- [The BEAM: Erlang's Virtual Machine](https://www.erlang-solutions.com/blog/the-beam-erlangs-virtual-machine/)

### Talks and Presentations

- [Unique Resiliency of the BEAM (InfoQ)](https://www.infoq.com/presentations/resilience-beam-erlang-otp/)
- [30 Years On and In the Beam](https://codebeamamerica.com/keynotes/30-years-on-and-in-the-beam/)
- [The Evolution of the BEAM Virtual Machine](https://elixirmerge.com/p/the-evolution-of-the-beam-virtual-machine-and-mastering-concurrency)

---

## 🗂️ Historical Context

### Origins

- **Creator:** Originally JAM (Joe's Abstract Machine) by Joe Armstrong and Mike Williams
- **BEAM development:** Bogumil "Bogdan" Hausman (original), Björn Gustavsson (current maintainer)
- **Purpose:** Built for Ericsson telecom infrastructure
- **Open Source:** Released in 1998
- **Battle-tested:** 30+ years of production use in critical systems

### Evolution

- **JAM (1987):** First Erlang VM, stack-based
- **BEAM (1996):** Register-based VM, replaced JAM
- **OTP R11B (2006):** First SMP support (1-1024 schedulers, single run queue)
- **OTP R12B (2007):** Multi-core support (one thread per core, shared run queue)
- **OTP R13B (2009):** Dedicated run queues per scheduler (current architecture)
- **Modern BEAM:** Continuous optimization, JIT compiler integration (OTP 24+)

---

## 🏗️ System Design Philosophy

### "Let It Crash"

BEAM embraces failure as normal:
- Processes are cheap to create and destroy
- Supervisors automatically restart failed processes
- Isolation prevents cascading failures
- Error handling through process hierarchy, not try/catch everywhere

### Immutability and No Shared State

- All data structures immutable
- No locks needed for data access
- Message passing is only communication mechanism
- Prevents entire classes of concurrency bugs

### Distribution by Design

- Same code runs locally or distributed
- Transparent network communication
- Built-in node discovery and clustering
- Network partitions handled gracefully
