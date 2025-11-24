# shmget
`shmget` is a System V IPC (Inter-Process Communication) call used to create or access shared memory segments in Unix-like operating systems. It enables multiple processes to map and manipulate the same memory region, providing extremely fast data exchange without kernel-mediated copying. For performance-sensitive systems in C and Zig, `shmget` is a foundational primitive for zero-copy IPC and shared-state architectures.

---

## ⚙️ Overview
`shmget` allocates or retrieves a shared memory segment identified by a key. It is typically paired with `shmat`, `shmdt`, and `shmctl` to fully manage the lifecycle of shared memory.

Core behavior:
- Creates or accesses a System V shared memory segment
- Returns an integer shared memory ID
- Segment then attached into process address space using `shmat`
- Requires explicit cleanup

This model is widely used in:
- Low-latency systems
- Trading engines
- Real-time robotics
- High-performance computing
- Multi-process RL environments

---

## 🧠 Core Concepts
- System V IPC: Legacy but powerful IPC mechanism
- Shared Memory Segment: Kernel-managed memory block
- Key: Integer identifier for segment lookup
- Permissions: POSIX-style flags (e.g., 0666)
- shmid: Returned identifier for segment
- Attachment: Mapping memory via `shmat`
- Detachment: Releasing mapping via `shmdt`

---

## 🔬 How It Works (Low-Level View)
Typical lifecycle:
1. Process calls `shmget(key, size, flags)`
2. Kernel allocates memory if not existing
3. Returns shared memory ID (`shmid`)
4. Process calls `shmat(shmid, addr, flags)`
5. Memory becomes part of virtual address space
6. Processes read/write to shared memory
7. Cleanup via `shmdt` and `shmctl(IPC_RMID)`

Internally, the kernel maps a single physical memory region into multiple process address spaces.

---

## 🧵 shmget in C
In C, `shmget` is declared in `<sys/shm.h>` and is typically used with raw pointers.

Example conceptually:
- Allocate shared segment: `shmget(key, size, IPC_CREAT | 0666)`
- Attach: `shmat(shmid, NULL, 0)`
- Cast to desired structure type

This low-level model offers:
- Zero abstraction overhead
- Full control
- Maximum performance
- High foot-gun potential

---

## 🧩 shmget in Zig
Zig does not expose System V IPC natively, but supports direct syscall interaction and C interop.

Approaches:
- Use Zig's `@cImport` to include `<sys/shm.h>`
- Call `shmget` and `shmat` directly through libc
- Wrap calls in safe Zig abstractions
- Use `std.os.linux.shmget` where available

Zig advantages:
- Stronger type safety wrapping
- Explicit error handling
- Safer pointer management
- Easier abstraction layers over IPC

Zig often pairs `shmget` with:
- `@ptrCast`
- Custom struct serialization
- Memory safety policies

---

## 📊 Comparison Chart — Shared Memory Mechanisms

| Mechanism | API Type | Performance | Safety | Typical Use | Language Fit |
|----------|----------|-------------|--------|--------------|--------------|
| shmget (System V) | Legacy IPC | Very High | Low | C native low-level IPC | Excellent in C, workable in Zig |
| POSIX shm_open | Modern IPC | Very High | Medium | Safer shared memory | Better portability |
| mmap | POSIX memory | High | Medium | File-backed IPC | Good in Zig and C |
| gRPC | Message-based | Low | High | Distributed systems | Best for microservices |
| Pipes | Kernel IPC | Medium | Safe | Simple process I/O | General |
| Sockets | Network IPC | Medium | High | Remote communication | Cross-platform |

System V `shmget` remains the fastest option for local IPC but requires careful memory discipline.

---

## 🎯 Use Cases
- Reinforcement Learning environment state sharing
- Real-time robotics sensor fusion
- Multi-process simulation engines
- High-frequency trading signal transfer
- Inter-process logging buffers
- Shared ring buffers for IPC

---

## ✅ Strengths
- Extremely fast access
- Zero-copy data transfer
- Kernel-managed synchronization
- Widely supported in Unix systems
- Mature and battle-tested

---

## ❌ Weaknesses
- Manual cleanup required
- Prone to memory leaks
- Poor portability to Windows
- Debugging complexity
- Requires synchronization primitives (mutex/semaphore)

---

## 🔑 Key Features
- Kernel-level memory sharing
- Process-independent lifetime
- Integer-based segment keys
- POSIX-style permission control
- Dynamic attachment and detachment

---

## 🛠 Developer Tools
- ipcs command for listing segments
- ipcrm for removing segments
- strace/ltrace for syscall tracing
- Valgrind for memory diagnostics
- GDB for attached memory debugging

---

## 📚 Documentation and Support
- Linux man pages: `shmget`, `shmat`, `shmdt`
- POSIX IPC documentation
- Zig libc interoperability guides
- Kernel IPC subsystem docs

---

## 🧪 Capabilities
- Multi-process memory coherence
- Real-time shared buffers
- Persistent IPC channels
- High-throughput data channels
- Deterministic performance paths

---

## 🧭 Summary
`shmget` remains one of the most powerful low-level IPC primitives available on Unix systems. In C, it offers raw, uncompromising control over shared memory, making it ideal for real-time and performance-critical workloads. In Zig, it becomes safer and more structured through explicit error handling and type-aware abstractions, enabling modern systems to harness shared memory without sacrificing stability.

---

## 🔗 Related Concepts / Notes
- [[Inter-Process Communication]]
- [[System V IPC]]
- [[POSIX Shared Memory]]
- [[Zero-Copy]]
- [[Memory Mapping]]
- [[Linux Kernel]]
- [[Concurrency Models]]
- [[Real-Time Systems]]

---

## 🌐 External Resources
- Linux man page: shmget
- POSIX Shared Memory specification
- Zig libc interop documentation
- Advanced IPC design guides
- Kernel shared memory internals

---

## 🚀 Compatible Items
- C
- Zig
- Rust (via libc bindings)
- Linux Kernel
- FreeBSD
- Embedded Linux

---

## 🧬 Variants
- POSIX shm_open
- mmap-based shared memory
- SysV Shared Memory (shmget)
- Named shared memory regions

---

## 🔧 Hardware Requirements
- Shared physical RAM
- Kernel support for System V IPC
- Multi-core systems benefit most
