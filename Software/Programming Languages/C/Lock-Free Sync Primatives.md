# Lock-Free Sync Primitives
Lock-free synchronization primitives are concurrency building blocks that allow multiple threads to coordinate without using traditional blocking locks. They rely on atomic operations and careful memory ordering to guarantee progress properties while often improving throughput and reducing priority inversion in highly concurrent systems. This note focuses on principles, common primitives, trade-offs, and practical guidance for systems and RL engineers building high-performance, low-latency software.

---

## ⚙️ Overview
Lock-free primitives aim to provide safe concurrent access to shared data while avoiding global blocking. They are widely used in OS kernels, low-latency servers, real-time systems, and high-frequency RL simulators where pause/stop from locks causes unacceptable jitter. Typical guarantees include *lock-freedom* (some thread makes progress) and *wait-freedom* (every thread makes progress in bounded steps).

---

## 🧠 Core Concepts
- **Atomic operations:** Hardware-supported ops such as `CAS` (compare-and-swap), `FAA` (fetch-and-add), `LOAD/STORE` with atomic semantics (`std::atomic`).
- **Memory ordering:** `seq_cst`, `acq_rel`, `acquire`, `release`, and relaxed orderings — crucial for correctness and performance.
- **Progress guarantees:**
  - *Wait-free* — every operation completes in finite steps.
  - *Lock-free* — system-wide progress guaranteed (at least one thread).
  - *Obstruction-free* — progress if a thread runs in isolation (weaker).
- **ABA problem:** A location changes A→B→A causing CAS to succeed erroneously; mitigations include tags/counters, `double-word CAS` or hazard pointers.
- **Memory reclamation:** Safe reclamation strategies (hazard pointers, epoch-based reclamation/RCU) to avoid use-after-free with concurrent access.
- **Non-blocking algorithms:** Queues, stacks, hash maps, linked lists implemented without locks using atomic primitives.

---

## 🔬 Comparison Chart
| Primitive / Technique | Blocking? | Typical Ops | Progress Guarantee | Use Cases | Complexity |
|---|---:|---|---|---|---:|
| Lock-free queue (Michael & Scott) | No | `CAS` on pointers | Lock-free | High-throughput messaging | Medium–High |
| Wait-free algorithms | No | Atomics + helping | Wait-free | Real-time systems | Very High |
| Mutex / `std::mutex` | Yes (blocking) | lock/unlock | Blocking | General-purpose | Low |
| Spinlock | Busy-waits (blocking) | test-and-set | Blocking | Short critical regions | Low |
| RCU (Read-Copy-Update) | Readers non-blocking | copy + grace periods | Reader-wait-free | Read-heavy workloads | Medium |
| Transactional Memory (HTM/STM) | Best-effort non-blocking | begin/commit/abort | Varies | Complex concurrent ops | High |
| Hazard pointers | Memory mgmt technique | pointer protection | N/A (reclamation) | Lock-free containers | Medium |

---

## 🧪 Common Primitives & Patterns
- **Compare-and-swap (CAS):** Foundation for most lock-free structures.  
- **Fetch-and-add (FAA):** Useful for counters and indices.  
- **Atomic pointers / `std::atomic<T*>`:** For building lock-free linked data structures.  
- **Tagged pointers / version counters:** Used to break ABA.  
- **Hazard Pointers:** Safe pointer protection for reclamation.  
- **Epoch-based reclamation / EBR / QSBR / RCU:** Fast reclamation for read-heavy workloads.  
- **Helping / cooperative algorithms:** Threads help complete each other's operations to ensure progress (common in wait-free designs).  

---

## ✅ Strengths
- Lower latency and jitter under contention (no blocking handoffs).  
- Avoids deadlocks and reduces priority inversion.  
- Often higher throughput for high-concurrency workloads.  
- Good fit for real-time, low-latency and RL simulation loops.

---

## ❌ Weaknesses
- Harder to design and reason about (subtle memory-order bugs).  
- Implementation complexity is high; debugging is non-trivial.  
- Memory reclamation is a major pain point—incorrect reclamation causes UB.  
- May perform worse than well-tuned locks for low contention or large critical sections.  
- Portability caveats with weaker memory models (some architectures require special handling).

---

## 🧩 Use Cases
- High-performance queues (work-stealing, task schedulers).  
- Lock-free stacks and free lists for allocators.  
- Real-time systems and low-latency networking stacks.  
- RL simulators requiring deterministic, low-jitter stepping.  
- OS kernels and hypervisors where blocking is unacceptable.

---

## 🔧 Developer Tools & Libraries
- `std::atomic`, `<atomic>` — C++ standard facilities.  
- `std::atomic_ref` (C++20) — atomic access to non-atomic storage.  
- Concurrency libraries: Folly (Facebook), libcds, Boost.Lockfree.  
- Memory reclamation: `libcds` reclamation schemes, userspace RCU implementations.  
- Sanitizers: ThreadSanitizer (TSan) helps find some races but can miss atomic ordering issues.  
- Formal tools: Model checkers and formal verification (TLA+, CBMC for some cases) for critical algorithms.

---

## 🧾 Strengths/Pros vs Weaknesses/Cons
- **Pros:** low-latency, avoids deadlocks, scalable under high contention, fits real-time constraints.  
- **Cons:** high design + verification cost, tricky memory reclamation, sometimes worse under low contention.

---

## 🔍 Variants & Related Algorithms
- **Lock-free linked lists / hash tables** — often use `CAS` loops and careful versioning.  
- **Lock-free ring buffers** — single-producer-single-consumer (SPSC) vs multi-producer-multi-consumer (MPMC) variants.  
- **Wait-free queues** — stronger progress guarantees via complex helping schemes.  
- **RCU** — read-mostly synchronization with deferred reclamation.  
- **Epoch GC / EBR** — simple, efficient reclamation when readers are frequent.

---

## 📚 Documentation & Further Reading
- Official C++ atomic and memory model documents (`<atomic>`, memory_order).  
- Library docs: Folly's concurrency primitives, Boost.Lockfree.  
- Academic papers: Michael & Scott queue, hazard pointers (Maged Michael), epoch reclamation, RCU.  
- Practical blog posts and guides on the ABA problem and memory ordering.

---

## 🧾 Examples (notes only — code should be in a follow-up file)
- `std::atomic<int>` counters with `fetch_add` for lock-free indices.  
- Michael & Scott queue as MPMC pattern for task queues.  
- RCU for read-heavy shared tables with occasional writers.

---

## 🔗 Related Concepts / Notes
- [[Concurrency]]  
- [[Memory Model]]  
- [[CAS]] (Compare-and-Swap)  
- [[ABA Problem]]  
- [[Hazard Pointers]]  
- [[Epoch-Based Reclamation]]  
- [[RCU]] (Read-Copy-Update)  
- [[Locking vs Non-Blocking]]  
- [[Wait-free Algorithms]]  
- [[Spinlocks]]  
- [[Lock-Free Data Structures]]  

---

## ♻️ Compatible Items / Where to Use
- Low-latency RL environments and simulators  
- High-throughput messaging layers in microservices  
- Custom allocators for concurrent workloads  
- Real-time data collectors / telemetry pipelines

---

## 🧭 Key Highlights
- Use lock-free primitives when low latency and progress under contention matter.  
- Always pair lock-free logic with a robust reclamation strategy (hazard pointers / epoch).  
- Profile! For low contention, locks may be simpler and faster.  
- Mind the memory model — `acquire`/`release` orderings often suffice; `seq_cst` is the safest but costly.

---
