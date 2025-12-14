# ETS (Erlang Term Storage)

**ETS (Erlang Term Storage)** is an in-memory storage system built into the BEAM VM for storing and retrieving Erlang or Elixir terms efficiently. It provides high-performance concurrent access to data and is widely used for caching, state storage in GenServers, PubSub message tracking, and fast telemetry aggregation. ETS is a foundational tool in Elixir concurrency, LiveView state management, and distributed system design.

---

## 📚 Overview

ETS tables reside entirely in memory and support millions of concurrent reads and writes. Unlike GenServer state, ETS tables are accessible by multiple processes simultaneously without blocking, making them ideal for shared state in high-throughput applications. ETS supports multiple table types, including `:set`, `:ordered_set`, and `:bag`, each with different access and ordering semantics.

Key properties:
- In-memory storage
- Process-independent lifetime (can be owned by processes)
- Supports concurrent reads and writes
- Keys and values are arbitrary BEAM terms
- Efficient for high-performance caching or lookup

---

## 🧠 Core Concepts

- **Tables**: Named or anonymous storage containers.
- **Ownership**: Tables are owned by a process; when the owner dies, the table is destroyed.
- **Access Types**: `:public`, `:protected`, `:private`.
  - `:public`: anyone can read/write
  - `:protected`: anyone can read, only owner writes
  - `:private`: only owner can access
- **Table Types**:
  - `:set` → unique keys, unordered
  - `:ordered_set` → unique keys, sorted order
  - `:bag` → multiple entries per key, unordered
  - `:duplicate_bag` → multiple entries per key, allows duplicates
- **Term Storage**: Keys and values are arbitrary BEAM terms

---

## 🔧 How It Works

1. An ETS table is created via `:ets.new/2`.
2. Processes access the table using `:ets.insert`, `:ets.lookup`, `:ets.delete`, etc.
3. ETS manages concurrency internally with lock-free algorithms for many read-heavy workloads.
4. Owner process termination triggers automatic table cleanup.
5. For long-lived tables independent of process lifetime, use `:heir` tables or external wrappers.

---

## 🛠️ Strengths

- Extremely fast read and write operations
- Can be shared among many processes
- Supports large-scale in-memory data storage
- Flexible key/value types (any BEAM term)
- Essential for caching, PubSub, and telemetry
- Works seamlessly with GenServer and LiveView

---

## ⚠️ Weaknesses

- In-memory only (not persistent)
- Can consume large amounts of RAM if misused
- Owner-based lifetime can lead to accidental table deletion
- No automatic replication between nodes
- Must manage table naming and access carefully

---

## 📊 Comparison Chart

| Storage | In-Memory | Concurrency | Persistence | Notes |
|---------|-----------|------------|------------|------|
| **ETS** | Yes | High | No | BEAM-native, very fast |
| **Mnesia** | Yes/Yes + Disk | High | Yes | Distributed DB built on ETS |
| **GenServer State** | Yes | Low | No | Single-process access |
| **Redis** | Yes | High | Optional | External, network overhead |
| **Elixir Agent** | Yes | Low | No | Process-based wrapper |
| **Deterministic Map** | Yes | Low | No | Purely functional, single owner |

---

## 🧩 Use Cases

- Caching frequently accessed data
- Tracking PubSub subscriptions or message IDs
- Storing LiveView session state for performance
- Telemetry aggregation in RL systems
- Lookup tables in numeric computations
- Shared state for distributed supervision trees

---

## 🛠️ Developer Tools

- `:ets` module API
- `:mnesia` for distributed persistent tables
- Observer tool for inspecting ETS tables
- `:dbg` for debugging table access
- Telemetry for monitoring ETS usage

---

## 📑 Documentation and Support

- Elixir Docs: `:ets` module
- Erlang Reference Manual: ETS section
- OTP Design Principles
- Phoenix / LiveView guides (state caching)
- Community performance benchmarks

---

## 🔗 Related Concepts / Notes

- [[Elixir]]
- [[Erlang]]
- [[BEAM]]
- [[GenServer]]
- [[Agent]]
- [[LiveView]]
- [[PubSub]]
- [[Mnesia]]
- [[Telemetry]]
- [[Concurrency]]

---

## 📝 Summary

ETS provides a high-performance, in-memory key-value store for the BEAM VM, enabling shared state and caching across processes. Its concurrency-friendly design and flexibility make it essential for PubSub-heavy systems, LiveView applications, and RL infrastructures that require fast access to mutable state without sacrificing process isolation. Careful ownership and memory management are crucial to avoid runtime pitfalls.
