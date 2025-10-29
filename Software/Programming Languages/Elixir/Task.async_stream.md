# Task.async_stream

`Task.async_stream` is an Elixir function that enables concurrent processing of enumerable data while maintaining control over parallelism, fault tolerance, and result ordering. It’s a high-level abstraction built on top of lightweight Elixir processes, ideal for handling I/O-bound tasks or data processing pipelines in robotics systems that rely on concurrency for performance and responsiveness.

---

## ⚙️ Overview

`Task.async_stream` runs a function concurrently across elements of an enumerable. It spawns supervised tasks that execute the given function, yielding results as they complete. Unlike `Task.async` and `Task.await`, it manages multiple tasks efficiently, controlling concurrency via the `:max_concurrency` option.

Example usage:  
`Task.async_stream(list, fn x -> heavy_compute(x) end, max_concurrency: 4) |> Enum.to_list()`

This makes it well-suited for robotics workloads like batch sensor data processing, network communications, and running multiple simulations in parallel without overloading system resources.

---

## 🧠 Core Concepts

- **Concurrency Model:** Built on lightweight BEAM (Erlang VM) processes.  
- **Enumerable Interface:** Accepts any enumerable (`List`, `Stream`, etc.).  
- **Supervision:** Tasks run under the current process; failures are isolated.  
- **Backpressure Control:** Limits the number of concurrent tasks via `:max_concurrency`.  
- **Timeout Handling:** Each task can be individually timed out using `:timeout`.  
- **Result Ordering:** By default, results are emitted in the same order as the input.

---

## 🧩 How It Works

1. The function iterates over an enumerable.  
2. Each element is passed to a worker task.  
3. The system manages concurrent task spawning and result collection.  
4. Results are lazily streamed, allowing for efficient resource use.  
5. When used with `Enum.to_list/1`, it gathers all results in order.

Example:  
`Task.async_stream(1..10, fn i -> compute_path(i) end, max_concurrency: 3, timeout: 5000) |> Enum.to_list()`

This executes up to three computations simultaneously while enforcing a 5-second timeout per task.

---

## 📊 Comparison Chart

| Function | Concurrency | Ordering | Fault Handling | Backpressure | Typical Use |
|-----------|--------------|-----------|----------------|---------------|--------------|
| **Task.async_stream** | Parallel | Ordered | Isolated per task | Yes | Large data processing, I/O-bound tasks |
| **Task.async** | Manual | Manual | Manual | No | Ad-hoc concurrency |
| **Task.await_many** | Parallel | Manual | Crashes on errors | No | Small fixed task sets |
| **Flow** | Distributed | Ordered or unordered | Supervised | Yes | Dataflow pipelines, large-scale |
| **GenStage** | Distributed | Controlled | Supervised | Yes | Reactive systems and pipelines |
| **Stream.map** | Sequential | Ordered | Immediate | No | Simple, synchronous processing |

---

## 🛠️ Use Cases

- Concurrent sensor data analysis across multiple streams.  
- Running perception modules in parallel (e.g., LiDAR, camera, and radar).  
- Parallel simulation of different path planning strategies.  
- Batch cloud API calls in fleet management systems.  
- Running concurrent diagnostics or hardware health checks.

---

## ✅ Strengths

- High-level abstraction for safe concurrency.  
- Easy to control concurrency with `:max_concurrency`.  
- Maintains result order predictably.  
- Integrates seamlessly with Elixir’s `Stream` and `Enum` APIs.  
- Great for both CPU- and I/O-bound tasks (with tuned concurrency).

---

## ❌ Weaknesses

- Each task carries overhead of a separate process.  
- Not suitable for very short or trivial tasks due to process spawn cost.  
- Limited to single-node concurrency (use [[Flow]] or [[GenStage]] for distributed).  
- Exceptions within a task only propagate if explicitly handled.

---

## 🧰 Compatible Tools

- [[Flow]] – For scalable, distributed dataflow parallelism.  
- [[GenStage]] – For building demand-driven concurrent systems.  
- [[Task.Supervisor]] – For spawning and supervising task groups.  
- [[Enum]] – For collecting or transforming `async_stream` results.  
- [[Logger]] – For monitoring asynchronous task behavior.  
- [[Telemetry]] – For instrumenting task metrics in real-time systems.

---

## 🧩 Variants and Related Functions

- **Task.async/1:** Spawns a single asynchronous task.  
- **Task.await/1:** Blocks until a task returns a result.  
- **Task.await_many/1:** Waits for multiple async tasks to complete.  
- **Task.yield/1:** Non-blocking retrieval of results with timeout.  
- **Task.Supervisor.async_stream/5:** Like `async_stream`, but under a supervisor for fault isolation.

---

## 🔗 Related Concepts

- [[Concurrency]] (Parallel execution model)  
- [[Elixir]] (Programming Language)  
- [[Erlang VM]] (BEAM)  
- [[Actor Model]] (Concurrency paradigm)  
- [[GenStage]] (Backpressure-driven concurrency)  
- [[Flow]] (Parallel data processing)  
- [[Task]] (Core concurrency module)  
- [[Stream]] (Lazy enumeration in Elixir)

---

## 📚 External Resources

- Official Docs: https://hexdocs.pm/elixir/Task.html#async_stream/3  
- Elixir Guides: https://elixir-lang.org/getting-started/mix-otp/task-and-genserver.html  
- Practical Concurrency in Elixir: https://elixirschool.com/en/lessons/advanced/otp-concurrency/  
- BEAM Architecture Overview: https://www.erlang.org/doc/system_architecture_intro/sys_arch_intro.html  

---

## 🧭 Summary

`Task.async_stream` is an elegant and powerful tool for achieving concurrent execution in Elixir applications. It abstracts away low-level process management while offering fine-grained control over concurrency and resource utilization. For robotics, it’s an efficient way to parallelize perception, computation, and communication tasks while maintaining safety, order, and responsiveness.

---
