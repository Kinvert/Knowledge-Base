# Work Stealing Scheduler

A **Work Stealing Scheduler** is a dynamic load-balancing algorithm used in parallel computing systems to efficiently distribute tasks among multiple threads or processors. Rather than relying on a centralized queue, each worker thread maintains its own queue of tasks and “steals” work from others when it becomes idle. This model provides scalability, minimizes contention, and is widely used in high-performance computing, robotics, and real-time systems.

---

## ⚙️ Overview

Work stealing is a scheduling technique where **idle workers proactively obtain ("steal") tasks** from the queues of other workers that have excess tasks. This contrasts with centralized scheduling, where all tasks are managed in a shared queue.

It is particularly well-suited for **irregular or unpredictable workloads**, such as those found in robotics (e.g., concurrent sensor fusion, motion planning, and path optimization).

---

## 🧠 Core Concepts

- **Worker Threads:** Each thread has its own double-ended queue (deque) for local tasks.  
- **Local vs. Stolen Tasks:** Workers push and pop tasks from their own queue (LIFO), while thieves steal from the opposite end (FIFO).  
- **Load Balancing:** Idle workers dynamically balance the system by taking work from busier ones.  
- **Minimal Contention:** Local access to queues is thread-safe without locks in most implementations.  
- **Decentralization:** Removes single points of failure and improves scalability.

---

## ⚖️ Comparison Chart

| Feature | Work Stealing Scheduler | Work Sharing Scheduler | Centralized Queue | Task Pool Scheduler | Priority-Based Scheduler |
|----------|------------------------|------------------------|-------------------|--------------------|---------------------------|
| Load Distribution | Dynamic, decentralized | Push-based | Centralized | Shared pool | Priority-driven |
| Scalability | High | Moderate | Low | Moderate | Depends on policy |
| Overhead | Moderate | Low | High | Moderate | Varies |
| Ideal Use Case | Irregular workloads | Predictable workloads | Small task sets | General-purpose | Real-time or priority-sensitive |
| Example Systems | Intel TBB, Go runtime | POSIX Threads | Early thread pools | OpenMP | RTOS schedulers |

---

## 🛠️ How It Works

1. Each worker maintains its own deque of tasks.  
2. When a worker spawns new tasks, it pushes them to the bottom of its own deque.  
3. When it runs out of tasks, it selects another worker at random and attempts to steal a task from the top of that worker’s deque.  
4. This continues until all tasks are completed or no work remains.  
5. Lock-free or fine-grained synchronization ensures thread safety with minimal overhead.

---

## 🤖 Use Cases in Robotics

- **Parallel Path Planning:** Divide and conquer algorithms for multi-agent or multi-goal pathfinding.  
- **SLAM (Simultaneous Localization and Mapping):** Parallel map updates and feature extraction.  
- **Sensor Fusion:** Concurrent processing of different sensor data streams (e.g., LiDAR, IMU, camera).  
- **Motion Planning:** Running multiple candidate trajectories or optimization paths simultaneously.  
- **Task Scheduling:** Real-time coordination of computation between CPU cores or GPU kernels.

---

## ✅ Strengths

- Excellent scalability for multi-core processors  
- Reduces idle time for workers  
- No central bottleneck  
- Automatically adapts to workload changes  
- Efficient for recursive and divide-and-conquer algorithms (e.g., [[Fork-Join Model]])

---

## ❌ Weaknesses

- Slightly higher synchronization overhead than static schedulers  
- Random stealing may lead to suboptimal cache locality  
- Complex to debug due to non-deterministic task execution order  
- Not ideal for highly interdependent tasks  

---

## 🧩 Related Concepts

- [[Fork-Join Model]]  
- [[Thread Pool]]  
- [[Parallelism]]  
- [[Task Scheduling]]  
- [[Load Balancing]]  
- [[Real-Time Scheduling]]  

---

## 🧰 Compatible Frameworks and Implementations

- **Intel TBB (Threading Building Blocks)** — classic example of work stealing  
- **Cilk Plus** — early influential model using work stealing  
- **Go Runtime Scheduler** — goroutines managed with work stealing  
- **Java ForkJoinPool** — built-in concurrency utility using work stealing  
- **OpenMP** (with dynamic scheduling options)  
- **Rust Rayon** — data-parallelism library using work stealing  

---

## 📚 External Resources

- "Scheduling Multithreaded Computations by Work Stealing" — Blumofe & Leiserson (MIT)  
- Intel TBB documentation: `https://www.intel.com/content/www/us/en/developer/tools/oneapi/tbb.html`  
- Go Scheduler design: `https://golang.org/doc/`  
- Cilk paper and resources: `https://supertech.csail.mit.edu/cilk/`  
- "Structured Parallel Programming" (Leiserson, Schardl)

---

## 🗺️ Summary

Work Stealing Schedulers achieve near-optimal utilization of multicore processors through **decentralized and adaptive task distribution**. They are particularly powerful in robotics and real-time systems where workloads can change rapidly or are highly irregular. Despite some complexity, their balance
