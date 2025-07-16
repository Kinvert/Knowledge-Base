# CPU Cache

A **CPU cache** is a small, high-speed memory layer located directly on or near the processor cores. It is used to store frequently accessed data and instructions, drastically reducing the time needed to fetch this information from main memory (RAM). Effective cache utilization is critical for performance in engineering tasks like simulation, compilation, and real-time robotics systems.

Modern CPUs include multiple levels of cache (L1, L2, L3), each offering different trade-offs in speed, size, and latency.

---

## 🧠 Overview

- **L1 Cache**: Fastest and smallest, typically split into separate instruction and data caches  
- **L2 Cache**: Larger than L1, shared per core or small cluster of cores  
- **L3 Cache**: Shared among all cores; significantly larger but slower  
- **L4 Cache** (less common): Found in some server or HEDT chips as an additional shared layer

Cache memory is implemented using **SRAM**, and operates at much higher speeds than DRAM (RAM). It serves as the primary buffer between the core and main memory.

---

## 🧪 Use Cases in Engineering

- Fast access to tight loops and frequently reused variables in simulation code  
- Efficient handling of matrix operations and tensor algebra  
- Reducing latency in real-time control systems (e.g., motor feedback loops)  
- Accelerating build systems through optimized instruction caching  
- Parallel reinforcement learning simulations where inter-core data sharing happens

---

## 📊 Comparison Table

| Cache Level | Typical Size (per core or CPU) | Speed        | Latency (cycles) | Shared Among      |
|-------------|----------------------------------|--------------|------------------|--------------------|
| L1          | 32–128 KB (I/D split)           | Fastest      | 1–5              | Per core           |
| L2          | 256 KB – 2 MB                   | Very Fast    | 10–20            | Per core / small cluster |
| L3          | 8–128 MB                        | Fast         | 30–50+           | All cores on die   |
| L4          | 64–512 MB (rare)                | Moderate     | 100+             | Entire CPU / SoC   |

---

## 🔧 Key Features

- **Low Latency**: Data can be accessed in nanoseconds  
- **Spatial & Temporal Locality**: Cache stores data likely to be reused soon  
- **Inclusive / Exclusive Policies**: Some CPUs store redundant data in lower caches  
- **Write-through vs. Write-back**: Dictates how changes propagate to main memory

---

## ✅ Pros

- Dramatically improves CPU performance  
- Reduces memory bottlenecks in tight loops  
- Minimizes context-switching costs and idle stalls  
- Enables efficient multithreading with shared cache levels

---

## ❌ Cons

- Limited size means poor cache design can cause thrashing  
- Shared L3 may become a contention point in multi-core CPUs  
- Expensive to manufacture (SRAM takes more die area than DRAM)  
- Hardware-controlled, not directly tunable by developers

---

## 🔗 Related Concepts

- [[CPUs]]  
- [[CPU Socket]]  
- [[DDR5]]  
- [[PCIe]]

---

## 📚 Further Reading

- [What Is CPU Cache? (Intel)](https://www.intel.com/content/www/us/en/gaming/resources/cpu-cache.html)  
- [Understanding CPU Caches (Red Hat)](https://access.redhat.com/blogs/766093/posts/1976023)  
- [Memory Hierarchy in Modern Architectures – ACM Queue](https://queue.acm.org/detail.cfm?id=945124)

---
