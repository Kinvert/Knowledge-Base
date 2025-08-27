# perf (Linux Performance Analysis Tool)

## Overview
`perf` is a powerful Linux profiling and performance analysis tool that provides insights into CPU usage, cache misses, context switches, and many other low-level hardware and kernel events. It helps developers and system administrators understand where bottlenecks exist in code and system execution.  

It is part of the Linux kernel tools (`linux-tools-common`, `linux-tools-$(uname -r)`), and integrates tightly with kernel performance counters, making it a go-to choice for profiling applications and the system.

---

## Key Features
- Low-overhead profiling using hardware performance counters.  
- Sampling and tracing capabilities for both user-space and kernel-space.  
- Records events like CPU cycles, instructions, cache references/misses, page faults, and context switches.  
- Supports function-level and line-level profiling via DWARF/unwind or frame pointers.  
- Can generate flame graphs and detailed call graphs.  
- Works well with multi-threaded and multi-core applications.  

---

## Common Use Cases
- Identify CPU-bound vs. I/O-bound workloads.  
- Pinpoint hot functions in code during execution.  
- Analyze cache performance for optimization.  
- Measure the impact of context switches and scheduling.  
- Performance regression testing.  

---

## One-Liner Cheatsheet

Here are some **`perf` one-liners** you’ll frequently use:

`perf list` → Show all available events to profile.  
`perf stat ls` → Show performance counters for running `ls`.  
`perf stat -p 1234` → Collect stats for process with PID `1234`.  
`perf stat -a -- sleep 5` → System-wide stats for 5 seconds.  
`perf top` → Real-time profiling of hottest functions.  
`perf record ./a.out` → Record performance data while running program.  
`perf report` → Display interactive analysis from last `perf record`.  
`perf record -g ./a.out` → Record with call-graph (stack traces).  
`perf annotate` → Show annotated disassembly with profiling data.  
`perf record -e cache-misses ./a.out` → Record cache misses for `a.out`.  
`perf stat -e cycles,instructions ./a.out` → Track CPU cycles and instructions.  
`perf record -F 99 -a -g -- sleep 10` → Profile whole system at 99Hz for 10s.  
`perf script` → Dump recorded perf data for external analysis.  
`perf sched record; perf sched latency` → Analyze scheduler latency.  
`perf mem record ./a.out` → Profile memory load/store performance.  

---

## Comparison with Other Tools

| Tool         | Strengths | Weaknesses | Typical Use Case |
|--------------|-----------|------------|------------------|
| **perf**     | Kernel-integrated, low overhead, wide range of events, call graph support | Requires root for some features, steep learning curve | General Linux performance profiling |
| **[[GDB]]**      | Debugging with breakpoints, memory inspection | Not designed for profiling, slower due to pausing execution | Debugging logic/segfaults |
| **[[valgrind]] (callgrind)** | Very detailed cache and branch profiling, easy visualization with KCacheGrind | High overhead (10-100x slowdown) | Deep function-level performance analysis |
| **[[strace]]**   | System call tracing | High overhead, no CPU/memory profiling | Debugging syscalls, I/O bottlenecks |
| **htop / top** | Easy system monitoring | Limited insight into hardware counters | Quick CPU/memory overview |
| **sysdig**   | System call and kernel tracing | Overhead can be noticeable, not CPU-cycle accurate | Security monitoring, syscall-level debugging |
| **[[DTrace]] / bpftrace** | Dynamic tracing, extremely powerful | Requires kernel support, learning curve | Advanced kernel + app performance tracing |

---

## Learning Resources
- [Official perf wiki](https://perf.wiki.kernel.org/index.php/Main_Page)  
- [Brendan Gregg’s perf examples](http://www.brendangregg.com/perf.html)  
- [Flame Graphs](http://www.brendangregg.com/flamegraphs.html)  

---

## Notes
- Use `sudo` if you see permission errors (`perf_event_paranoid`).  
- Combine with `FlameGraph` tools for visualization of stack traces.  
- Overhead is typically very low compared to Valgrind, making `perf` suitable for production.  

---

## Summary
`perf` is the **Swiss Army knife of Linux performance tools**, offering deep insights into CPU, memory, and scheduling performance. Unlike debuggers (`gdb`) or higher-overhead analyzers (`valgrind`), `perf` provides **lightweight yet detailed profiling** suitable for both development and production environments. It is best used in combination with visualization tools (like Flame Graphs) for maximum clarity.  
