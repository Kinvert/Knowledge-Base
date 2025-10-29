# cProfile

`cProfile` is a built-in Python module used to measure the performance of programs by collecting detailed statistics on function calls. It helps developers identify bottlenecks in their code by reporting how much time is spent in each function, how many times each function is called, and the relationships between callers and callees. This makes it an essential tool for performance optimization in robotics systems where efficiency and timing are critical.

---

## ⚙️ Overview

`cProfile` is part of Python’s standard library and provides deterministic profiling — meaning it tracks every function call in a predictable way rather than sampling execution at intervals. It’s ideal for identifying slow or frequently called functions in robotics software, especially in computationally intensive pipelines such as [[SLAM]], [[Path Planning]], or real-time [[Control Theory]] loops.

You can invoke it directly from the command line using `python -m cProfile your_script.py` or within code using the `cProfile.Profile()` class.

---

## 🧠 Core Concepts

- **Deterministic Profiling:** Tracks all function calls and returns.
- **Call Count:** Number of times a function is invoked.
- **Total Time:** Total time spent within a function (excluding subcalls).
- **Cumulative Time:** Total time spent in a function and all its subcalls.
- **Call Graph:** Relationship of caller and callee functions.
- **Stats Output:** Results can be saved using the `pstats` module for sorting and further analysis.

---

## 🧩 How It Works

1. **Start Profiling:** Create a `cProfile.Profile()` object.
2. **Run Code:** Wrap target code or function using `profiler.runcall()` or `profiler.enable()/disable()`.
3. **Collect Stats:** After execution, retrieve and sort statistics.
4. **Analyze Output:** View by total time, cumulative time, or function call frequency.

Command-line example:  
`python -m cProfile -s cumulative your_script.py`

---

## 📊 Comparison Chart

| Profiler | Type | Overhead | Suitable For | Notes |
|-----------|------|-----------|---------------|--------|
| **cProfile** | Deterministic | Moderate | General-purpose Python profiling | Built into Python, reliable and well-supported |
| **profile** | Deterministic | High | Small scripts | Pure Python, slower than `cProfile` |
| **line_profiler** | Line-level | High | Fine-grained performance analysis | Requires decorator usage |
| **pyinstrument** | Sampling | Low | Web apps, long-running programs | Less precise, lower overhead |
| **yappi** | Deterministic | Moderate | Multi-threaded apps | Handles concurrency better |
| **perf** | Sampling | Low | System-level performance | Linux tool, not Python-specific |

---

## 🛠️ Use Cases

- Profiling motion control code in real-time robotics.
- Optimizing sensor fusion pipelines.
- Measuring time consumption in deep learning inference (e.g., [[PyTorch]] or [[TensorFlow]]).
- Debugging latency in robotic middleware.
- Identifying performance bottlenecks in simulation environments like [[Gazebo]] or [[Ignition]].

---

## ✅ Strengths

- Included in Python’s standard library (no extra installation).
- Deterministic and reliable.
- Integrates well with `pstats` and external tools like [[SnakeViz]] or [[KCachegrind]].
- Can output to file for later inspection or visualization.

---

## ❌ Weaknesses

- Adds measurable overhead to execution.
- Provides function-level granularity (not line-level).
- Harder to visualize without external tools.
- Does not inherently handle multi-threaded performance accurately.

---

## 🧰 Compatible Tools

- [[SnakeViz]] – Web-based visualizer for cProfile output.
- [[KCachegrind]] – GUI visualizer (via conversion with `pyprof2calltree`).
- [[pstats]] – Python module for post-processing profile data.
- [[line_profiler]] – Complementary line-by-line profiling.
- [[cProfilev]] – Lightweight web viewer for `cProfile` data.

---

## 🔗 Related Concepts

- [[Profiling]] (Performance measurement in software)
- [[Performance Optimization]]
- [[pstats]] (Profile statistics viewer)
- [[SnakeViz]] (Visualization tool)
- [[Threading]] (Concurrency in Python)
- [[Multiprocessing]] (Parallelism in Python)
- [[Python]] (Programming Language)

---

## 📚 External Resources

- Official Python Documentation: https://docs.python.org/3/library/profile.html
- SnakeViz: https://jiffyclub.github.io/snakeviz/
- Yappi Profiler: https://github.com/sumerc/yappi
- Tutorial: https://realpython.com/python-profile/

---

## 🧭 Summary

`cProfile` is a core performance diagnostic tool for Python developers, especially valuable in robotics where computational efficiency matters. Its deterministic nature and integration into the Python ecosystem make it a first-line profiler for understanding code performance, before moving on to more specialized tools like `line_profiler` or sampling profilers.

---
