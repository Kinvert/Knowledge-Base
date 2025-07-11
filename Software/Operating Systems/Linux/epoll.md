# epoll

**epoll** is a scalable I/O event notification mechanism in the Linux kernel, designed to efficiently monitor large numbers of file descriptors. It is commonly used in high-performance servers, network applications, and embedded systems requiring asynchronous, non-blocking I/O.

---

## 📚 Overview

epoll improves on older mechanisms like `select()` and `poll()` by avoiding the need to linearly scan file descriptors each time. Instead, it uses an event-driven interface where the kernel notifies the user process when I/O events are ready, drastically reducing CPU overhead and latency in applications that manage thousands of sockets or devices.

It operates through three system calls: `epoll_create1()`, `epoll_ctl()`, and `epoll_wait()`.

---

## 🧠 Core Concepts

- **epoll instance**: A kernel object tracking a set of file descriptors  
- **Edge-Triggered vs Level-Triggered**: Edge is more efficient but complex; level is simpler  
- **Event loop**: Application blocks on `epoll_wait()` until an event occurs  
- **File descriptors**: Can represent sockets, pipes, or other I/O streams  
- **Non-blocking I/O**: epoll is often used with non-blocking file descriptors  

---

## 🧰 Use Cases

- High-performance web servers (e.g., NGINX)  
- Real-time communication systems  
- Robotics applications with many sensors  
- Industrial gateways interfacing with multiple devices  
- Network protocol stacks and asynchronous APIs  

---

## ✅ Pros

- Scalable to thousands of connections  
- Efficient kernel-level event dispatch  
- No need to repeatedly pass entire FD sets  
- Supports both edge and level triggering  
- Suitable for real-time and low-latency systems  

---

## ❌ Cons

- Linux-specific (not portable to Windows/macOS)  
- Edge-triggered mode is complex and easy to misuse  
- Requires file descriptors to be set to non-blocking  
- Not compatible with all types of file descriptors (e.g., some special devices)  

---

## 📊 Comparison: I/O Event Models

| Feature             | select()     | poll()       | epoll        | kqueue (BSD)  | IOCP (Windows) |
|---------------------|--------------|--------------|--------------|----------------|----------------|
| Scalability         | Poor         | Poor         | Excellent    | Excellent       | Excellent       |
| Kernel Support      | All UNIX     | All UNIX     | Linux only   | BSD/macOS       | Windows only    |
| Edge Triggering     | No           | No           | Yes          | Yes             | Yes             |
| Dynamic FD Set      | No           | No           | Yes          | Yes             | Yes             |
| Use in Robotics     | Rare         | Sometimes    | Frequent     | Rare            | Rare            |

---

## 🤖 In a Robotics Context

| Application               | epoll Use Case                              |
|---------------------------|---------------------------------------------|
| Sensor Polling            | Efficiently wait for input from many devices  
| Multi-process IPC         | Monitor multiple communication pipes/sockets  
| Robot Web Interfaces      | Enable async HTTP servers for control  
| Embedded Gateways         | Handle concurrent device I/O streams  
| Diagnostic Tooling        | Aggregate logs or status reports asynchronously  

---

## 🔧 Developer Tools & Libraries

- `select`, `poll`, `epoll` – Standard in Python (`selectors` module), C, Rust, etc.  
- **libev** / **libevent** – Event loop libraries wrapping epoll  
- **boost::asio** – Uses epoll under the hood on Linux  
- **asyncio** – Python's async runtime leverages epoll  
- **nginx**, **haproxy**, **redis** – Use epoll for high-performance networking  

---

## 🔧 Compatible Items

- [[Real-Time Systems]] – epoll is compatible with soft real-time systems  
- [[Linux]] – Required OS for epoll functionality  
- [[Multi-threaded Systems]] – Used in conjunction with thread pools for parallel I/O  

---

## 🔗 Related Concepts

- [[Linux]] (Kernel-level feature only available in Linux)  
- [[Asynchronous Programming]] (epoll is foundational in async frameworks)  
- [[Event Loop]] (epoll is a backend mechanism for event loops)  
- [[Sockets]] (Commonly monitored using epoll)  
- [[File Descriptors]] (Core to UNIX I/O models)  

---

## 📚 Further Reading

- [Linux man page: epoll](https://man7.org/linux/man-pages/man7/epoll.7.html)  
- [Edge vs Level Triggered epoll – LWN.net](https://lwn.net/Articles/611906/)  
- [The epoll API in Depth (IBM Developer)](https://developer.ibm.com/articles/l-epoll/)  
- [Understanding epoll (Ulrich Drepper)](https://www.akkadia.org/drepper/)  
- [Python selectors module](https://docs.python.org/3/library/selectors.html)

---
