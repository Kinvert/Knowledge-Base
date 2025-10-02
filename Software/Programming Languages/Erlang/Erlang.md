# Erlang

Erlang is a functional, concurrent programming language originally developed at Ericsson for building scalable, fault-tolerant systems. It is particularly well-known in telecommunications and distributed systems, where reliability and uptime are critical. Erlang’s runtime system (the BEAM virtual machine) emphasizes lightweight processes, message passing, and fault tolerance through supervision trees.

---

## 📚 Overview

Erlang is designed for **concurrency**, **distribution**, and **fault tolerance**. It enables developers to write applications that can scale across many nodes and recover gracefully from failures. Erlang is often used where continuous operation and real-time responsiveness are essential.

---

## 🧠 Core Concepts

- **Actor model of concurrency**: computation is performed by processes that communicate via message passing.
- **Lightweight processes**: millions of processes can run concurrently, managed by the BEAM VM.
- **Fault tolerance**: “let it crash” philosophy combined with supervision hierarchies.
- **Hot code swapping**: supports upgrading code in running systems without downtime.
- **Immutable data**: functional style ensures predictability in concurrent operations.

---

## 📊 Comparison Chart

| Language/Framework     | Concurrency Model       | Fault Tolerance | Primary Use Cases                  | VM/Runtime       |
|-------------------------|-------------------------|-----------------|------------------------------------|------------------|
| **Erlang**             | Actor model (message passing) | Strong (supervision trees) | Telecom, distributed systems, messaging | BEAM VM |
| **Elixir**             | Actor model (built on Erlang) | Strong | Web apps, distributed services | BEAM VM |
| **Go**                 | Goroutines + channels   | Medium          | Cloud services, infrastructure     | Native runtime   |
| **Akka (Scala/Java)**  | Actor model             | Medium-High     | Distributed event-driven systems   | JVM              |
| **Rust**               | Ownership + async/await | Medium          | Systems programming, safety        | Native           |
| **C++ (with threads)** | Shared memory, threads  | Low             | High-performance systems           | Native           |

---

## 🛠️ Use Cases

- Telecom systems (Erlang’s origin and primary success)
- Distributed databases (e.g., [[Riak]])
- Messaging platforms (e.g., WhatsApp backend)
- IoT systems requiring uptime and scalability
- Control systems in robotics requiring fault tolerance

---

## ✅ Strengths

- High reliability in long-running systems
- Massive concurrency support
- Distribution and scaling across nodes is built-in
- Robust standard library for networking and concurrency
- Mature ecosystem for telecommunications

---

## ❌ Weaknesses

- Less mainstream than other languages, smaller developer pool
- Syntax can feel unusual compared to C-like languages
- Performance not always as high as systems-level languages like [[C++]]
- Limited ecosystem for modern web tooling (better handled in [[Elixir]])

---

## 🔧 Compatible Items

- [[Elixir]] (runs on BEAM VM, interoperable with Erlang)
- [[OTP]] (Open Telecom Platform, Erlang’s set of libraries and design principles)
- [[BEAM]] (Erlang virtual machine)
- [[Riak]] (distributed database written in Erlang)
- [[RabbitMQ]] (messaging broker built on Erlang)

---

## 📑 Related Concepts

- [[Actor Model]] (message-based concurrency paradigm)
- [[Elixir]] (a modern functional language leveraging BEAM)
- [[OTP]] (framework for building fault-tolerant applications)
- [[BEAM]] (virtual machine runtime for Erlang/Elixir)
- [[Functional Programming]] (paradigm Erlang follows)

---

## 🌍 External Resources

- Official Erlang site: https://www.erlang.org
- Erlang/OTP GitHub: https://github.com/erlang/otp
- Learn You Some Erlang (community resource): https://learnyousomeerlang.com
- WhatsApp engineering blog (Erlang in practice): https://engineering.fb.com

---

## 🏆 Summary

Erlang remains one of the most influential programming languages in the domain of **concurrent and distributed systems**. Its principles of fault tolerance, scalability, and “let it crash” philosophy have shaped the design of many modern distributed frameworks and languages, especially through the influence of the BEAM VM and OTP libraries.
