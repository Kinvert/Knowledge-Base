# Microservices Architecture

Microservices Architecture is a design paradigm in which applications are composed of small, independently deployable services that communicate over lightweight protocols, typically HTTP or gRPC. Each microservice is focused on a specific business capability and can be developed, deployed, and scaled independently. This modularity makes microservices particularly well-suited to robotics and simulation environments, where loosely coupled systems (e.g., perception, planning, control) need to interact reliably across distributed platforms.

---

## 📚 Overview

Instead of building a single monolithic system, a microservices approach breaks the application into discrete components, each with its own scope, data, and interface. These services often expose their functionality through REST or gRPC APIs and communicate asynchronously via queues or synchronously via RPC mechanisms. This separation of concerns encourages rapid development, scalability, and robustness.

---

## 🧠 Core Concepts

- **Service Independence**: Each microservice has its own lifecycle, codebase, and persistence.
- **API Contracts**: Well-defined APIs (like gRPC) ensure compatibility and enforce boundaries.
- **Polyglot Programming**: Different services can use different languages or runtimes.
- **Scalability**: Services scale independently based on load.
- **Observability**: Logs, metrics, and traces help monitor and debug systems made of many parts.

---

## 🧰 Use Cases

- Modular robotics stacks: SLAM, path planning, control, and UI as separate services
- Distributed simulations with ROS2 components decoupled into services
- Web-based robot dashboards communicating via REST or gRPC
- kRPC (Kerbal RPC) integrations: Connect real-world control panels to simulated spacecraft
- Building a distributed home automation system using microservices + MQTT
- Real-time multiplayer robotics games or drone competitions

---

## ✅ Pros

- Enables independent development and deployment
- Improves fault isolation and system resilience
- Facilitates experimentation (e.g., swap perception modules)
- Simplifies integration testing for isolated units
- Encourages clean, well-defined interfaces

---

## ❌ Cons

- Operational complexity (deployment, monitoring, service discovery)
- Network latency and overhead from inter-service calls
- Requires distributed tracing to debug full workflows
- Tight version coupling across APIs may introduce fragility
- Harder to test as a complete unit compared to monoliths

---

## 📊 Comparison Chart

| Architecture Style | Deployment Granularity | Scaling | Coupling | Best For                                 |
|--------------------|------------------------|---------|----------|------------------------------------------|
| Monolith           | Single binary          | Global  | Tight    | Small teams, fast prototyping            |
| Microservices      | Multiple services       | Per-service | Loose | Distributed, scalable robotics systems   |
| SOA                | Enterprise-wide         | Global  | Moderate | Legacy systems, complex business domains |
| Serverless         | Per-function            | Automatic | Loose | Event-driven, infrequent compute tasks   |

---

## 🛰️ gRPC and RPC in Context

gRPC is a modern, high-performance Remote Procedure Call (RPC) framework that uses HTTP/2 and Protocol Buffers. It's a common glue layer in microservices systems due to:

- Low latency and bi-directional streaming
- Strongly typed service definitions
- Compatibility with multiple languages

**kRPC** (Kerbal Remote Procedure Call) is a concrete and fun example of RPC in action. It enables external clients to control Kerbal Space Program (KSP) via Python, C++, or other languages. This opens doors to:

- Building physical control panels with real switches and dials
- Creating automated launch sequences or flight controllers
- Running simulations with external telemetry capture
- Teaching RPC concepts via a hands-on simulator

This style of communication mirrors what happens in real-world microservice ecosystems, where services like telemetry, control, and simulation may all run in different containers or languages.

---

## 🔧 Compatible Items

- [[gRPC]], [[Docker Container]], [[Kubernetes]]
- Protocol Buffers (`.proto` files)
- [[ROS2 Services]], [[ROS2 Actions]]
- [[CI-CD Pipelines]] (for deploying microservices)
- [[Dev Containers]], [[Edge Computing]], [[MQTT]]

---

## 🔗 Related Concepts

- [[gRPC]] (Protocol used between services)
- [[ROS2 Services]] (Conceptually similar to RPC)
- [[ROS2 Actions]] (For longer-running RPC-like tasks)
- [[Docker Compose]] (Defines services and their relationships)
- [[Kubernetes]] (Runs and manages services at scale)
- [[CI-CD Pipelines]] (Builds and tests microservices)
- [[Kubernetes Service]] (Exposes a microservice in a cluster)

---

## 🛠 Developer Tools

- `protoc` – compile `.proto` files for gRPC services
- `grpcurl` – test gRPC APIs interactively
- `Postman` – REST testing tool, useful for non-gRPC microservices
- `kRPC Console` – interface to Kerbal RPC server
- `docker compose up` – start microservice networks locally
- `kubectl` – manage microservices deployed on Kubernetes

---

## 📚 Further Reading

- [Microservices at a Glance](https://martinfowler.com/articles/microservices.html)
- [gRPC Concepts](https://grpc.io/docs/what-is-grpc/introduction/)
- [kRPC for Kerbal Space Program](https://krpc.github.io/krpc/)
- [Designing Microservices](https://12factor.net/)
- [Google's Site Reliability Engineering Book](https://sre.google/books/)

---

## 🗂 Suggested Folder Location

Systems > Cloud and Orchestration  
or  
Software > Robotics > Architecture  
or  
Software > Tools > Distributed Systems
