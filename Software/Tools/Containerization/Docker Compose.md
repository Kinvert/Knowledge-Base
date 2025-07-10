# Docker Compose

Docker Compose is a tool for defining and managing multi-container Docker applications using a declarative YAML configuration file. It simplifies the orchestration of complex setups by enabling services like ROS2 nodes, databases, visualizers, and simulators to be spun up with a single command. This is especially useful in robotics, where multiple tightly integrated components must run concurrently.

---

## 📚 Overview

Compose uses a file called `docker-compose.yml` to define services, networks, and volumes. It provides a CLI (`docker compose up`) to start, stop, and manage the entire application stack. It is well-suited for local development, prototyping, integration testing, and edge deployments.

---

## 🧠 Core Concepts

- **Service**: A container definition (image, ports, volumes, etc.)
- **Network**: A virtual network for communication between services
- **Volume**: Persistent or shared storage across containers
- **Depends_On**: Specifies service dependencies and startup order
- **Build**: Option to build an image directly from a local `Dockerfile`

---

## 🧰 Use Cases

- Launching a complete SLAM pipeline (e.g., sensor node, visualizer, backend)
- Running Gazebo + RViz + bridge nodes for simulation
- Defining ROS2 ecosystems for collaborative research
- Starting up diagnostics, monitoring, and control UIs alongside robot code
- Creating portable integration testing environments

---

## ✅ Pros

- Simple orchestration for multi-container setups
- Single command to start/stop the whole system
- Services run in isolated but connected environments
- Supports volume mounts for live development
- Easily portable between machines and teams

---

## ❌ Cons

- No built-in scaling or cluster management (not suitable for large-scale deployments)
- Limited support for advanced networking or autoscaling
- Debugging startup order or failed services can be tricky
- Containers may become tightly coupled if overused

---

## 📊 Comparison Chart

| Feature                | Docker Compose        | Docker CLI (manual)     | [[Kubernetes]]                | [[ROS2 Launch Files]]        |
|------------------------|------------------------|---------------------------|----------------------------|---------------------------|
| **Use Case**           | Local orchestration    | One-off containers        | Cluster orchestration      | ROS-specific orchestration |
| **Syntax**             | YAML                   | CLI                      | YAML                       | Python/XML                |
| **Multi-Container**    | ✅ Built-in            | ❌ Manual                | ✅ Native                  | ⚠️ Manual w/ nodes        |
| **Scaling**            | ⚠️ Limited            | ❌ None                 | ✅ Declarative             | ❌ None                  |
| **Best For**           | Dev & testing setups   | Simple runs               | Production-scale systems   | ROS node config           |

---

## 🤖 Comparison: Docker Compose vs Kubernetes

| Feature              | Docker Compose         | Kubernetes                   |
|----------------------|-------------------------|-------------------------------|
| **Scope**            | Single host             | Multi-node clusters           |
| **Startup Speed**    | ⚡ Fast                 | ⚠️ Slower (more overhead)     |
| **Complexity**       | ✅ Simple              | ❌ Complex                    |
| **Monitoring**       | ⚠️ Minimal             | ✅ Built-in (e.g., Prometheus)|
| **Preferred For**    | Robotics prototyping    | Fleet/cloud robotics          |

---

## 🔧 Compatible Items

- `docker-compose.yml`
- `docker compose up`, `down`, `logs`, `exec`, `build`
- [[Docker Container]], [[Docker Image]], [[Dockerfile]]
- [[ROS2]], [[Gazebo]], [[RViz]]
- [[CI-CD Pipelines]], [[VSCode Dev Containers]]

---

## 🔗 Related Concepts

- [[Dockerfile]] (Used in Compose via `build`)
- [[Docker Image]] (Services are based on these)
- [[Docker Container]] (Runtime of defined services)
- [[Kubernetes]] (Alternative orchestration for production)
- [[ROS2]] (Often orchestrated via Compose)
- [[CI-CD Pipelines]] (Compose-based test environments)

---

## 🛠 Developer Tools

- `docker compose up` — start all services
- `docker compose down` — stop and clean up
- `docker compose logs -f` — view live logs
- `docker compose exec service bash` — open shell inside a service
- `docker compose build` — rebuild services

---

## 📚 Further Reading

- [Docker Compose Official Docs](https://docs.docker.com/compose/)
- [Best Practices for Compose Files](https://docs.docker.com/compose/best-practices/)
- [Docker Compose and ROS2](https://hub.docker.com/_/ros)
- [Dev Containers with Docker Compose](https://code.visualstudio.com/docs/devcontainers/compose)

---
