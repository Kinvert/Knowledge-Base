# Docker Container

A Docker Container is a lightweight, standalone, and executable software package that includes everything needed to run a specific application: code, runtime, libraries, and settings. Unlike traditional virtual machines, containers share the host OS kernel and are much more efficient, making them ideal for robotics, embedded systems, and scalable cloud applications.

---

## 📚 Overview

Containers are created from Docker Images and can be started, stopped, paused, and restarted quickly. In robotics, containers allow engineers to isolate ROS environments, simplify dependency management, and deploy consistent builds across different hardware and platforms.

---

## 🧠 Core Concepts

- **Image vs Container**: An image is a static blueprint; a container is a running instance of that image.
- **Isolation**: Containers run in isolated user spaces with their own process trees, file systems, and network interfaces.
- **Port Mapping**: Enables services inside the container to be accessed from outside.
- **Volumes**: Provide persistent or shared storage between the host and container.
- **Entrypoint & CMD**: Control the default command and behavior when the container starts.

---

## 🧰 Use Cases

- Running isolated ROS2 nodes for development or deployment
- Testing vision pipelines, sensor drivers, or SLAM algorithms
- Launching web servers for robot diagnostics or configuration UIs
- Running Gazebo simulations in CI pipelines
- Deploying edge-AI inference models on NVIDIA Jetson hardware

---

## ✅ Pros

- Fast startup and low resource usage
- Consistent environments across dev, test, and production
- Easy to distribute and version using Docker Hub or GHCR
- Simplifies debugging and reproducing bugs in robotics systems
- Compatible with GPU acceleration and USB pass-through

---

## ❌ Cons

- Hardware access (e.g., GPIO, serial, USB) requires extra configuration
- Containers are ephemeral by default (data loss unless volumes used)
- Networking can be confusing across hosts or clusters
- Real-time performance needs careful tuning and OS configuration

---

## 📊 Comparison Chart

| Feature               | Docker Container         | [[Docker Image]]             | [[Kubernetes Pod]]           | [[Virtual Machine]]         |
|-----------------------|---------------------------|----------------------------|----------------------------|--------------------------|
| **State**             | Runtime                   | Static                    | Runtime (orchestrated)    | Runtime (with full OS)   |
| **Isolation**         | Process-level             | N/A                       | Multi-container isolation | Full OS + Hypervisor     |
| **Startup Time**      | ⚡ Fast                   | N/A                       | ⚠️ Fast (but managed)     | 🐢 Slow                 |
| **Persistence**       | ❌ Unless mounted         | ✅ Always                 | ✅ Via volumes            | ✅ Via disk              |
| **Common Use Case**   | Development, testing      | Blueprint for containers  | Scaled deployment         | Full legacy software     |

---

## 🤖 Comparison: Docker Container vs Kubernetes Pod

| Feature              | Docker Container         | [[Kubernetes Pod]]                 |
|----------------------|---------------------------|----------------------------------|
| **Scope**            | Single container          | One or more containers          |
| **Orchestration**    | ❌ Manual or Compose       | ✅ Automated via K8s            |
| **Networking**       | Bridged or host-mode       | Shared network namespace        |
| **Fault Tolerance**  | ⚠️ Needs manual scripting | ✅ Built-in restart strategies  |
| **Deployment Style** | CLI, Compose, scripts      | Declarative YAML                |

---

## 🔧 Compatible Items

- `docker run`, `docker stop`, `docker exec`, `docker ps`
- `docker-compose` for multi-container setup
- [[Docker Image]], [[ROS2]], [[Gazebo]], [[CI-CD]]
- [[VSCode Dev Containers]], [[Jetson Family]]
- [[Podman]] (alternative runtime)

---

## 🔗 Related Concepts

- [[Docker]] (Container platform and tooling)
- [[Docker Image]] (Container blueprint)
- [[Kubernetes Pod]] (Orchestrated unit of containers)
- [[VSCode Dev Containers]] (Containers for local development)
- [[ROS2]] (Often deployed in containers)
- [[CI-CD]] (Containers run test jobs or builds)

---

## 🛠 Developer Tools

- `docker run -it ros:humble /bin/bash`
- `docker exec -it <container_id> bash`
- `docker-compose up`, `docker stop`, `docker logs`
- `docker cp` to transfer files between host and container
- Use `--mount` or `-v` for volume persistence

---

## 📚 Further Reading

- [Docker Containers Explained](https://docs.docker.com/get-started/)
- [Best Practices for Running ROS in Docker](https://hub.docker.com/_/ros)
- [Using Docker with GPU and Jetson](https://docs.nvidia.com/datacenter/cloud-native/)
- [Understanding Volumes in Docker](https://docs.docker.com/storage/volumes/)
- [Docker vs Podman Comparison](https://www.redhat.com/en/blog/podman-can-now-run-docker-compose)

---
