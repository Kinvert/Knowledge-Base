# Docker Image

A Docker Image is a lightweight, standalone, and executable package that includes everything needed to run a piece of software—code, runtime, libraries, environment variables, and configuration files. In robotics and engineering, Docker images allow developers to distribute reproducible environments across various platforms, ensuring consistency from development to deployment.

---

## 📚 Overview

Docker images serve as the *blueprint* for Docker containers. They are built from a [[Dockerfile]], can be versioned and layered, and are typically stored in registries like Docker Hub or GitHub Container Registry (GHCR). When you run an image, it becomes a container.

---

## 🧠 Core Concepts

- **Immutable Layers**: Each instruction in a Dockerfile creates a new layer, enabling caching and version control.
- **Base Image**: The starting point for a custom image (e.g., `ubuntu`, `ros:foxy`).
- **Tagged Versions**: Images can be tagged (e.g., `myimage:latest`, `ros:humble`).
- **Registries**: Online storage for images, public or private (e.g., Docker Hub, GHCR).
- **Multistage Builds**: Efficient way to separate build dependencies from runtime.

---

## 🧰 Use Cases

- Packaging a robotics workspace with ROS2, OpenCV, and drivers
- Distributing a vision pipeline or AI model inference tool
- Creating development environments for CI pipelines
- Running deterministic simulations on different machines
- Deploying robotics applications to the cloud or edge

---

## ✅ Pros

- Ensures consistent runtime environments
- Images can be shared, cached, and versioned
- Supports layering, reducing redundancy
- Works across platforms with emulation (e.g., `arm64` via QEMU)

---

## ❌ Cons

- Images can become large if not optimized
- Security must be maintained through regular updates
- May require special configuration for hardware access (e.g., USB, GPU)
- Poorly layered Dockerfiles lead to slow builds

---

## 📊 Comparison Chart

| Feature               | Docker Image          | [[Docker Container]]        | [[Virtual Machine Image]]      | [[Python Virtualenv]]       |
|-----------------------|------------------------|--------------------------|-----------------------------|--------------------------|
| **Purpose**           | Blueprint              | Runtime instance         | Full OS snapshot            | Isolated Python deps     |
| **Mutability**        | Immutable              | Mutable at runtime       | Mutable                     | Mutable                  |
| **Resource Use**      | ✅ Efficient           | ✅ Efficient             | ❌ Heavy                    | ✅ Lightweight           |
| **Start Time**        | ⚡ Instant             | ⚡ Instant               | 🐢 Slow                    | ⚡ Instant               |
| **Used In**           | CI/CD, cloud, robotics | Local/remote execution   | Legacy systems, VMs         | Local dev                |

---

## 🧪 Comparison: Docker Image vs Kubernetes Pod

| Feature             | Docker Image              | [[Kubernetes Pod]]                            |
|---------------------|----------------------------|--------------------------------------------|
| **Role**            | Software environment       | Execution unit in Kubernetes               |
| **Scope**           | Single app per image       | One or more containers with shared context |
| **Build Time**      | Built once                 | Uses existing images                       |
| **Deployment**      | Manual or scripted         | Declarative via YAML manifests             |
| **Best Use Case**   | Packaging and testing      | Scalable orchestration                     |

---

## 🔧 Compatible Items

- `Dockerfile`, `docker build`, `docker save/load`
- `docker push`, `docker pull`, `docker tag`
- [[Docker]] (The runtime for images)
- [[VSCode Dev Containers]]
- [[GitHub Actions]] (Images in CI/CD pipelines)
- [[ROS2]], [[Jetson Family]], [[Gazebo]], [[TensorFlow]]

---

## 🔗 Related Concepts

- [[Docker]] (Overall container platform)
- [[Docker Container]] (Instance created from an image)
- [[Kubernetes]] (Orchestrates containers from images)
- [[Dev Containers]] (Development using Docker images)
- [[ROS2]] (Often built into Docker images)
- [[CI-CD]] (Docker images ensure reproducibility)

---

## 🛠 Developer Tools

- `docker build -t myimage:tag .`
- `docker images`, `docker inspect`
- `docker tag`, `docker push`, `docker pull`
- `docker save`, `docker load` for air-gapped systems
- `multiarch/qemu-user-static` for multi-platform builds

---

## 📚 Further Reading

- [Dockerfile Reference](https://docs.docker.com/engine/reference/builder/)
- [Docker Image Best Practices](https://docs.docker.com/develop/develop-images/dockerfile_best-practices/)
- [Working with Docker Images](https://docs.docker.com/get-started/02_our_app/)
- [ROS2 Docker Images](https://hub.docker.com/_/ros)

---
