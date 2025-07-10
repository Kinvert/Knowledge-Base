# Dockerfile

A `Dockerfile` is a text document that defines the instructions to build a Docker Image. It automates the creation of customized environments, including operating systems, libraries, application code, and runtime settings. In robotics, Dockerfiles are crucial for building reproducible containers that include ROS, simulation tools, AI libraries, and hardware dependencies.

---

## 📚 Overview

Dockerfiles define layers in a container image using declarative instructions. Each instruction adds a layer, making the image cacheable, versionable, and reproducible. This approach supports CI/CD, collaborative development, and simplifies deployment across heterogeneous robotic platforms.

---

## 🧠 Core Concepts

- `FROM`: Specifies the base image (e.g., `ubuntu:22.04`, `ros:humble`)
- `RUN`: Executes shell commands to install packages or build code
- `COPY` / `ADD`: Brings files into the image
- `WORKDIR`: Sets the working directory for subsequent instructions
- `CMD` / `ENTRYPOINT`: Specifies the default command when a container starts
- `ENV`: Defines environment variables inside the container
- `ARG`: Accepts build-time variables
- `EXPOSE`: Documents port exposure (optional metadata)

---

## 🧰 Use Cases

- Building ROS2 environments tailored to specific hardware (Jetson, x86)
- Packaging computer vision pipelines with OpenCV and TensorFlow
- Recreating research environments for SLAM or Reinforcement Learning
- Creating lightweight simulation stacks with tools like Ignition or Gazebo
- Sharing development environments via GitHub and Docker Hub

---

## ✅ Pros

- Fully reproducible build environments
- Fine-grained control over dependencies
- Supports multi-stage builds to reduce image size
- Easy integration into CI/CD workflows
- Platform-agnostic builds (with `--platform` and QEMU)

---

## ❌ Cons

- Poorly written Dockerfiles can lead to bloated, insecure images
- Debugging failed builds can be tricky in long multi-step builds
- Frequent changes can invalidate cache layers, slowing rebuilds
- Requires familiarity with shell scripting and Linux package managers

---

## 📊 Comparison Chart

| Feature              | Dockerfile           | [[Conda Env File]]        | [[Makefile]] / [[Bash Script]]   | [[VM Snapshot]]           |
|----------------------|-----------------------|------------------------|---------------------------|------------------------|
| **Purpose**          | Build Docker images   | Configure Python deps  | Automate tasks            | Save entire system     |
| **Scope**            | System-wide           | Python/conda only      | General tasks             | Full OS                |
| **Reproducibility**  | ✅ Strong             | ⚠️ Medium             | ⚠️ Depends on script      | ❌ Weak                |
| **Portability**      | ✅ Cross-platform     | ✅ Conda-based         | ⚠️ Depends on host        | ❌ Limited             |
| **Use with Docker**  | ✅ Essential          | ❌ Not directly         | ⚠️ Sometimes              | ❌ Not used            |

---

## 🤖 Comparison: Dockerfile vs docker-compose.yml

| Feature            | Dockerfile                   | [[docker-compose.yml]]                 |
|--------------------|-------------------------------|-------------------------------------|
| **Purpose**         | Build individual image         | Define multi-container system       |
| **Used By**         | `docker build`                | `docker-compose up`                |
| **Granularity**     | Low-level instructions         | High-level orchestration            |
| **Use Case**        | Create custom images           | Coordinate services (e.g., DB + app)|
| **In Robotics**     | ROS container builds           | SLAM stack with multiple containers |

---

## 🔧 Compatible Items

- `docker build`, `docker tag`, `docker push`
- `Makefile` to invoke build steps
- [[Docker Image]], [[Docker Container]]
- [[VSCode Dev Containers]], [[ROS2]]
- [[CI-CD Pipelines]], [[GitHub Actions]]

---

## 🔗 Related Concepts

- [[Docker]] (Container engine for Dockerfiles)
- [[Docker Image]] (Built from Dockerfile)
- [[Docker Container]] (Instance of image)
- [[ROS2]] (Often containerized via Dockerfiles)
- [[Multi-Stage Build]] (Common optimization pattern)
- [[Dev Containers]] (Often use Dockerfile for base config)

---

## 🛠 Developer Tools

- `docker build -t myimage .`
- `docker run -it myimage bash`
- `docker tag myimage repo/image:tag`
- `docker push repo/image:tag`
- `docker history myimage` to inspect build layers

---

## 📚 Further Reading

- [Dockerfile Reference](https://docs.docker.com/engine/reference/builder/)
- [Best Practices for Dockerfiles](https://docs.docker.com/develop/develop-images/dockerfile_best-practices/)
- [Building Multi-Arch Docker Images](https://docs.docker.com/build/building/multi-platform/)
- [Optimizing Dockerfiles for ROS](https://hub.docker.com/_/ros)

---
