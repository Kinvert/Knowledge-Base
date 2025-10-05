# Docker CLI Cheatsheet

Docker is a powerful platform for building, shipping, and running applications inside lightweight containers. The Docker CLI (Command-Line Interface) provides a wide range of commands to manage containers, images, networks, and volumes. This cheatsheet is a comprehensive reference for engineers who want to quickly find and apply Docker commands in robotics, development, or production workflows.

---

## ⚙️ Overview

Docker CLI commands fall into several categories: managing containers, images, networks, volumes, system resources, and advanced orchestration. Understanding these commands is essential for automation, debugging, and deployment.

---

## 📖 Core Concepts

- **Container**: A lightweight, isolated runtime environment for applications.
- **Image**: A template used to create containers.
- **Registry**: A repository for storing and distributing images (e.g., Docker Hub).
- **Volume**: A mechanism for persisting and sharing data between containers.
- **Network**: Virtual networks for container communication.
- **Daemon**: The background service managing Docker containers (`dockerd`).

---

## 🔑 Key Commands

### Container Management
- `docker ps` – List running containers
- `docker ps -a` – List all containers (including stopped)
- `docker run <image>` – Run container from image
- `docker exec -it <container> <cmd>` – Run command in running container
- `docker stop <container>` – Stop a running container
- `docker start <container>` – Start a stopped container
- `docker restart <container>` – Restart a container
- `docker kill <container>` – Force stop a container
- `docker rm <container>` – Remove container
- `docker logs <container>` – View logs of a container
- `docker attach <container>` – Attach to a container’s process

### Image Management
- `docker images` – List local images
- `docker pull <image>` – Download image from registry
- `docker push <image>` – Upload image to registry
- `docker build -t <tag> .` – Build image from Dockerfile
- `docker rmi <image>` – Remove image
- `docker save -o file.tar <image>` – Save image to tarball
- `docker load -i file.tar` – Load image from tarball
- `docker tag <source> <target>` – Tag an image

### File & Data
- `docker cp <container>:<path> <local>` – Copy from container to host
- `docker cp <local> <container>:<path>` – Copy from host to container
- `docker volume create <name>` – Create volume
- `docker volume ls` – List volumes
- `docker volume rm <name>` – Remove volume

### Networking
- `docker network ls` – List networks
- `docker network create <name>` – Create network
- `docker network rm <name>` – Remove network
- `docker network inspect <name>` – Inspect network

### System & Info
- `docker info` – Display system-wide information
- `docker version` – Show Docker version
- `docker system df` – Show disk usage
- `docker system prune` – Remove unused containers, images, networks
- `docker stats` – Show live resource usage
- `docker top <container>` – Show running processes inside container

---

## 🛠️ Common One-Liners

- Start a container and enter bash: `docker run -it ubuntu bash`
- Remove all stopped containers: `docker rm $(docker ps -aq)`
- Remove all dangling images: `docker rmi $(docker images -q -f dangling=true)`
- Clean up everything: `docker system prune -a`
- Run container with port mapping: `docker run -p 8080:80 nginx`
- Run container with volume: `docker run -v /host/path:/container/path ubuntu`
- Check container logs in real-time: `docker logs -f <container>`
- Export container filesystem: `docker export <container> > container.tar`
- Import container filesystem: `cat container.tar | docker import - newimage`

---

## 🏆 Use Cases

- **Development**: Test robotics code inside isolated environments.
- **CI/CD**: Automate builds with reproducible images.
- **Simulation**: Run ROS or Gazebo in containers without polluting the host.
- **Deployment**: Push robot control software to edge devices.
- **Debugging**: Inspect dependencies inside containers.

---

## ✅ Strengths

- Portable and lightweight compared to virtual machines
- Large ecosystem and community support
- Easy integration with CI/CD pipelines
- Fine-grained resource isolation

---

## ❌ Weaknesses

- Security risks if not patched or configured correctly
- Networking can become complex
- Containers are ephemeral unless data is persisted with volumes
- Requires some learning curve for orchestration at scale

---

## 📚 Related Notes

- [[Docker]]
- [[Podman]] (Daemonless container engine)
- [[LXC]] (Linux Containers)
- [[Nomad]] (Workload orchestrator)
- [[CI-CD]] (Continuous Integration and Continuous Deployment)

---

## 🌐 External Resources

- [Docker Official Documentation](https://docs.docker.com/)
- [Docker Hub](https://hub.docker.com/)
- [Play with Docker](https://labs.play-with-docker.com/)
- [Awesome Docker GitHub](https://github.com/veggiemonk/awesome-docker)

---
