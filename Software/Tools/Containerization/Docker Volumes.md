# Docker Volumes

**Docker Volumes** are persistent data storage mechanisms used by Docker containers. Unlike container layers, which are ephemeral, volumes provide a way to persist and share data between containers and across container restarts. Volumes are essential for managing data in robotics and engineering workflows—whether it’s logs, sensor data, simulation outputs, or configuration files.

---

## 📚 Overview

Volumes are stored in the host filesystem (usually under `/var/lib/docker/volumes/`) and can be mounted into containers at runtime. This allows data generated by containers to survive container deletions and updates, which is especially useful in CI/CD pipelines, long-running simulations, and when interfacing with external systems.

---

## 🧠 Core Concepts

- **Named Volumes**: Reusable and easier to manage (`my_data_volume`)
- **Anonymous Volumes**: Created automatically, not easily referenced
- **Volume Drivers**: Plugins that allow custom storage backends (e.g., NFS, cloud storage)
- **Mount Points**: Inside containers (e.g., `/data` or `/ros_ws/src`)
- **Bind Mounts vs Volumes**: Bind mounts directly map a host path; volumes are Docker-managed

---

## 🧰 Use Cases

- Store ROS2 workspace builds and logs
- Share sensor calibration data between runs
- Persist database files for mapping or vision data
- Feed simulation results into post-processing scripts
- Enable communication between containers (e.g., ROS2 ↔ web UI)

---

## ✅ Pros

- Data persists across container lifecycles
- Easy to share data between containers
- Backed by Docker's volume management (portable and manageable)
- Can be used in Docker Compose and Kubernetes
- Reduces coupling between container logic and host filesystem

---

## ❌ Cons

- Anonymous volumes can clutter the system if not cleaned
- Harder to manage than bind mounts for debugging
- Limited introspection unless explicitly mounted or inspected
- Network volume drivers may add latency

---

## 📊 Comparison Chart

| Feature                  | Docker Volume       | Bind Mount            | tmpfs Mount          |
|--------------------------|---------------------|------------------------|-----------------------|
| Persistent Storage       | ✅ Yes              | ✅ Yes                | ❌ Ephemeral         |
| Docker Managed           | ✅ Yes              | ❌ No                 | ✅ Yes               |
| Ease of Use              | ✅ Named volumes     | ⚠️ Host path needed   | ✅ Easy              |
| Security                 | ✅ Scoped by Docker  | ❌ Full host access   | ✅ Isolated           |
| Portability              | ✅ High              | ❌ Low                | ❌ Low                |

---

## 🤖 In a Robotics Context

| Use Case                          | Docker Volume Role                              |
|-----------------------------------|--------------------------------------------------|
| ROS2 workspace persistence        | Store `/ros_ws` between builds                   |
| SLAM map output                   | Persist `.pgm` and `.yaml` files after run       |
| Simulation log collection         | Store Gazebo logs and bag files                  |
| Configuration sharing             | Provide shared config files to all nodes        |
| Web ↔ backend bridge              | Exchange JSON or gRPC results via volume         |

---

## 🔧 Useful Commands (One-Liners)

- `docker volume create my_volume` – Create a named volume  
- `docker run -v my_volume:/app/data my_image` – Mount volume to container  
- `docker volume ls` – List all volumes  
- `docker volume inspect my_volume` – Show volume details  
- `docker volume rm my_volume` – Remove volume  
- `docker volume prune` – Remove unused volumes  

---

## 🔧 Compatible Items

- [[Dockerfile]] – Doesn’t define volumes but must support them
- [[Docker Compose]] – Mounts volumes across services
- [[ROS2 Package]] – Data or config can be volume-mounted
- [[CI-CD Pipelines]] – Store build/test artifacts persistently
- [[Kubernetes Pod]] – Analogous concept: `PersistentVolumeClaim`

---

## 🔗 Related Concepts

- [[Docker Container]] (Volumes are attached to containers)
- [[Docker Compose]] (Manages and mounts volumes across services)
- [[Kubernetes Volume]] (Equivalent concept for k8s)
- [[CI-CD Pipelines]] (Volumes help persist logs/artifacts)
- [[ROS2 Launch Files]] (Use mounted config or bag files)

---

## 📚 Further Reading

- [Docker Docs: Volumes](https://docs.docker.com/storage/volumes/)
- [Docker Bind Mounts vs Volumes](https://docs.docker.com/storage/)
- [Managing Volumes in Compose](https://docs.docker.com/compose/compose-file/compose-file-v3/#volumes)
- [Best Practices for Docker Data](https://docs.docker.com/storage/storagedriver/)

---
