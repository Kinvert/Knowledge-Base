# Kubernetes Container Runtime

The **Kubernetes Container Runtime** is the low-level component responsible for running containers on each worker node. It serves as the bridge between the `kubelet` and the actual container execution engine, enabling Kubernetes to launch, monitor, and manage containers. In robotics, this layer ensures that ROS2 nodes, perception modules, or simulators are reliably executed on a variety of hardware platforms.

---

## 📚 Overview

Kubernetes does **not** run containers directly. Instead, it uses a **Container Runtime Interface (CRI)** that abstracts away the underlying runtime. Popular runtimes like **containerd**, **CRI-O**, and formerly **Docker** implement the CRI to allow Kubernetes to manage containers in a standardized way.

---

## 🧠 Core Concepts

- **[[CRI]] (Container Runtime Interface)**: A gRPC-based API standard that Kubernetes uses to talk to runtimes.
- **[[containerd]]**: The default runtime in most modern Kubernetes setups.
- **[[CRI-O]]**: A lightweight runtime purpose-built for Kubernetes.
- **[[Docker Shim]]**: Legacy bridge used to connect Docker to Kubernetes (deprecated in Kubernetes v1.24+).
- **[[RuntimeClass]]**: Kubernetes abstraction for selecting different runtimes per workload.

---

## 🧰 Use Cases

- Running lightweight containers on embedded/edge robotics nodes (e.g., Jetson)
- Executing compute-intensive perception stacks with GPU support
- Separating real-time containers (e.g., navigation) from general workloads
- Testing compatibility of robotic workloads across container runtimes
- Deploying simulation environments alongside live containers

---

## ✅ Pros

- Clean separation of container execution from Kubernetes logic
- Allows swapping runtimes depending on use case (e.g., real-time, GPU)
- Enhances flexibility and portability of robotic applications
- Supports namespaces, cgroups, seccomp, and other isolation tools

---

## ❌ Cons

- Misconfigured runtimes can lead to container failures
- Advanced features (e.g., GPU support) require tuning per runtime
- Docker users may face confusion after shim deprecation
- Requires familiarity with runtime-specific configuration files and tooling

---

## 📊 Comparison Chart

| Runtime      | CRI Compatible | Kubernetes Default | Lightweight | GPU Support | Notes                          |
|--------------|----------------|---------------------|--------------|--------------|--------------------------------|
| [[containerd]]   | ✅ Yes         | ✅ Yes (default)     | ⚠️ Medium     | ✅ With NVIDIA plugin | Actively maintained by CNCF    |
| [[CRI-O]]        | ✅ Yes         | ✅ Yes               | ✅ Very       | ⚠️ Limited        | Designed for Kubernetes         |
| [[Docker]]       | ❌ No (shim)   | ❌ Deprecated        | ⚠️ Heavier    | ✅ Native         | Deprecated after v1.24         |
| [[gVisor]]       | ✅ Partial     | ❌ Not default       | ✅ Secure     | ❌ No           | Focused on sandboxing          |
| [[Kata Containers]] | ✅ Yes     | ❌ Not default       | ⚠️ Medium     | ⚠️ Partial       | Hardware virtualization         |

---

## 🤖 Comparison: Docker vs containerd

| Feature         | Docker                         | containerd                     |
|------------------|----------------------------------|----------------------------------|
| **Focus**         | Developer experience             | Kubernetes integration           |
| **Used With**     | Standalone, Docker Compose       | Kubernetes (via CRI)             |
| **Layered?**      | ✅ Includes CLI, daemon, engine  | ❌ Runtime only                  |
| **ROS2 Deploy?**  | ✅ Easy with Compose             | ✅ Native with kubelet           |

---

## 🔧 Compatible Items

- [[kubelet]] (Uses container runtime to start pods)
- [[Dockerfile]], [[Docker Container]], [[ROS2]]
- NVIDIA Container Toolkit (for GPU workloads)
- RuntimeClass and resource constraints

---

## 🔗 Related Concepts

- [[Kubernetes]] (Manages container runtimes via kubelet)
- [[kubelet]] (Directly communicates with the runtime)
- [[Docker Container]] (Created and managed via runtime)
- [[Kubernetes Pod]] (Executed via runtime)
- [[CI-CD]] (Build container images for runtime use)
- [[GPU Support in Kubernetes]] (Requires runtime plugins)
- [[containerd]]

---

## 🛠 Developer Tools

- `ctr` (CLI for containerd)
- `crictl` (CRI-compatible CLI tool)
- `runc` (Low-level OCI runtime used by containerd)
- `systemctl restart containerd`
- `kubectl get node -o json | jq .status.nodeInfo.containerRuntimeVersion`

---

## 📚 Further Reading

- [Kubernetes Container Runtimes](https://kubernetes.io/docs/setup/production-environment/container-runtimes/)
- [containerd Documentation](https://containerd.io/)
- [CRI-O Project](https://cri-o.io/)
- [NVIDIA GPU Support in Kubernetes](https://docs.nvidia.com/datacenter/cloud-native/kubernetes/overview.html)
- [Kubernetes Deprecates Docker Shim](https://kubernetes.io/blog/2020/12/02/dockershim-faq/)

---

## 🗂 Suggested Folder Location

Software > Tools > Kubernetes  
or  
Software > Robotics > Deployment  
or  
Systems > Cloud and Orchestration
