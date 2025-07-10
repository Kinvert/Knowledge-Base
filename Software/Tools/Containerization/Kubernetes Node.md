# Kubernetes Node

A **Kubernetes Node** is a physical or virtual machine in a Kubernetes cluster that runs the containerized workloads (pods). Each node is managed by the Kubernetes control plane and includes components like `kubelet`, `kube-proxy`, and a **container runtime**. In robotics, nodes can be powerful servers, embedded Jetson devices, or even remote edge computers hosting specific parts of the robot stack.

---

## 📚 Overview

Nodes are the execution environment for all application containers in a Kubernetes cluster. A healthy node reports its status and resource availability (CPU, memory, etc.) to the control plane. When a workload is scheduled to the node, the `kubelet` ensures it runs properly. Nodes can be homogeneous or heterogeneous and support GPU, FPGA, or real-time OS extensions.

---

## 🧠 Core Concepts

- **kubelet**: Ensures containers are running and healthy.
- **kube-proxy**: Manages network routing rules for pod communication.
- **Container Runtime**: Runs containers (e.g., containerd, CRI-O).
- **Pod Hosting**: Nodes host one or more pods, which are the smallest deployable units.
- **Labels/Taints**: Used to control scheduling behavior or designate special-purpose nodes.

---

## 🧰 Use Cases

- Deploying vision pipelines to Jetson nodes near cameras
- Running SLAM or perception containers close to the robot’s sensors
- Hosting large simulations or ROS2 backends on desktop/server nodes
- Offloading AI inference to GPU-equipped nodes
- Isolating critical workloads using node selectors or affinities

---

## ✅ Pros

- Supports heterogeneity (e.g., x86 + ARM nodes)
- Modular: failures are isolated to individual nodes
- Scalable: easily add/remove nodes from the cluster
- Granular resource tracking and reporting
- Edge-friendly for distributed robotics

---

## ❌ Cons

- Failure of node can result in pod eviction or downtime
- Requires monitoring and log collection at node level
- Networking setup can vary across environments (cloud, local, edge)
- Performance varies based on container runtime and hardware

---

## 📊 Comparison Chart

| Component           | Kubernetes Node              | [[Kubernetes Pod]]                    | [[kubelet]]                        | [[Kubernetes Control Plane]]      |
|---------------------|-------------------------------|----------------------------------|----------------------------------|----------------------------------|
| **Function**         | Hosts containers              | Smallest unit of execution       | Node agent                      | Orchestrates the whole cluster |
| **Scale**            | One or many per cluster       | One or many per node             | One per node                    | One or more (HA setup)         |
| **Location**         | Physical or virtual machine   | Logical on the node              | Inside the node                 | Separate from nodes            |
| **Relevance to ROS2**| Hosts ROS2 containers         | Runs ROS2 nodes                  | Ensures ROS2 nodes stay alive   | Schedules/monitors ROS2 jobs   |

---

## 🖥️ Node Types in Robotics

| Node Type         | Purpose                                  | Example Use Case                      |
|-------------------|-------------------------------------------|----------------------------------------|
| Edge Node         | Near sensors, for low-latency compute     | Jetson running camera feed analysis    |
| Compute Node      | High-performance workloads                | SLAM or deep learning inference        |
| Simulation Node   | Host Gazebo, Isaac Sim, or similar        | Multi-robot scenario playback          |
| Control Node      | Run robot state machines or planners      | ROS2 planners, behavior trees          |

---

## 🔧 Compatible Items

- [[kubelet]], [[kube-proxy]], [[Kubernetes Control Plane]]
- [[Kubernetes Pod]], [[Docker Container]]
- [[Kubernetes Scheduler]], [[Microservices Architecture]]
- Node-affinity rules, GPU support plugins

---

## 🔗 Related Concepts

- [[Kubernetes Pod]] (Runs on a node)
- [[kubelet]] (Manages container lifecycle per node)
- [[Kubernetes Control Plane]] (Assigns pods to nodes)
- [[Docker Container]] (Executed on node)
- [[Microservices Architecture]] (Services run across nodes)

---

## 🛠 Developer Tools

- `kubectl get nodes`
- `kubectl describe node <node-name>`
- `kubectl cordon <node>` – mark node unschedulable
- `kubectl drain <node>` – safely remove all pods
- `journalctl -u kubelet` – view node logs

---

## 📚 Further Reading

- [Kubernetes Node Concepts](https://kubernetes.io/docs/concepts/architecture/nodes/)
- [Managing Nodes in Kubernetes](https://kubernetes.io/docs/concepts/architecture/nodes/#managing-nodes)
- [Kubernetes on Edge Devices](https://kubeedge.io/)
- [ROS2 with Kubernetes](https://micro-ros.github.io/)

---
