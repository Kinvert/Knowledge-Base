# kubelet

The `kubelet` is a core component of the Kubernetes **worker node** responsible for managing pods and their containers. It acts as an agent that runs on each node and ensures that the containers described in PodSpecs are running and healthy. In robotics deployments, `kubelet` is crucial for running distributed ROS2 nodes, simulators, and monitoring tasks on edge devices or embedded systems.

---

## 📚 Overview

The `kubelet` listens to the Kubernetes API Server for pod assignments and works with the container runtime (e.g., Docker or containerd) to start, stop, and monitor containers. It also reports node and pod status back to the control plane and handles logging and resource usage.

---

## 🧠 Core Concepts

- **Pod Lifecycle Management**: Ensures containers run as declared in the spec.
- **Node Agent**: Registers the node with the control plane.
- **Health Monitoring**: Reports pod and node status using probes (`liveness`, `readiness`).
- **Resource Enforcement**: Enforces CPU/memory limits via cgroups.
- **Volume Management**: Mounts requested volumes for pods.

---

## 🧰 Use Cases

- Running ROS2 workloads on edge nodes or robots
- Monitoring health of deployed vision, SLAM, or navigation services
- Applying CPU/GPU constraints for real-time performance
- Managing lifecycle of sensor data acquisition pods
- Enabling ROS2 nodes to autoscale via resource metrics

---

## ✅ Pros

- Lightweight and efficient node-level control
- Integrates cleanly with diverse container runtimes
- Automatic recovery and restart of failed pods
- Enforces system-level isolation and constraints
- Logs and metrics useful for debugging robotic services

---

## ❌ Cons

- Failure can lead to loss of pod control on that node
- Misconfigured probes or resources can cause false failures
- Requires tuning for real-time robotics or embedded use
- Can't be used standalone (relies on control plane)

---

## 📊 Comparison Chart

| Component     | kubelet                         | [[containerd]]                     | [[kube-apiserver]]                  | [[ROS2 Node]]               |
|---------------|----------------------------------|----------------------------------|----------------------------------|--------------------------|
| **Role**      | Manages pods on a node           | Runs containers at low level     | Manages entire cluster           | Runs within container    |
| **Location**  | Worker node                      | Worker node                     | Control plane node               | Inside pod               |
| **Interacts With** | API server, container runtime | Kubernetes, Docker               | Developers via `kubectl`         | Other ROS2 nodes         |
| **Failure Impact** | Local pod failure/recovery    | Container crash                  | Cluster-wide failure             | Single component failure |

---

## 🤖 Comparison: kubelet vs kube-proxy

| Feature             | kubelet                        | [[kube-proxy]]                      |
|---------------------|----------------------------------|----------------------------------|
| **Main Function**   | Manages containers/pods         | Manages network routing/rules   |
| **Operates On**     | Local node resources            | iptables or IPVS rules           |
| **Related To**      | Runtime state, pod health       | Cluster service networking       |
| **ROS2 Context**    | Keeps nodes alive               | Routes services (e.g., DDS ports)|

---

## 🔧 Compatible Items

- [[Kubernetes]], [[Kubernetes Pod]]
- [[Docker Container]], [[Dockerfile]], [[ROS2]]
- Container runtimes (containerd, CRI-O)
- `kubectl` commands for monitoring

---

## 🔗 Related Concepts

- [[Kubernetes]] (Orchestrates node-level kubelets)
- [[Kubernetes Pod]] (Scheduled and monitored by kubelet)
- [[Docker Container]] (Managed on node by kubelet)
- [[ROS2]] (Runs inside containers supervised by kubelet)
- [[Kubernetes Control Plane]] (Assigns pods to nodes via kubelet)
- [[CI-CD Pipelines]] (Deploy workloads monitored by kubelet)

---

## 🛠 Developer Tools

- `systemctl status kubelet` — check kubelet status
- `journalctl -u kubelet` — view kubelet logs
- `kubectl describe node <node-name>` — view kubelet resource view
- Configure via `/var/lib/kubelet/config.yaml`

---

## 📚 Further Reading

- [kubelet Documentation](https://kubernetes.io/docs/reference/command-line-tools-reference/kubelet/)
- [Understanding Kubernetes Node Architecture](https://kubernetes.io/docs/concepts/architecture/nodes/)
- [Debugging kubelet Failures](https://learnk8s.io/debugging-kubelet)
- [Edge Robotics with KubeEdge and kubelet](https://kubeedge.io)

---
