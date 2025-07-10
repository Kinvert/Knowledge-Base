# kube-proxy

`kube-proxy` is a network component of Kubernetes that runs on each node and maintains network rules to enable communication between services and pods. It manages virtual IPs (ClusterIPs), load balancing, and routing, making service discovery and pod-to-pod communication seamless—even in complex, multi-host robotics deployments.

---

## 📚 Overview

`kube-proxy` enables the Kubernetes **Service abstraction** by implementing network rules via `iptables`, `ipvs`, or user-space proxies. These rules allow traffic to be routed to the appropriate backend pods. For robotics systems using ROS2, simulation clusters, or sensor data processing graphs, `kube-proxy` ensures that distributed nodes can discover and talk to each other.

---

## 🧠 Core Concepts

- **Service Discovery**: Routes traffic from a ClusterIP to available backend pods.
- **Load Balancing**: Distributes traffic across multiple pod instances of a service.
- **iptables/ipvs**: Implements forwarding rules using native Linux networking features.
- **Endpoints**: List of pod IPs backing a given service.
- **NodePort**: Exposes a service on a static port on each node for external access.

---

## 🧰 Use Cases

- Enabling ROS2 services and nodes to talk over DDS across multiple pods
- Routing messages between sensor nodes and SLAM backends in separate pods
- Balancing simulation service traffic (e.g., Web UIs or loggers)
- Supporting multi-robot deployments in the same Kubernetes cluster
- Exposing data ingestion endpoints from robots to backend processing services

---

## ✅ Pros

- Automates internal service discovery and routing
- Supports various backends: iptables, IPVS, user space
- Minimal configuration required for internal communication
- Works with all standard Kubernetes Services (ClusterIP, NodePort, LoadBalancer)

---

## ❌ Cons

- Latency overhead compared to direct pod-to-pod communication
- Not suitable for high-performance UDP by default
- Troubleshooting network issues requires deep knowledge of Linux networking
- Load balancing is session-unaware (may cause instability for long-lived ROS2 connections)

---

## 📊 Comparison Chart

| Feature                  | kube-proxy            | CoreDNS              | Ingress Controller      | Service Mesh (e.g. Istio) |
|--------------------------|------------------------|-----------------------|--------------------------|----------------------------|
| **Primary Function**     | Pod/service routing    | DNS resolution        | HTTP/HTTPS ingress       | L7 traffic control         |
| **Runs On**              | Every node             | Cluster-wide pod      | Edge/gateway pod         | Sidecar + control plane    |
| **Works With**           | ClusterIP, NodePort    | Service names         | LoadBalancer + TLS       | Pods and services          |
| **Best For**             | TCP/UDP load balancing | Service discovery     | External web traffic     | Telemetry + traffic shaping|

---

## 🤖 Comparison: kube-proxy vs kubelet

| Feature                | kube-proxy                      | kubelet                        |
|------------------------|----------------------------------|----------------------------------|
| **Role**               | Networking / traffic routing     | Pod lifecycle management        |
| **Runs On**            | Every worker node                | Every worker node               |
| **Configures**         | iptables/ipvs rules              | Container runtime               |
| **ROS2 Impact**        | Enables intra-cluster discovery  | Runs ROS2 node containers       |

---

## 🔧 Compatible Items

- [[Kubernetes Pod]], [[Kubernetes Service]]
- [[kubelet]]
- Linux `iptables`, `ipvsadm`
- Cluster DNS via [[CoreDNS]]

---

## 🔗 Related Concepts

- [[Kubernetes]] (Networking managed via kube-proxy)
- [[Kubernetes Pod]] (Traffic routed to/from)
- [[Kubernetes Control Plane]] (Informs kube-proxy about endpoint changes)
- [[Ingress Controller]] (Works alongside kube-proxy for external traffic)
- [[Docker Container]] (Container-level routing enabled)

---

## 🛠 Developer Tools

- `kubectl get services`
- `iptables -L -t nat` (view routing rules)
- `kubectl get endpoints`
- `systemctl status kube-proxy`
- View logs: `journalctl -u kube-proxy`

---

## 📚 Further Reading

- [Kubernetes Networking Concepts](https://kubernetes.io/docs/concepts/cluster-administration/networking/)
- [kube-proxy Documentation](https://kubernetes.io/docs/reference/command-line-tools-reference/kube-proxy/)
- [iptables vs ipvs in kube-proxy](https://kubernetes.io/blog/2018/11/07/why-we-created-a-new-proxy-mode-in-kubernetes/)

---
