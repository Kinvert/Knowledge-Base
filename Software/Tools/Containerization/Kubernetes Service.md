# Kubernetes Service

A **Kubernetes Service** is an abstraction that defines a stable network endpoint to access a set of pods. Because pods are ephemeral and can change IP addresses, services provide a consistent method for inter-pod or external communication. In robotics, services enable reliable communication between modules such as mapping, control, and visualization—even as components scale or restart.

---

## 📚 Overview

Services group one or more pods using **labels** and expose them through a single DNS name or IP address. The service handles load balancing, optional port mapping, and cluster-wide accessibility. Kubernetes supports several service types, such as `ClusterIP`, `NodePort`, and `LoadBalancer`, depending on accessibility needs.

---

## 🧠 Core Concepts

- **Selector**: Matches pods using labels to route traffic.
- **ClusterIP**: Default service type, accessible only within the cluster.
- **NodePort**: Exposes a service on a static port across all nodes.
- **LoadBalancer**: Exposes a service via cloud load balancer (cloud environments).
- **Headless Service**: Skips load balancing, often used with StatefulSets or DNS discovery.
- **Endpoints**: Tracks the real-time IPs of pods matched by the service selector.

---

## 🧰 Use Cases

- Accessing a SLAM backend from a front-end UI in a pod
- Routing messages to perception or control subsystems
- Exposing simulation dashboards (e.g., RViz, Gazebo web interfaces)
- Communicating with external robotics hardware or APIs via NodePort
- Providing a stable name to discover multi-instance services (e.g., planners)

---

## ✅ Pros

- Abstracts dynamic pod IPs behind a consistent address
- Supports internal and external load balancing
- Scales easily with pod replicas
- Integrates with DNS (`service-name.namespace.svc.cluster.local`)
- Works seamlessly with `kube-proxy` and other networking layers

---

## ❌ Cons

- NodePort exposes all nodes, may increase attack surface
- ClusterIP isn't accessible outside the cluster by default
- LoadBalancer requires cloud provider or MetalLB setup
- Cannot load balance based on content or headers (use Ingress instead)

---

## 📊 Comparison Chart

| Service Type    | Accessible From       | Use Case                              | Notes                            |
|------------------|------------------------|----------------------------------------|----------------------------------|
| ClusterIP        | Inside cluster only     | Internal microservices                 | Default, DNS-based               |
| NodePort         | External via node port  | Dev testing, simple external access    | Port range 30000-32767          |
| LoadBalancer     | Public IP (cloud)       | Production web services, APIs          | Requires cloud or MetalLB       |
| Headless         | Inside cluster (no IP)  | Stateful workloads, custom discovery   | Returns individual pod IPs      |

---

## 🤖 In a Robotics Context

| Component         | Exposed As           | Example                                 |
|-------------------|----------------------|------------------------------------------|
| SLAM Backend       | ClusterIP             | `/slam_server:8080` inside cluster       |
| Web-based RViz     | NodePort              | Accessed via `http://node-ip:31000`      |
| Control Module     | LoadBalancer (cloud)  | Used in simulation farms                 |
| Sensor Aggregator  | Headless              | Queried directly by stateful clients     |

---

## 🔧 Compatible Items

- [[Kubernetes Pod]], [[kube-proxy]], [[Kubernetes Node]]
- [[Ingress Controller]] for HTTP-level routing
- [[Microservices Architecture]], [[Docker Container]]
- DNS: CoreDNS resolves service names to IPs

---

## 🔗 Related Concepts

- [[Kubernetes Pod]] (Services route to pods)
- [[kube-proxy]] (Implements the routing)
- [[Kubernetes Node]] (Can expose NodePort services)
- [[Ingress Controller]] (Wraps services for web traffic)
- [[Microservices Architecture]] (Service discovery core component)

---

## 🛠 Developer Tools

- `kubectl get services`
- `kubectl expose pod <name> --port=80 --target-port=8080`
- `kubectl port-forward service/<svc> 8080:80`
- `nslookup <service>.<namespace>.svc.cluster.local`
- `curl <service-name>:<port>` from inside cluster

---

## 📚 Further Reading

- [Kubernetes Services Concepts](https://kubernetes.io/docs/concepts/services-networking/service/)
- [CoreDNS and Kubernetes DNS](https://kubernetes.io/docs/concepts/services-networking/dns-pod-service/)
- [Service Types Explained](https://kubernetes.io/docs/concepts/services-networking/service/#publishing-services-service-types)
- [MetalLB for Bare-Metal LoadBalancers](https://metallb.universe.tf/)

---

## 🗂
