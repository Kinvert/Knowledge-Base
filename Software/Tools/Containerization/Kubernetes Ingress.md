# Kubernetes Ingress

A **Kubernetes Ingress** is an API object that manages external access to services in a Kubernetes cluster, typically HTTP and HTTPS traffic. It provides fine-grained routing rules, SSL termination, and load balancing, enabling seamless and secure exposure of multiple services through a single external IP or hostname.

---

## 📚 Overview

Ingress acts as a smart gateway for external users or systems to reach internal Kubernetes services. Unlike Services (which provide cluster-internal routing or simple external exposure via NodePort/LoadBalancer), Ingress supports advanced traffic control such as path-based routing, virtual hosts, and TLS termination. This is particularly useful in robotics deployments with web dashboards, REST APIs, or multi-service microservices architectures.

---

## 🧠 Core Concepts

- **Ingress Resource**: Declarative routing rules (host, path, TLS).
- **Ingress Controller**: The implementation that watches Ingress resources and configures the underlying proxy (e.g., NGINX, Traefik, HAProxy).
- **TLS Termination**: Offloads SSL encryption/decryption at the ingress.
- **Path-based Routing**: Routes requests based on URL paths.
- **Host-based Routing**: Routes requests based on hostname or domain.
- **Backend Services**: Target Kubernetes Services that receive traffic.

---

## 🧰 Use Cases

- Expose multiple robotics microservices through a single external IP.
- Secure web-based monitoring dashboards (e.g., RViz web, simulation GUIs).
- Enable HTTPS with automatic certificate provisioning (e.g., via cert-manager).
- Route traffic for different robot fleets or environments via hostnames or paths.
- Implement blue-green or canary deployments by controlling traffic split.

---

## ✅ Pros

- Centralizes external access management.
- Enables HTTPS/TLS with easy certificate management.
- Supports complex routing (paths, hosts).
- Reduces need for multiple LoadBalancer IPs.
- Integrates with cloud provider or custom ingress controllers.

---

## ❌ Cons

- Requires deployment and management of an ingress controller.
- Can add latency and complexity in traffic flow.
- Configuration syntax can be complex for advanced rules.
- Not suitable for non-HTTP protocols without specialized controllers.

---

## 📊 Comparison Chart

| Feature            | Kubernetes Ingress      | Service (LoadBalancer)    | Service (NodePort)      | API Gateway               |
|--------------------|------------------------|---------------------------|-------------------------|---------------------------|
| Protocol Support   | Primarily HTTP/HTTPS    | TCP/UDP                   | TCP/UDP                 | HTTP/HTTPS + more         |
| Load Balancing     | Yes                    | Yes                       | Yes                     | Advanced routing          |
| TLS Termination    | Yes                    | Limited                   | No                      | Yes                       |
| Routing Rules      | Host/Path based         | None                      | None                    | Extensive                 |
| External IPs       | Single IP per ingress   | One per service           | Uses node IP + port     | Single or multiple IPs    |
| Complexity         | Medium-High            | Low                       | Low                     | High                      |

---

## 🤖 In Robotics Context

| Scenario                | Use of Ingress                                            |
|-------------------------|----------------------------------------------------------|
| Web-based control UI     | Route /control to robot control service                   |
| Telemetry dashboards    | Route /telemetry to monitoring services                   |
| Fleet management        | Host multiple fleet namespaces on different domains       |
| Simulation portals      | Provide HTTPS access to Gazebo or Webots dashboards       |

---

## 🔧 Compatible Items

- [[Kubernetes Service]], [[Kubernetes Pod]], [[Kubernetes Node]]
- [[Helm Chart]] for ingress controller deployment
- [[cert-manager]] for automated TLS certificates
- [[Microservices Architecture]], [[Docker Container]]

---

## 🔗 Related Concepts

- [[Kubernetes Service]] (Ingress routes traffic to Services)
- [[Load Balancer]] (Ingress often backed by LB)
- [[Helm Chart]] (Deploy ingress controllers)
- [[Microservices Architecture]] (Centralized routing)
- [[Docker Container]] (Ingress routes to containerized apps)

---

## 🛠 Developer Tools

- `kubectl get ingress`
- `kubectl describe ingress <name>`
- Deploy ingress controllers: e.g., `helm install nginx-ingress ingress-nginx/ingress-nginx`
- Use `cert-manager` to automate TLS provisioning
- Debug with logs from ingress controller pods

---

## 📚 Further Reading

- [Kubernetes Ingress Concepts](https://kubernetes.io/docs/concepts/services-networking/ingress/)
- [Ingress Controllers Overview](https://kubernetes.io/docs/concepts/services-networking/ingress-controllers/)
- [NGINX Ingress Controller](https://kubernetes.github.io/ingress-nginx/)
- [Cert-Manager for TLS](https://cert-manager.io/docs/)
- [Using Ingress with ROS2 Web Services](https://micro-ros.github.io/docs/tutorials/core/advanced_networking/)

---
