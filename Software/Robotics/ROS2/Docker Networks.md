# Docker Networks

**Docker Networks** provide communication channels for Docker containers to interact with each other, the host system, and external networks. Networking is fundamental to containerized applications, allowing isolated or shared communication environments depending on deployment needs.

---

## 📚 Overview

Docker supports multiple network types to fit different use cases: bridge networks for container isolation, host networking for performance, overlay networks for multi-host communication, and macvlan for direct physical network access. Understanding Docker networking is key to architecting scalable, secure, and flexible containerized systems.

---

## 🧠 Core Concepts

- **Bridge Network**: Default isolated network; containers on the same bridge can communicate via IP  
- **Host Network**: Containers share the host’s network stack, no isolation  
- **Overlay Network**: Enables multi-host container communication over encrypted tunnels (used in Docker Swarm and Kubernetes)  
- **Macvlan Network**: Assigns a MAC address to a container, making it appear as a physical device on the network  
- **Network Drivers**: Software components implementing network types and connectivity  
- **DNS and Service Discovery**: Docker’s embedded DNS resolves container names within networks  

---

## 🧰 Use Cases

- Isolating containers in development environments  
- Enabling microservices in different containers to communicate seamlessly  
- Providing high-performance networking for latency-sensitive apps  
- Connecting containers across multiple hosts in a swarm or cluster  
- Integrating containers into existing LANs with macvlan  

---

## ✅ Pros

- Flexible networking options tailored to various scenarios  
- Automatic service discovery and DNS resolution inside networks  
- Secure isolation between container groups  
- Supports scaling container apps across multiple hosts  
- User-defined networks provide fine-grained control  

---

## ❌ Cons

- Overlay networks add latency and complexity  
- Host networking can expose containers to security risks  
- Macvlan requires compatible physical network and switch configuration  
- Network troubleshooting can be challenging in complex setups  

---

## 📊 Comparison Chart of Docker Network Types

| Network Type  | Isolation           | Multi-Host Support | Performance    | Use Case Example                 |
|---------------|---------------------|--------------------|----------------|---------------------------------|
| bridge        | Yes (container group) | No                 | Moderate       | Default network for isolated containers |
| host          | No                  | No                 | High (no overlay) | Single-host, performance-critical containers |
| overlay       | Yes                 | Yes                | Moderate to Low | Multi-host Swarm/K8s cluster networking |
| macvlan       | Partial (physical LAN) | No                | High           | Containers require direct LAN presence |
| none          | Complete isolation  | No                 | N/A            | Containers with no networking (sandboxed) |

---

## 🔧 Useful Commands (One-Liners)

- `docker network ls` – List all Docker networks  
- `docker network inspect <network_name>` – Show details about a network  
- `docker network create --driver bridge my_bridge` – Create a custom bridge network  
- `docker network rm <network_name>` – Remove a Docker network  
- `docker run --network my_bridge ...` – Run a container attached to a specific network  
- `docker network connect <network> <container>` – Attach running container to network  
- `docker network disconnect <network> <container>` – Detach container from network  

---

## 🔧 Compatible Items

- Docker containers (all network features apply to containers)  
- Docker Compose (supports defining multiple networks for services)  
- Docker Swarm (uses overlay networks for service communication)  
- Kubernetes (can integrate with Docker networking or use its own CNI plugins)  

---

## 🔗 Related Concepts

- [[Docker Containers]] (Networking enables container communication)  
- [[Docker Compose]] (Defines networks in multi-container apps)  
- [[Overlay Networks]] (Used in multi-host Docker and Kubernetes clusters)  
- [[Network Namespaces]] (Linux kernel feature underpinning container networking)  
- [[CNI]] (Container Network Interface, alternative to Docker networking in Kubernetes)  

---

## 📚 Further Reading

- [Docker Networking Overview](https://docs.docker.com/network/)  
- [Docker Network Drivers](https://docs.docker.com/network/network-tutorial-overlay/)  
- [Using Macvlan Networks](https://docs.docker.com/network/macvlan/)  
- [Networking in Docker Swarm](https://docs.docker.com/engine/swarm/networking/)  
- [Troubleshooting Docker Networks](https://docs.docker.com/network/troubleshoot/)  

---
