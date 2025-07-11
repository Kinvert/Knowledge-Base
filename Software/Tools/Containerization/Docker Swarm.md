# Docker Swarm

**Docker Swarm** is Docker’s native clustering and orchestration tool that enables managing a group of Docker hosts as a single virtual system. It facilitates deploying, scaling, and managing containerized applications across multiple machines with built-in load balancing and service discovery.

---

## 📚 Overview

Docker Swarm transforms a pool of Docker engines into a cluster of nodes (managers and workers) to orchestrate container deployment and lifecycle. It offers an easy-to-use, integrated solution for container orchestration directly within Docker CLI, making it a simpler alternative to more complex orchestrators like Kubernetes.

---

## 🧠 Core Concepts

- **Swarm Cluster**: A set of Docker nodes managed as one  
- **Manager Nodes**: Control and maintain cluster state, schedule tasks  
- **Worker Nodes**: Execute containers as directed by managers  
- **Services**: Declarative definitions of containerized applications with replicas  
- **Tasks**: Individual container instances running as part of a service  
- **Load Balancing**: Built-in routing mesh distributes requests to containers  
- **Overlay Networks**: Enable multi-host container communication inside the swarm  

---

## 🧰 Use Cases

- Deploying scalable microservices across multiple hosts  
- High availability container orchestration with automatic failover  
- Simplified container cluster management for small to medium-sized setups  
- Rapidly scaling services based on demand  
- Continuous deployment pipelines using Docker-native tooling  

---

## ✅ Pros

- Native Docker integration with familiar CLI and API  
- Simple setup compared to Kubernetes  
- Built-in service discovery and load balancing  
- Supports rolling updates and rollbacks  
- Overlay networking for multi-host container communication  

---

## ❌ Cons

- Limited ecosystem and community compared to Kubernetes  
- Less flexible and feature-rich for complex workloads  
- Scaling beyond hundreds of nodes can be challenging  
- No support for custom resource definitions or advanced scheduling policies  
- Less cloud-provider integration  

---

## 📊 Comparison Chart: Docker Swarm vs Kubernetes

| Feature                 | Docker Swarm       | Kubernetes           |
|-------------------------|--------------------|----------------------|
| Complexity              | Low                | High                 |
| Setup Time             | Minutes            | Hours to days        |
| Scalability            | Moderate (up to 100s nodes) | Very High (thousands of nodes) |
| Ecosystem & Extensibility | Limited            | Extensive            |
| Load Balancing         | Built-in routing mesh | Ingress & Service mesh |
| Rolling Updates & Rollbacks | Yes             | Yes                  |
| Network Model          | Overlay Networks   | CNI Plugins           |
| Cloud Provider Support | Limited            | Extensive             |
| CLI & API              | Docker CLI & API    | kubectl & APIs       |

---

## 🔧 Useful Commands (One-Liners)

- `docker swarm init` – Initialize a new swarm cluster (manager node)  
- `docker swarm join --token <token> <manager_ip>:2377` – Join a worker node  
- `docker service create --replicas 3 --name myservice nginx` – Deploy a service with replicas  
- `docker service ls` – List running services  
- `docker node ls` – List nodes in the swarm  
- `docker service update --image nginx:1.19 myservice` – Update service image (rolling update)  
- `docker service rollback myservice` – Roll back service update  
- `docker stack deploy -c docker-compose.yml mystack` – Deploy stack from compose file  

---

## 🔧 Compatible Items

- [[Docker Networks]] – Overlay networks enable multi-host communication  
- [[Docker Containers]] – Swarm schedules containers as service tasks  
- [[Docker Compose]] – Compose files can be deployed as stacks in Swarm  
- [[Docker Volumes]] – Persist data across service replicas  
- [[Docker Registry]] – Pull container images for swarm services  

---

## 🔗 Related Concepts

- [[Docker]] (Core container platform for Swarm)  
- [[Kubernetes]] (More feature-rich orchestration alternative)  
- [[Docker Compose]] (Defines multi-container apps, deployable in Swarm stacks)  
- [[Overlay Networks]] (Network backbone for swarm services)  
- [[Load Balancing]] (Routing mesh provides built-in load distribution)  

---

## 📚 Further Reading

- [Docker Swarm Official Documentation](https://docs.docker.com/engine/swarm/)  
- [Docker Swarm Mode Overview](https://docs.docker.com/engine/swarm/swarm-mode/)  
- [Deploying Services in Docker Swarm](https://docs.docker.com/engine/swarm/how-swarm-mode-works/swarm-task-states/)  
- [Comparison: Docker Swarm vs Kubernetes](https://www.mirantis.com/blog/docker-swarm-vs-kubernetes/)  

---
