# etcd

**etcd** is a distributed, reliable key-value store used by Kubernetes and other systems to persist critical configuration and state data. In Kubernetes, it is the **single source of truth** for the entire cluster’s state. In robotics or simulation environments, `etcd` ensures that deployments, resource states, and service discovery mechanisms are reliable and consistent—even across reboots or failures.

---

## 📚 Overview

`etcd` is designed for **high availability**, **strong consistency**, and **speed**. It uses the **Raft consensus algorithm** to replicate data across multiple nodes and provides a consistent snapshot of system state at all times. Any updates to cluster objects—like creating a new pod, updating a deployment, or modifying configuration—are stored in `etcd`.

---

## 🧠 Core Concepts

- **Key-Value Store**: All data in `etcd` is stored as key-value pairs, often JSON-encoded.
- **Watch Mechanism**: Allows clients (e.g., API server) to subscribe to changes.
- **Snapshotting & Backup**: Supports periodic snapshots for disaster recovery.
- **Raft Consensus**: Ensures state agreement across a cluster of `etcd` nodes.
- **Namespaces & TTLs**: Supports hierarchical keys and leases with expiration.

---

## 🧰 Use Cases

- Storing full Kubernetes cluster state: Pods, ConfigMaps, Secrets, etc.
- Providing state sync for multi-robot coordination (via custom etcd use)
- Tracking distributed sensor metadata in autonomous vehicle fleets
- Storing ROS2 node configurations or robot fleet topology centrally
- Powering leader election or service discovery in microservice-based robotics

---

## ✅ Pros

- Strong consistency guarantees via Raft
- Fast read and write performance
- Built-in clustering for high availability
- Supports watch/subscription model for real-time updates
- Ideal for Kubernetes and other distributed systems

---

## ❌ Cons

- Critical single point of failure if not replicated correctly
- Requires secure access control and TLS configuration
- Manual backup and disaster recovery setup required
- Can become corrupted if disk or memory is unstable
- Not suitable for high-volume, non-critical telemetry

---

## 📊 Comparison Chart

| Feature            | etcd                        | [[Redis]]                        | [[Consul]]                    | [[ZooKeeper]]                  |
|--------------------|------------------------------|-------------------------------|----------------------------|-----------------------------|
| Use Case           | Cluster state/configuration  | Caching, queues, pub/sub      | Service discovery, config  | Coordination, discovery     |
| Strong Consistency | ✅ Yes                       | ❌ Eventual (default)         | ⚠️ Partial                 | ✅ Yes                      |
| Watch Support      | ✅ Yes                       | ⚠️ Limited                    | ✅ Yes                     | ✅ Yes                      |
| Used in Kubernetes | ✅ Yes (core dependency)      | ❌ No                          | ❌ No                      | ❌ No                       |
| Storage Model      | Key-Value (Raft)             | Key-Value (non-consensus)     | Key-Value + health checks | Hierarchical + ZAB         |

---

## 🤖 Comparison: etcd vs ConfigMap

| Feature          | etcd                         | [[Kubernetes ConfigMap]]          |
|------------------|-------------------------------|--------------------------------|
| **Scope**         | Internal cluster state        | Application config             |
| **Persistence**   | Required for all cluster ops  | Optional, reloadable           |
| **Used By**       | API server and controllers    | Pods and workloads             |
| **Accessed How**  | Only via Kubernetes APIs      | Mounted or env-injected        |

---

## 🔧 Compatible Items

- [[Kubernetes API Server]], [[kubelet]], [[Kubernetes Control Plane]]
- [[Microservices Architecture]], [[CI-CD Pipelines]]
- Kubernetes etcd backup tools (`etcdctl`, `velero`)
- ROS2 deployments storing metadata externally

---

## 🔗 Related Concepts

- [[Kubernetes Control Plane]] (Reads/writes to etcd)
- [[Kubernetes API Server]] (etcd is its backing store)
- [[kubelet]] (Gets node info via API server backed by etcd)
- [[Microservices Architecture]] (etcd used for coordination or config)
- [[CI-CD Pipelines]] (Stateful deployment relies on accurate etcd state)

---

## 🛠 Developer Tools

- `etcdctl get <key>` – query stored keys
- `etcdctl snapshot save backup.db` – create backup
- `etcdctl member list` – inspect cluster members
- `journalctl -u etcd` – check logs on systemd-based systems
- `kubectl get events` – errors often trace back to etcd issues

---

## 📚 Further Reading

- [Official etcd Docs](https://etcd.io/docs/)
- [etcd in Kubernetes](https://kubernetes.io/docs/tasks/administer-cluster/configure-upgrade-etcd/)
- [Raft Protocol Overview](https://raft.github.io/)
- [Disaster Recovery for etcd](https://kubernetes.io/docs/tasks/administer-cluster/configure-upgrade-etcd/#disaster-recovery)

---
