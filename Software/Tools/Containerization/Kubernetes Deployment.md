# Kubernetes Deployment

A **Kubernetes Deployment** is a controller that manages the lifecycle of application pods by maintaining a desired state. It automates the creation, updating, and rollback of Pods and ReplicaSets, making it the most common way to deploy applications in Kubernetes clusters.

---

## 📚 Overview

With Deployments, developers define the desired number of replicas, the container image, configuration (like environment variables), and update strategies. The Deployment controller ensures that the cluster always matches this desired state—even if nodes fail or pods crash. In robotics, this is crucial for ensuring high availability of services like perception, SLAM, or control backends.

---

## 🧠 Core Concepts

- **ReplicaSet**: Automatically created by the Deployment to maintain a stable set of running Pods
- **Rolling Update**: Default strategy to apply changes gradually to avoid downtime
- **Rollback**: Revert to a previous working version if a new update fails
- **Declarative YAML**: Define the desired configuration in a `deployment.yaml` file
- **Reconciliation Loop**: Kubernetes constantly ensures actual state matches the desired state

---

## 🧰 Use Cases

- Deploying a sensor fusion service that must always run in 3 replicas
- Rolling out an updated SLAM algorithm without downtime
- Restarting or scaling control nodes dynamically during a mission
- Managing web-based robot dashboards or APIs at scale
- Automatically recovering failed components without manual intervention

---

## ✅ Pros

- Built-in high availability and self-healing
- Easy rolling updates with minimal downtime
- Supports declarative configuration (YAML or JSON)
- Integrated with `kubectl`, Helm, CI/CD systems
- Easily scalable via command line or autoscaling

---

## ❌ Cons

- Not ideal for stateful services (use StatefulSets instead)
- More overhead than simpler approaches (e.g., raw Pods)
- YAML syntax can be verbose or error-prone
- Requires image immutability practices for safe rollbacks

---

## 📊 Comparison Chart

| Feature               | Deployment           | DaemonSet              | StatefulSet            | Job / CronJob             |
|-----------------------|----------------------|-------------------------|------------------------|----------------------------|
| Pod Management        | ReplicaSet           | One per Node           | Fixed identity & order | One-shot / scheduled runs |
| Use Case              | Stateless services   | Node-local agents       | Databases, sensors     | Data processing, logs      |
| Update Strategy       | Rolling updates      | Manual / On Node change| Ordered updates        | N/A                        |
| Volume Persistence    | Shared or ephemeral  | Varies                 | Per-pod persistent     | Ephemeral                  |

---

## 🤖 In a Robotics Context

| Component                | Deployment Use Case                              |
|--------------------------|--------------------------------------------------|
| SLAM Module              | Keep 1–2 replicas alive with rolling updates     |
| Lidar Processing Service | Scale with sensor load, auto-recover on failure |
| ROS2 Web Bridge          | Expose web interface reliably to clients         |
| Perception Stack         | Run parallel image processing pipelines          |
| Dashboard/API Service    | Ensure always-on access to robot telemetry       |

---

## 🔧 Useful Commands (One-Liners)

- `kubectl apply -f deployment.yaml` – Deploy or update resources  
- `kubectl rollout status deployment/my-deployment` – Check rollout progress  
- `kubectl rollout undo deployment/my-deployment` – Roll back to previous version  
- `kubectl scale deployment my-deployment --replicas=5` – Manually scale  
- `kubectl delete deployment my-deployment` – Remove deployment  
- `kubectl get deployments` – List all deployments  
- `kubectl describe deployment my-deployment` – View full configuration and status  

---

## 🔧 Compatible Items

- [[kubectl]] – Primary interface to create and manage Deployments
- [[Kubernetes Pod]] – Managed by the Deployment controller
- [[Helm Chart]] – Often used to template Deployment YAMLs
- [[Docker Container]] – The application unit inside pods
- [[Kubernetes Service]] – Used to expose Deployment-managed pods
- [[CI-CD Pipelines]] – Automate creation/updates of Deployments

---

## 🔗 Related Concepts

- [[Kubernetes Pod]] (Managed as part of a Deployment)
- [[Kubernetes Service]] (Provides stable access to Deployment pods)
- [[kubectl]] (CLI interface to manage Deployments)
- [[Helm Chart]] (Templating for Deployment configs)
- [[Dockerfile]] (Defines the image used in Deployments)
- [[Microservices Architecture]] (Commonly deployed using Deployments)

---

## 📚 Further Reading

- [Official Kubernetes Docs: Deployments](https://kubernetes.io/docs/concepts/workloads/controllers/deployment/)
- [kubectl apply Reference](https://kubernetes.io/docs/reference/generated/kubectl/kubectl-commands#apply)
- [Understanding ReplicaSets](https://kubernetes.io/docs/concepts/workloads/controllers/replicaset/)
- [Helm + Deployment Patterns](https://helm.sh/docs/topics/chart_best_practices/)

---
