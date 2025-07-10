# kubectl

**kubectl** is the command-line interface for interacting with a Kubernetes cluster. It allows users to deploy applications, inspect and manage cluster resources, debug problems, and automate workflows. It's essential for developers and DevOps engineers working with Kubernetes in robotics, cloud infrastructure, and scalable microservices.

---

## 📚 Overview

`kubectl` acts as the bridge between your local terminal and the Kubernetes API server. You can use it to create, inspect, update, delete, and monitor virtually every object in a Kubernetes system—pods, services, deployments, config maps, secrets, and more.

---

## 🧠 Core Concepts

- **kubectl client** communicates with the **Kubernetes API server** defined in your `kubeconfig` file.
- Resources are managed via `kubectl <verb> <resource> <name>`, e.g., `kubectl get pods`
- Commands can work at namespace, cluster, or context levels
- Supports YAML/JSON manifest files for declarative configuration

---

## 🧰 Use Cases

- Deploy robotics workloads into a cloud-based Kubernetes cluster
- Monitor sensor-processing pipelines using pod logs
- Scale up inference services or simulation pods
- Debug malfunctioning nodes or services remotely
- Automate DevOps pipelines with `kubectl` commands

---

## ✅ Pros

- Powerful and flexible control of all cluster components
- Standardized interface for every Kubernetes object
- Works well with scripts and CI/CD systems
- YAML support for configuration as code
- Can manage local or remote clusters with the same syntax

---

## ❌ Cons

- Steep learning curve for new users
- Complex syntax for advanced operations
- Misuse can easily cause disruptions in production environments
- Requires secure access to cluster kubeconfig

---

## 📊 Comparison Chart

| Tool         | Use Case                    | CLI Interface | Works with ROS? | Target User       |
|--------------|-----------------------------|----------------|------------------|-------------------|
| kubectl      | Kubernetes control           | ✅ Yes         | ✅ Indirectly    | DevOps, Developers |
| docker CLI   | Container lifecycle          | ✅ Yes         | ✅ Yes           | Developers         |
| Helm         | Package manager for K8s      | ⚠️ Not primary | ✅ Yes           | DevOps, SREs       |
| ROS2 CLI     | ROS2 topic/service mgmt      | ❌ No          | ✅ Native        | Robotics Devs      |
| k9s          | TUI for Kubernetes           | ✅ Yes         | ✅ Indirectly    | Visual Admins      |

---

## 🧪 Useful Commands (One-Liners)

- `kubectl get pods` – List all pods in the current namespace  
- `kubectl get all -n my-namespace` – Get all resources in a namespace  
- `kubectl describe pod mypod` – Detailed pod info for debugging  
- `kubectl logs mypod` – Get logs from a pod  
- `kubectl exec -it mypod -- bash` – Open an interactive shell inside a pod  
- `kubectl apply -f myfile.yaml` – Apply a config file to create/update resources  
- `kubectl delete -f myfile.yaml` – Delete resources defined in a YAML  
- `kubectl scale deployment myapp --replicas=3` – Scale a deployment  
- `kubectl rollout restart deployment myapp` – Restart a deployment  
- `kubectl port-forward pod/mypod 8080:80` – Forward local port to a pod  

---

## 🤖 In a Robotics Context

| Use Case                              | kubectl Command Example                                  |
|---------------------------------------|-----------------------------------------------------------|
| Launch a SLAM module in K8s           | `kubectl apply -f slam-deployment.yaml`                  |
| Check Lidar processing pod status     | `kubectl get pods -l app=lidar-processor`                |
| Get logs from sensor fusion container | `kubectl logs sensor-fusion-pod`                         |
| Forward dashboard to local machine    | `kubectl port-forward svc/robot-dashboard 8080:80`       |
| Run bash in a robotics service pod    | `kubectl exec -it control-node -- bash`                  |

---

## 🔧 Compatible Items

- [[Kubernetes Pod]] (Managed directly by `kubectl`)
- [[Kubernetes Deployment]] (Declarative app deployment)
- [[Helm Chart]] (Higher-level package manager using `kubectl` under the hood)
- [[Docker Container]] (Used in pods managed via `kubectl`)
- [[ROS2 Web Bridge]] (Deployed as services managed with `kubectl`)
- [[CI-CD Pipelines]] (Scripted `kubectl` for automation)

---

## 🔗 Related Concepts

- [[Kubernetes Control Plane]] (API server responds to `kubectl`)
- [[Kubernetes API Server]] (Main component interfacing with CLI)
- [[Helm Chart]] (Alternative method to deploy resources)
- [[Dockerfile]] (Used to create images consumed by K8s)
- [[Microservices Architecture]] (Often managed via `kubectl`)

---

## 📚 Further Reading

- [kubectl Cheat Sheet (official)](https://kubernetes.io/docs/reference/kubectl/cheatsheet/)
- [Kubernetes API Concepts](https://kubernetes.io/docs/concepts/)
- [kubectl Reference](https://kubernetes.io/docs/reference/generated/kubectl/)
- [Managing Clusters with kubectl](https://kubernetes.io/docs/tasks/tools/)
- [Awesome Kubernetes](https://github.com/ramitsurana/awesome-kubernetes)

---
