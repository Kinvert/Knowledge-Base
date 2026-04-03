# Helm

**Helm** is a package manager for Kubernetes that simplifies the deployment and management of applications on Kubernetes clusters. Helm uses “charts” — pre-configured Kubernetes resource templates — to deploy, upgrade, and version applications in a repeatable and manageable way.

---

## 📚 Overview

Helm streamlines Kubernetes application management by packaging all necessary resources (Deployments, Services, ConfigMaps, etc.) into charts. It supports templating, versioning, rollbacks, and dependency management, making complex Kubernetes deployments accessible and maintainable. In robotics, Helm can be used to deploy cloud-native robotics middleware, simulation platforms, or orchestration layers.

---

## 🧠 Core Concepts

- **Chart**: A packaged collection of Kubernetes manifests and templates
- **Release**: An instance of a chart deployed to a Kubernetes cluster
- **Repository**: Remote storage of charts (e.g., Helm Hub, private repos)
- **Values**: Configuration data supplied to templates at install/upgrade
- **Templating Engine**: Uses Go templates to customize resource definitions dynamically

---

## 🧰 Use Cases

- Deploying ROS2 middleware and microservices in Kubernetes clusters
- Managing cloud simulators or visualization platforms like Foxglove
- Automating updates and rollbacks of robotics platform components
- Configuring multi-tenant robotics platforms with different parameters
- Sharing reusable Kubernetes application blueprints across teams

---

## ✅ Pros

- Simplifies complex Kubernetes deployments with templating
- Supports version control and easy rollbacks of releases
- Handles dependencies between charts
- Large ecosystem of community and official charts
- Integrates well with CI/CD pipelines for automated deployments

---

## ❌ Cons

- Learning curve for Helm templating language and Kubernetes concepts
- Debugging templated manifests can be challenging
- Overhead for very simple deployments
- Chart maintenance can become complex for large projects

---

## 📊 Comparison Chart

| Feature                  | Helm               | Kustomize         | kubectl (raw YAML) | Terraform          | Ansible (K8s)      |
|--------------------------|--------------------|-------------------|--------------------|--------------------|--------------------|
| Template Support         | ✅ Yes (Go templates) | ⚠️ Patch overlays  | ❌ No               | ✅ Yes             | ✅ Yes             |
| Package Management       | ✅ Charts & repos   | ❌ No              | ❌ No               | ✅ Modules         | ✅ Roles            |
| Versioning & Rollbacks   | ✅ Built-in         | ❌ No              | ❌ No               | ✅ Yes             | ⚠️ Partial          |
| Declarative             | ✅                  | ✅                 | ✅                  | ✅                  | ✅                  |
| Integration with CI/CD   | ✅                  | ✅                 | ✅                  | ✅                  | ✅                  |

---

## 🤖 In a Robotics Context

| Task                                   | Helm Utility                                  |
|----------------------------------------|------------------------------------------------|
| Deploying ROS2 microservices           | Manage multiple ROS2 nodes as Helm releases     |
| Setting up cloud simulation environments | Automate simulator deployment with Helm         |
| Version-controlled multi-node setups   | Easy upgrades and rollbacks during testing      |
| Sharing reusable platform configs      | Distribute Helm charts for standardized setups  |
| Continuous deployment pipelines        | Integrate Helm with Jenkins, GitHub Actions     |

---

## 🔧 Useful Commands (One-Liners)

- `helm repo add stable https://charts.helm.sh/stable` – Add a chart repo  
- `helm search repo nginx` – Search for charts in repos  
- `helm install myapp stable/nginx` – Install a chart as a release  
- `helm upgrade myapp stable/nginx` – Upgrade a release  
- `helm rollback myapp 1` – Rollback to previous release version  
- `helm list` – List deployed releases  
- `helm template ./mychart` – Render chart templates locally  

---

## 🔧 Compatible Items

- [[Kubernetes Deployment]] – Helm installs and manages deployments  
- [[Kubernetes Service]] – Charts define services alongside deployments  
- [[kubectl]] – Helm uses `kubectl` under the hood for Kubernetes operations  
- [[CI-CD]] – Helm automates deployments in pipelines  
- [[Docker]] – Helm deploys Dockerized containers to clusters  
- [[Kubernetes Ingress]] – Managed by Helm charts for routing  

---

## 🔗 Related Concepts

- [[Kubernetes]] (Helm is native to Kubernetes app management)  
- [[kubectl]] (Command-line client Helm complements)  
- [[CI-CD]] (Automate Helm chart deployment)  
- [[Docker]] (Container images deployed via Helm)  
- [[Kustomize]] (Alternative Kubernetes configuration tool)  
- [[Terraform]] (Infrastructure as code for Kubernetes and beyond)  

---

## 📚 Further Reading

- [Helm Official Website](https://helm.sh/)  
- [Helm Documentation](https://helm.sh/docs/)  
- [Creating Helm Charts](https://helm.sh/docs/topics/charts/)  
- [Using Helm in CI/CD](https://helm.sh/docs/howto/charts_tips_and_tricks/)  
- [Helm vs Kustomize Comparison](https://www.weave.works/blog/helm-vs-kustomize)

---
