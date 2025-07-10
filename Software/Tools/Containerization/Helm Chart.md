# Helm Chart

A **Helm Chart** is a package format used by Helm, the Kubernetes package manager. It defines a set of Kubernetes resources—such as Deployments, Services, ConfigMaps, and more—in a structured, templated way. Helm charts are used to simplify the deployment and management of complex applications, including robotics stacks, simulation clusters, or ROS2-based microservices.

---

## 📚 Overview

Helm abstracts Kubernetes manifests into reusable templates, allowing parameterization and versioning. Charts can be shared via Helm repositories, making it easier to install, upgrade, and rollback applications in Kubernetes environments. This is especially useful in robotics, where multi-node configurations (e.g., SLAM + planning + control) must be deployed consistently across devices or test environments.

---

## 🧠 Core Concepts

- **Chart**: A collection of files that describe a related set of Kubernetes resources.
- **Values File**: A YAML file that provides configurable inputs to chart templates.
- **Templates**: Kubernetes manifests with embedded Go templating syntax.
- **Release**: A specific deployment of a chart into a namespace.
- **Repository**: A collection of packaged charts that can be downloaded and deployed.

---

## 🧰 Use Cases

- Deploying a full ROS2-based application stack with one command
- Installing simulation tools (e.g., Gazebo, Webots, Ignition) into a test cluster
- Managing robotic microservices like control, mapping, and monitoring
- Rolling back to previous versions of a software stack
- Parameterizing deployments for different robot models or environments

---

## ✅ Pros

- Simplifies deployment of complex apps
- Reusable and shareable across teams and clusters
- Supports versioning, upgrades, and rollbacks
- Parameterized with environment-specific values
- Excellent for CI/CD integration in robotics projects

---

## ❌ Cons

- Template logic can become complex and hard to debug
- Requires understanding of Helm’s templating language (Go templates)
- Debugging issues may involve multiple layers (Helm → YAML → Kubernetes)
- Can mask underlying Kubernetes concepts for beginners

---

## 📊 Comparison Chart

| Feature         | Helm Chart                 | Raw YAML Manifests          | [[Kustomize]]                  | Operator                    |
|------------------|-----------------------------|------------------------------|----------------------------|-----------------------------|
| Parameterization | ✅ Yes (values.yaml)         | ❌ No                         | ✅ Yes (patches/overlays)   | ✅ Yes (via CRDs)           |
| Reusability      | ✅ High                      | ❌ Low                        | ⚠️ Medium                  | ✅ Yes                      |
| Rollbacks        | ✅ Built-in                  | ❌ Manual                     | ❌ Manual                   | ✅ Custom logic             |
| Learning Curve   | ⚠️ Moderate (templates)      | ✅ Low                        | ⚠️ Moderate                | ⚠️ High (custom code)       |
| Best Use Case    | Complex, repeatable apps     | Simple or one-off configs    | Layered dev/test/prod setups| Domain-specific logic       |

---

## 🤖 In a Robotics Context

| Component         | Helm Use Case                             |
|-------------------|-------------------------------------------|
| SLAM System        | Template SLAM + map server + viz stack    |
| Simulation System  | Deploy entire Gazebo + bridge services    |
| Sensor Interface   | Parameterize camera or LIDAR topics       |
| Fleet Manager      | Manage namespaces per robot               |
| Behavior Planner   | Roll out versioned logic to field robots  |

---

## 🔧 Compatible Items

- [[Kubernetes]], [[Kubernetes Service]], [[Kubernetes Pod]]
- [[CI-CD Pipelines]], [[Microservices Architecture]]
- [[Docker Container]]
- Templating engines (Go templates via Helm)

---

## 🔗 Related Concepts

- [[Kubernetes]] (Helm deploys resources to the cluster)
- [[CI-CD Pipelines]] (Charts used in automated deployments)
- [[Kubernetes Service]] (Declared in Helm templates)
- [[ROS2 Node]] (Robot nodes deployed as Helm-managed workloads)
- [[Microservices Architecture]] (Bundles service orchestration logic)
- [[Package Managers]]

---

## 🛠 Developer Tools

- `helm create <chart-name>` – scaffold a new chart
- `helm install <release> ./chart/` – deploy chart
- `helm upgrade --install` – upgrade or install if not present
- `helm lint` – validate chart structure
- `helm template` – render chart to raw YAML

---

## 📚 Further Reading

- [Helm Official Documentation](https://helm.sh/docs/)
- [Awesome Helm Charts](https://artifacthub.io/)
- [Deploying ROS with Helm](https://github.com/ros-industrial/ros-industrial-ci)
- [Helm vs Kustomize Comparison](https://helm.sh/docs/topics/alternatives/)
- [Go Templating Basics](https://golangdocs.com/templates)

---
