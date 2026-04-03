# Jenkins

**Jenkins** is an open-source automation server used to build, test, and deploy software. It is a widely adopted tool in DevOps and CI/CD pipelines and is known for its flexibility, plugin ecosystem, and ability to integrate with nearly any system. Jenkins is often used in robotics and engineering workflows to automate testing, container builds, and deployments for projects like ROS2 packages or embedded firmware.

---

## 📚 Overview

Jenkins provides a web interface and runs on Java. It allows users to create **pipelines** for automating complex workflows. Jobs can be configured through the GUI or via code using **Jenkinsfiles**, which define pipeline stages (e.g., build, test, deploy) in a declarative format.

It supports a vast library of plugins for integrating with GitHub, Docker, Kubernetes, Slack, and many other tools.

---

## 🧠 Core Concepts

- **Job**: A unit of work such as a build or test
- **Pipeline**: A sequence of stages defined in a `Jenkinsfile`
- **Node/Agent**: A machine that runs Jenkins jobs (can be local or remote)
- **Executor**: A slot on a node that runs one job at a time
- **Plugin**: Extends Jenkins functionality (e.g., Docker plugin, Git plugin)
- **Declarative vs Scripted Pipeline**: Two syntax styles for defining jobs

---

## 🧰 Use Cases

- CI/CD for robotics projects (e.g., ROS2, PX4, OpenCV)
- Building and testing Docker containers
- Running simulations as part of test suites
- Deploying to Kubernetes or embedded targets
- Monitoring hardware-in-the-loop (HIL) tests
- Scheduling nightly or regression builds
- Automated firmware builds for microcontrollers

---

## ✅ Pros

- Extremely flexible and customizable
- Vast plugin ecosystem
- Integrates with almost any system or protocol
- Supports both GUI-driven and code-driven pipelines
- Good choice for private or air-gapped infrastructure

---

## ❌ Cons

- Can be complex and heavy to set up and maintain
- UI can be unintuitive compared to modern alternatives
- Plugins may be outdated or poorly maintained
- Scaling Jenkins is more difficult than cloud-native CI tools
- Less seamless GitHub integration compared to GitHub Actions

---

## 📊 Comparison Chart

| Feature                | Jenkins             | GitHub Actions       | GitLab CI/CD         | CircleCI             | Travis CI            |
|------------------------|---------------------|------------------------|-----------------------|----------------------|----------------------|
| Hosting Type           | Self-hosted         | GitHub cloud-native    | Cloud/self-hosted     | Cloud-native         | Cloud-native         |
| Extensibility          | ✅ Huge plugin base | ⚠️ Moderate           | ✅ Built-in templates | ✅ Some plugins       | ⚠️ Limited            |
| Built-in GitHub UX     | ❌ No               | ✅ Tight integration   | ⚠️ Partial            | ✅ Yes               | ✅ Yes               |
| Declarative Pipelines  | ✅ Jenkinsfile       | ✅ YAML                | ✅ YAML              | ✅ YAML              | ✅ YAML              |
| UI/UX                  | ❌ Clunky           | ✅ Modern              | ✅ Modern            | ✅ Modern            | ⚠️ Aging              |

---

## 🤖 In a Robotics Context

| Task                               | Jenkins Application                            |
|------------------------------------|-------------------------------------------------|
| Build and test ROS2 packages       | Use Docker + Jenkinsfile to build colcon stacks |
| Run simulation tests               | Trigger Gazebo tests via CI                    |
| Deploy container to K8s            | Use `kubectl` or `helm` in Jenkins pipeline     |
| Firmware cross-compilation         | Run on agent with arm-none-eabi toolchain       |
| Validate SLAM outputs              | Compare map data post-run using custom scripts  |

---

## 🔧 Useful Commands & Features (one-liners)

- `jenkins-cli.jar -s http://localhost:8080/ build <job-name>` – Trigger build
- `jenkins-cli.jar -s http://localhost:8080/ list-jobs` – List available jobs
- `docker run -p 8080:8080 jenkins/jenkins:lts` – Start Jenkins in Docker
- `pipeline { agent any; stages { stage('Build') { steps { ... }}}}` – Jenkinsfile snippet
- `post { failure { mail ... }}` – Jenkinsfile notification hook

---

## 🔧 Compatible Items

- [[Dockerfile]] – Used to build containerized apps in Jenkins
- [[Docker Container]] – Can run jobs inside isolated containers
- [[Kubernetes Deployment]] – Target for CI/CD pipelines
- [[GitHub Actions]] – Simpler alternative for GitHub-hosted projects
- [[ROS2 Package]] – Jenkins can build and test colcon workspaces
- [[CI-CD]] – Jenkins is a core tool in this ecosystem

---

## 🔗 Related Concepts

- [[CI-CD]] (Jenkins is one of the most powerful implementations)
- [[Microservices Architecture]] (Automate builds for each service)
- [[Helm Chart]] (Used in deploy stages)
- [[kubectl]] (Run commands in Jenkins to deploy)

---

## 📚 Further Reading

- [Jenkins Official Docs](https://www.jenkins.io/doc/)
- [Pipeline Syntax Guide](https://www.jenkins.io/doc/book/pipeline/syntax/)
- [Jenkins GitHub Integration](https://plugins.jenkins.io/github/)
- [Jenkins Plugin Index](https://plugins.jenkins.io/)
- [Jenkins in Docker (Official)](https://www.jenkins.io/doc/book/installing/docker/)

---
