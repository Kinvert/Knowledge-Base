# CI-CD Pipelines

CI/CD (Continuous Integration and Continuous Deployment/Delivery) Pipelines are automated workflows that build, test, and deploy code. In robotics and engineering projects, CI/CD ensures that software is reliably integrated, containerized, and deployed to hardware, cloud, or simulation environments without manual intervention.

---

## 📚 Overview

CI/CD pipelines consist of stages such as code checkout, dependency installation, build, testing, and deployment. These stages are executed automatically upon events like a `git push` or `pull request`, helping maintain software quality and reducing deployment friction.

---

## 🧠 Core Concepts

- **CI (Continuous Integration)**: Automatically build and test code with each commit or merge.
- **CD (Continuous Delivery)**: Automatically prepare builds for deployment, often requiring manual approval.
- **CD (Continuous Deployment)**: Automatically deploy validated builds to production systems.
- **Pipeline Stages**: Steps like `build`, `test`, `lint`, `deploy`.
- **Artifacts**: Files (binaries, images, logs) produced by the pipeline for distribution or debugging.

---

## 🧰 Use Cases

- Building and testing ROS2 nodes in Docker containers
- Running unit tests, integration tests, and hardware-in-the-loop (HIL) tests
- Auto-deploying robotics services to Kubernetes clusters or edge devices
- Linting and static analysis of C++, Python, or launch files
- Publishing Docker Images or binary firmware to registries

---

## ✅ Pros

- Catch integration bugs early through automated testing
- Enforces code quality with static analysis and style checks
- Reproducible and auditable builds and deployments
- Saves engineering time and reduces deployment errors
- Encourages modular and testable system design

---

## ❌ Cons

- Setup can be complex, especially for robotics-specific environments
- Build pipelines for simulations or hardware can be resource-intensive
- Hardware-in-the-loop testing requires physical test rigs or emulation
- Needs careful secret management and permission configuration

---

## 📊 Comparison Chart

| Feature                  | [[GitHub Actions]]       | [[GitLab CI-CD]]           | [[Jenkins]]                 | [[CircleCI]]                | [[Drone CI]]               |
|--------------------------|----------------------|--------------------------|--------------------------|--------------------------|-------------------------|
| **Hosting Model**        | SaaS + Self-hosted   | SaaS + Self-hosted       | Self-hosted              | SaaS                     | Self-hosted/lightweight |
| **Docker Support**       | ✅ Built-in          | ✅ Native                | ✅ Plugins available      | ✅ Native                | ✅ Built-in             |
| **K8s Integration**      | ⚠️ Indirect         | ✅ Native                | ✅ With plugins           | ⚠️ Limited              | ✅ With runners         |
| **Best For**             | GitHub Projects      | GitLab repos             | Highly customizable jobs | Fast iteration           | Lightweight CI/CD       |

---

## 🤖 Comparison: CI vs CD

| Feature                | CI (Integration)         | CD (Deployment/Delivery)     |
|------------------------|---------------------------|-------------------------------|
| **Trigger**            | Code push or PR           | Post-CI build/test            |
| **Focus**              | Testing, validation       | Deployment or packaging       |
| **Frequency**          | Multiple per day          | Fewer, more deliberate        |
| **Tools Used**         | Linters, unit tests       | Docker, SSH, Helm, `rsync`    |
| **Failure Action**     | Block merge               | Block deploy                  |

---

## 🔧 Compatible Items

- `docker`, `docker-compose`, `kubectl`, `colcon`, `catkin`
- [[Dockerfile]], [[Docker Image]], [[ROS2]], [[Kubernetes]], [[Gazebo]]
- Secrets management tools (e.g., GitHub Secrets, Vault, environment vars)
- [[Docker Registry]], [[Helm Chart]], [[VSCode Dev Containers]]

---

## 🔗 Related Concepts

- [[Dockerfile]] (CI/CD often builds these)
- [[Docker Image]] (Produced or pulled during pipelines)
- [[Docker Registry]] (CD pushes to these)
- [[Kubernetes]] (CD often deploys here)
- [[ROS2]] (Frequently tested and deployed via CI/CD)
- [[Dev Containers]] (Used to define test/build environments)

---

## 🛠 Developer Tools

- GitHub Actions: `.github/workflows/main.yml`
- GitLab CI: `.gitlab-ci.yml`
- Jenkins: `Jenkinsfile`
- CLI tools: `gh`, `glab`, `docker`, `kubectl`, `colcon test`

---

## 📚 Further Reading

- [What is CI/CD?](https://www.redhat.com/en/topics/devops/what-is-ci-cd)
- [GitHub Actions for ROS2](https://github.com/ros-tooling/action-ros-ci)
- [Continuous Integration with Gazebo](http://gazebosim.org/tutorials?tut=ci)
- [Best Practices in Robotics CI/CD](https://rosindustrial.org/news/2020/5/21/continuous-integration-ros-industrial)

---
