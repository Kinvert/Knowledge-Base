# GitHub Actions

**GitHub Actions** is a CI/CD (Continuous Integration and Continuous Deployment) platform built into GitHub. It allows you to automate tasks in your software development lifecycle directly from your GitHub repository, such as building, testing, linting, deploying, or publishing packages.

In robotics and engineering workflows, GitHub Actions can automate code quality checks, build ROS packages, containerize apps with Docker, and deploy services to cloud platforms like Kubernetes.

---

## 📚 Overview

GitHub Actions works by defining workflows in `.github/workflows/*.yaml` files. These workflows are triggered by events (like a `push`, `pull_request`, or `schedule`), and they execute jobs composed of individual steps. Jobs run on GitHub-hosted or self-hosted runners using Ubuntu, macOS, or Windows.

---

## 🧠 Core Concepts

- **Workflow**: A YAML file describing automation logic
- **Job**: A set of steps executed on the same runner
- **Step**: A command or action that runs inside a job
- **Runner**: The environment where the job runs (e.g., `ubuntu-latest`)
- **Action**: A reusable unit (either public or custom) that performs a task (e.g., `actions/checkout`, `docker/build-push-action`)
- **Secrets**: Secure variables used for tokens or keys in workflows

---

## 🧰 Use Cases

- Lint and test code automatically on every PR
- Build and deploy ROS2 packages in a Docker container
- Push Docker images to a registry (e.g., Docker Hub, GHCR)
- Deploy Kubernetes manifests using `kubectl` or `Helm`
- Schedule simulation tests or nightly builds
- Run unit tests for robotics perception or planning modules
- Trigger deployments to cloud platforms like [[Heroku]] or [[DigitalOcean App Platform]]

---

## ✅ Pros

- Integrated directly into GitHub
- Easy to version workflows in the repository
- Extensive marketplace of pre-built actions
- Secure secret management
- Scalable runners for most needs
- Works well with monorepos and microservices

---

## ❌ Cons

- Free-tier limits for private repos or heavy compute jobs
- Can become slow with large matrix builds
- Debugging failed builds may require verbose logging
- Some features (e.g., caching, Docker-in-Docker) require tuning

---

## 📊 Comparison Chart

| CI/CD Platform       | GitHub Actions       | GitLab CI/CD        | Jenkins              | CircleCI             | Travis CI            |
|----------------------|----------------------|----------------------|----------------------|----------------------|----------------------|
| Integrated with Git  | ✅ GitHub native      | ✅ GitLab native     | ❌ External setup    | ✅ GitHub + others    | ✅ GitHub + others    |
| Setup Complexity     | ✅ Low                | ⚠️ Moderate          | ❌ High              | ✅ Low                | ✅ Low                |
| UI/UX                | ✅ Clean              | ✅ Clean              | ❌ Dated             | ✅ Clean              | ⚠️ Aging              |
| Marketplace Support  | ✅ Large              | ✅ Moderate           | ❌ Plugin-based      | ✅ Moderate           | ⚠️ Limited            |
| Self-hosted Support  | ✅ Yes                | ✅ Yes               | ✅ Yes               | ✅ Yes                | ⚠️ Yes                |

---

## 🤖 In a Robotics Context

| Workflow Goal                      | Example Configuration/Action                           |
|-----------------------------------|----------------------------------------------------------|
| ROS2 build and test               | Use `ros-tooling/setup-ros` and `colcon build`          |
| Lint Python code                  | Use `actions/setup-python` + `flake8`                   |
| Build Docker image for SLAM       | Use `docker/build-push-action`                          |
| Push image to registry            | Authenticate and push to GHCR or Docker Hub             |
| Deploy to Kubernetes              | Use `azure/setup-kubectl` or `helm` action              |
| Run unit tests for sensor modules | Use `pytest` or Google Test in `run:` steps             |

---

## 🔧 Useful Commands & Features (one-liners)

- `gh workflow list` – List workflows (with GitHub CLI)
- `gh run watch` – Watch a workflow run live
- `gh workflow run <workflow.yml>` – Manually trigger a workflow
- `actions/checkout@v3` – Checkout the repository
- `actions/setup-python@v4` – Setup Python environment
- `docker/build-push-action@v5` – Build and push Docker image

---

## 🧩 Compatible Items

- [[Dockerfile]] (Often built during CI/CD runs)
- [[Docker Container]] (Image build/deploy automation)
- [[Kubernetes Deployment]] (Trigger deploys post-build)
- [[ROS2 Package]] (Can be built and tested automatically)
- [[CI-CD]] (GitHub Actions is one implementation)
- [[Heroku]] / [[DigitalOcean App Platform]] (For automated deploys)

---

## 🔗 Related Concepts

- [[CI-CD]] (General concept of continuous automation)
- [[Docker Compose]] (May be used in local testing workflows)
- [[Microservices Architecture]] (Each service can have its own workflow)
- [[Kubernetes]] (Actions can deploy directly to clusters)

---

## 📚 Further Reading

- [GitHub Actions Docs](https://docs.github.com/en/actions)
- [Awesome Actions (GitHub)](https://github.com/sdras/awesome-actions)
- [ROS2 GitHub Action Template](https://github.com/ros-tooling/action-ros-ci)
- [GitHub CLI Docs](https://cli.github.com/manual/)
- [Docker + GitHub Actions](https://docs.docker.com/ci-cd/github-actions/)

---
