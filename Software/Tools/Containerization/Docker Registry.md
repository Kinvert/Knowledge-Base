# Docker Registry

A Docker Registry is a storage and distribution system for Docker Images. It allows developers and organizations to push, pull, and manage container images, enabling efficient collaboration, deployment, and version control. Registries are essential in robotics pipelines where containerized environments must be shared across developers, robots, and CI systems.

---

## 📚 Overview

Registries can be public (like Docker Hub) or private (hosted on-prem or in the cloud). They support tagging, versioning, and access control for images. Registries are often integrated into CI/CD pipelines to automate testing and deployment of robotics software, simulation environments, and ML models.

---

## 🧠 Core Concepts

- **Image Tag**: A human-readable label (e.g. `:latest`, `:v1.0.3`) for a version of an image.
- **Push**: Uploading an image to the registry (`docker push`).
- **Pull**: Downloading an image from the registry (`docker pull`).
- **Authentication**: Access to private registries usually requires credentials or tokens.
- **Namespaces/Repositories**: Images are stored in logical namespaces (e.g., `username/image-name`).

---

## 🧰 Use Cases

- Distributing robotics simulation stacks (e.g. Gazebo + ROS2 environments)
- Sharing containerized AI/ML inference models across platforms
- Storing validated CI-built images for deployment on robots
- Hosting multiple platform builds (e.g., `amd64`, `arm64` for Jetson)
- Versioning robot software for reproducibility and rollback

---

## ✅ Pros

- Enables reproducibility across teams and hardware
- Supports versioning and tagging of images
- Works with public and private deployments
- Many managed options (e.g., GitHub, AWS, GCP, Azure)
- Integrates seamlessly with CI/CD pipelines

---

## ❌ Cons

- Private registries may incur cost or setup complexity
- Pushing large images over limited networks is slow
- Version management can get messy without conventions
- Requires secure credential handling for authentication

---

## 📊 Comparison Chart

| Feature                  | [[Docker Hub]]           | [[GitHub Container Registry]] | [[GitLab Container Registry]] | [[Self-Hosted Registry]]   |
|--------------------------|----------------------|----------------------------|----------------------------|------------------------|
| **Public/Private**       | ✅ Both              | ✅ Both                   | ✅ Both                   | ✅ Configurable        |
| **Integration**          | General-purpose      | GitHub CI/CD               | GitLab CI/CD               | Any Docker client      |
| **Rate Limits**          | ⚠️ Yes (free tier)  | ✅ Higher limits           | ✅ Higher limits           | ❌ None               |
| **Ease of Use**          | ✅ Easiest           | ✅ GitHub-native            | ✅ GitLab-native            | ⚠️ Requires setup      |
| **Best For**             | Public sharing       | GitHub-based projects      | GitLab pipelines           | Local & air-gapped use |

---

## 🤖 Comparison: Docker Registry vs Package Manager

| Feature               | Docker Registry         | [[Package Manager]] (e.g. `apt`, `pip`) |
|------------------------|--------------------------|--------------------------------------|
| **Purpose**            | Store and deliver images | Store and deliver packages           |
| **Granularity**        | Full environment         | Individual software components       |
| **Containerized**      | ✅ Yes                   | ❌ No                                |
| **Deployment Speed**   | Fast once downloaded     | Often slower, needs dependency solve |

---

## 🔧 Compatible Items

- `docker login`, `docker push`, `docker pull`
- CI/CD tools like [[GitHub Actions]], [[GitLab CI]], [[Drone]]
- [[Dockerfile]], [[Docker Image]], [[Docker Container]]
- [[Kubernetes]] (pulls images from registries)
- [[ROS2]] images for specific robots or simulations

---

## 🔗 Related Concepts

- [[Docker Image]] (Stored in registries)
- [[Docker Container]] (Pulled from registry and run)
- [[Dockerfile]] (Build source for images pushed to registry)
- [[CI-CD Pipelines]] (Use registries for reproducible deploys)
- [[Kubernetes Pod]] (Images fetched from registries)
- [[Air-Gapped Deployment]] (May require local registry)

---

## 🛠 Developer Tools

- `docker push username/image:tag`
- `docker pull image:tag`
- `docker login registry.domain.com`
- `docker tag image registry.domain.com/image`
- `docker logout`

---

## 📚 Further Reading

- [Docker Hub Documentation](https://docs.docker.com/docker-hub/)
- [GitHub Container Registry (GHCR)](https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry)
- [Self-Hosted Docker Registry Guide](https://docs.docker.com/registry/deploying/)
- [Managing Access Tokens for Private Registries](https://docs.docker.com/engine/reference/commandline/login/)

---
