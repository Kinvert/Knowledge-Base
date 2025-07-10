# Heroku

**Heroku** is a popular cloud [[PaaS]] (Platform-as-a-Service) that enables developers to build, run, and operate applications entirely in the cloud. It abstracts infrastructure management by providing easy deployment, scaling, and monitoring for web applications and services. Heroku is widely used for rapid prototyping, small-to-medium web apps, APIs, and backend services—including robotics web dashboards, microservices, and AI model hosting.

---

## 📚 Overview

Heroku supports many programming languages via [[Heroku Buildpacks]] including Python (e.g., Django, Flask), Node.js, Ruby, Java, Go, PHP, and more. Its git-based deployment model makes it simple to push changes, while add-ons provide managed databases, caching, logging, monitoring, and messaging. Heroku manages runtime, scaling, load balancing, and security, so developers can focus on code.

---

## 🧠 Core Concepts

- **[[Heroku Dynos]]**: Lightweight Linux containers running your application processes.
- **[[Heroku Buildpacks]]**: Scripts that prepare the environment and dependencies per language.
- **[[Heroku Add-ons]]**: Managed third-party services (databases, caches, logging).
- **Git-based Deploy**: Deploy apps via `git push heroku main`.
- **Procfile**: Defines process types (web, worker) and startup commands.
- **[[Heroku CLI]]**: Command-line tool to manage apps, add-ons, logs, scaling.

---

## 🧰 Use Cases

- Hosting web applications or [[REST API]]s (e.g., Django, Flask, Express.js)
- Running backend microservices for robotics fleet management or telemetry
- Prototyping and testing new robotic web frontends or dashboards
- Deploying lightweight AI model APIs (e.g., TensorFlow Serving wrappers)
- Integrating databases and caches with minimal setup

---

## ✅ Pros

- **Ease of use:** Simple git-based deployment workflow
- **Multi-language support:** Official and community buildpacks for many languages
- **Scalability:** Easily scale dynos up/down or horizontally
- **Managed services:** Add-ons for Postgres, Redis, Elasticsearch, etc.
- **Free tier:** Good for experimentation and small projects
- **Integration:** Works well with CI/CD, GitHub, Docker, and popular dev tools
- **Security:** Managed SSL, automatic OS patching, and platform security

---

## ❌ Cons

- **Cost:** Can become expensive as you scale or add add-ons
- **Limited control:** Abstracts infrastructure, less customizable than raw cloud VMs or Kubernetes
- **Dyno sleeping:** Free-tier apps “sleep” after inactivity causing cold starts
- **Not ideal for stateful workloads:** Primarily designed for stateless web apps
- **Limited GPU support:** Not suited for heavy AI model training or large-scale robotics compute

---

## 📊 Comparison Chart: Heroku vs Other PaaS / Cloud Platforms

| Feature              | Heroku                     | Google App Engine           | AWS Elastic Beanstalk        | DigitalOcean App Platform    | Kubernetes (EKS/GKE/AKS)       |
|----------------------|----------------------------|----------------------------|------------------------------|-----------------------------|-------------------------------|
| Multi-language       | ✅ Wide via buildpacks       | ✅ Wide                      | ✅ Wide                       | ✅ Wide                      | ✅ Any (containerized)         |
| Deployment Model     | Git push                   | Git, gcloud CLI, Docker    | CLI, Console, Docker          | Git, CLI, Docker             | kubectl, Helm, CLI             |
| Infrastructure Control| Minimal                   | Minimal                   | Moderate                     | Minimal                     | Full control                  |
| Scaling              | Auto/manual scaling        | Auto/manual scaling        | Auto/manual scaling           | Auto/manual scaling          | Customizable pod scaling       |
| Add-ons / Marketplace| Large variety              | Integrated services         | Wide AWS ecosystem            | Add-ons marketplace          | Huge ecosystem, self-managed  |
| Free Tier            | Yes (limited)              | Yes (limited)              | Limited                       | Yes (limited)               | No (pay per cloud resources)  |
| GPU Support          | No                        | Limited (App Engine Flex)  | Possible via EC2              | Limited                     | Yes, full control             |
| Suitable For         | Web apps, APIs, prototyping | Web apps, APIs             | Web apps, APIs, more control  | Web apps, APIs              | Complex microservices, robotics|

---

## 🤖 In Robotics Context

| Scenario                      | Heroku Usage                                    |
|-------------------------------|------------------------------------------------|
| Web dashboard for robot fleet | Host React/Vue frontend + Flask/Django backend  |
| API gateway for telemetry     | REST API with Node.js or Python on Heroku       |
| Prototyping control UI        | Quick deploy and test without infra management  |
| Lightweight AI inference API  | Deploy model wrappers or small ML services      |
| Database-backed robot logging | Use Heroku Postgres add-on for persistence      |

---

## 🔧 Useful Heroku CLI Commands (one liners)

- `heroku login` – Authenticate CLI with your Heroku account  
- `heroku create <app-name>` – Create a new Heroku app  
- `git push heroku main` – Deploy code from local git to Heroku  
- `heroku ps:scale web=1` – Scale web dynos to 1 instance  
- `heroku logs --tail` – Stream app logs in real-time  
- `heroku addons:create heroku-postgresql:hobby-dev` – Add free Postgres database  
- `heroku config:set KEY=value` – Set environment variables  
- `heroku run bash` – Open a remote shell inside a dyno  
- `heroku open` – Open the app in a browser  
- `heroku releases` – View deployment history  
- `heroku rollback` – Roll back to previous release  

---

## 🔗 Related Platforms and Alternatives

- [[Google App Engine]] – Managed PaaS with deeper GCP integration  
- [[AWS Elastic Beanstalk]] – AWS’s PaaS for simple app deployment  
- [[DigitalOcean App Platform]] – Easy app deployment on DigitalOcean  
- [[Netlify]] – Specialized for static sites and frontend apps  
- [[Vercel]] – Frontend-focused deployment platform  
- [[Kubernetes]] – For full control over container orchestration  
- [[Docker Container]] – Container packaging for Heroku apps

---

## 🔗 Related Concepts

- [[Docker Container]] (You can deploy Dockerized apps on Heroku)  
- [[CI-CD Pipelines]] (Integrate Heroku deployment into pipelines)  
- [[Microservices Architecture]] (Deploy microservices as Heroku apps)  
- [[Python]] (Django, Flask apps run well on Heroku)  
- [[Node.js]] (Express and other web servers on Heroku)  

---

## 📚 Further Reading

- [Heroku Dev Center](https://devcenter.heroku.com/)  
- [Deploying Python (Django/Flask) Apps](https://devcenter.heroku.com/categories/python-support)  
- [Heroku Buildpacks](https://devcenter.heroku.com/articles/buildpacks)  
- [Heroku CLI Commands](https://devcenter.heroku.com/articles/heroku-cli-commands)  
- [Heroku Add-ons Marketplace](https://elements.heroku.com/addons)  
- [12 Factor App Methodology](https://12factor.net/) (Heroku inspired)

---
