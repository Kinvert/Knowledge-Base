# npm

**npm** (Node Package Manager) is the default package manager for the JavaScript runtime environment **Node.js**. It manages JavaScript packages and dependencies, making it a key tool in modern web development, frontend frameworks, backend APIs, and increasingly in cross-platform tools for robotics dashboards and control panels.

---

## 📚 Overview

npm installs and manages packages from the npm registry, which hosts over a million packages. It supports version control, semantic versioning, script automation, and dependency trees. In robotics and engineering contexts, npm is often used for front-end visualization tools (e.g., dashboards, Foxglove Studio plugins) and back-end servers (e.g., gRPC-Web or REST APIs).

---

## 🧠 Core Concepts

- **`package.json`**: Defines project metadata, dependencies, and scripts
- **Scripts**: Task automation (e.g., `npm run build`, `npm test`)
- **Semantic Versioning**: Uses `^`, `~`, etc., to manage version ranges
- **Global vs Local Installs**: Install tools for system-wide (`-g`) or per-project use
- **node_modules**: Auto-managed folder storing all project dependencies

---

## 🧰 Use Cases

- Robotics dashboards (e.g., Node.js + React for monitoring)
- Building and serving front-end tools for [[ROS2 Web Bridge]]
- Managing build tools and static asset compilers (Webpack, Rollup, Vite)
- Hosting REST/gRPC backend services with Express or Fastify
- Controlling UI interfaces for physical simulators or control apps

---

## ✅ Pros

- Huge ecosystem and active community
- Easy to automate tasks and builds
- Cross-platform support (Windows, macOS, Linux)
- Seamless integration with front-end and back-end tooling
- Efficient module caching and installation

---

## ❌ Cons

- `node_modules` can become bloated or hard to manage
- Version conflicts due to deep nested dependencies
- Security issues with poorly maintained packages
- Not well-suited for low-level or embedded development

---

## 📊 Comparison Chart

| Feature                   | npm               | yarn              | pip               | conda             | apt                |
|---------------------------|-------------------|-------------------|-------------------|-------------------|--------------------|
| Language Support          | ✅ JavaScript      | ✅ JavaScript      | ❌ Python-only     | ✅ Multi-lang      | ✅ OS-level         |
| Speed                    | ⚠️ Moderate         | ✅ Fast (offline cache) | ✅ Fast          | ⚠️ Moderate        | ✅ Fast             |
| Lock Files               | ✅ `package-lock.json` | ✅ `yarn.lock`  | ⚠️ Optional         | ✅ YAML            | ❌ No               |
| Build Tools Integration  | ✅ Webpack, Vite    | ✅                | ✅ setuptools      | ⚠️ Mixed           | ❌ Not applicable   |
| Front-End Usage          | ✅ Yes              | ✅ Yes             | ❌                | ❌                | ❌                 |

---

## 🤖 In a Robotics Context

| Task or Tool                             | npm Usage                                           |
|------------------------------------------|-----------------------------------------------------|
| Building web dashboards                  | Install React, Vue, or Svelte front-ends            |
| gRPC-Web or REST UI                      | Manage frontend client libraries and bundlers       |
| ROS2 Web Bridge integration              | Compile and serve interface tools                   |
| Physical simulator GUI                   | Manage packages for electron or browser UIs         |
| Foxglove plugins                         | Build and serve visualization extensions            |

---

## 🔧 Useful Commands (One-Liners)

- `npm init -y` – Create a new `package.json`  
- `npm install express` – Install a package  
- `npm install` – Install dependencies listed in `package.json`  
- `npm run build` – Run a script defined in `package.json`  
- `npm outdated` – Show outdated packages  
- `npm update` – Update all packages  
- `npm install -g typescript` – Globally install a tool  

---

## 🔧 Compatible Items

- [[Node.js]] – Runtime that npm is built for  
- [[ROS2 Web Bridge]] – Often paired with npm-built interfaces  
- [[gRPC]] – Use `@grpc/grpc-js` and similar via npm  
- [[REST API]] – Backend frameworks like Express installed via npm  
- [[Dockerfile]] – npm used to build and run web servers in containers  
- [[CI-CD]] – Commonly used in GitHub Actions or Jenkins builds  

---

## 🔗 Related Concepts

- [[Node.js]] (npm's runtime environment)  
- [[Dockerfile]] (npm installs in frontend/backend build stages)  
- [[REST API]] (Often built using Express or Koa)  
- [[gRPC]] (npm provides web-friendly gRPC clients)  
- [[ROS2 Web Bridge]] (npm-powered GUIs interact with ROS2)  
- [[CI-CD]] (Automate install/test/deploy of npm apps)  

---

## 📚 Further Reading

- [npm Official Docs](https://docs.npmjs.com/)
- [npm Registry](https://www.npmjs.com/)
- [Understanding `package.json`](https://nodesource.com/blog/the-basics-of-package-json-in-node-js/)
- [npm vs yarn Comparison](https://classic.yarnpkg.com/en/docs/migrating-from-npm/)

---
