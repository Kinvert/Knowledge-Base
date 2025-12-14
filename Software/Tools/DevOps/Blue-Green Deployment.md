# Blue-Green Deployment

*Blue-Green deployment* is a release strategy that maintains two nearly identical production environments—**Blue** (current live) and **Green** (new version). Traffic is switched from Blue to Green in a single, controlled cutover. This approach minimizes downtime and enables extremely fast rollback, making it attractive for infrastructure-heavy systems, real-time services, and reinforcement learning (RL) platforms where availability and determinism matter.

---

## 📚 Overview

In a Blue-Green setup, only one environment serves live traffic at a time. The new version is fully deployed and validated in the idle environment before traffic is redirected. If a problem occurs, traffic is immediately switched back.

In RL systems, Blue-Green deployments are commonly used for:
- Replacing inference services or policy servers
- Upgrading environment simulators
- Deploying schema-breaking changes safely
- Ensuring deterministic rollouts for evaluation

---

## 🧠 Core Concepts

- **Two Full Environments**: Blue (live) and Green (idle or candidate)
- **Instant Traffic Switch**: Load balancer or DNS-based cutover
- **Zero (or Near-Zero) Downtime**
- **Simple Rollback**: Switch traffic back
- **Environment Parity**: Blue and Green must be identical

---

## 🔧 How It Works

1. Blue environment serves all production traffic.
2. Green environment is deployed with the new version.
3. Green is validated via health checks, smoke tests, and probes.
4. Traffic is switched from Blue to Green.
5. If issues arise, traffic is switched back instantly.

Unlike canary deployments, there is no gradual exposure—traffic moves all at once.

---

## 🛠️ Strengths

- Extremely fast rollback
- Simple mental model
- Minimal downtime
- Strong isolation between versions
- Works well for schema or dependency changes
- Deterministic behavior post-cutover

---

## ⚠️ Weaknesses

- Requires double the infrastructure
- No gradual exposure to real traffic
- Harder to detect rare edge cases before full switch
- State and data migrations must be carefully coordinated
- Less observability during the instant transition

---

## 📊 Comparison Chart

| Strategy | Rollout Style | Infrastructure Cost | Rollback Speed | Notes |
|--------|---------------|---------------------|----------------|-------|
| **Blue-Green Deployment** | Instant switch | High | Very Fast | Two full environments |
| **Canary Deployment** | Gradual | Medium | Fast | Metric-driven |
| **Rolling Deployment** | Sequential | Low | Medium | Default in many systems |
| **A-B Testing** | Parallel | Medium | Fast | Experiment-focused |
| **Shadow Deployment** | Duplicate traffic | Medium | N/A | No user impact |
| **Big Bang Deployment** | All at once | Low | Slow | Highest risk |

---

## 🧩 Use Cases

- Deploying new RL inference or policy servers
- Introducing breaking API changes
- Infrastructure or OS upgrades
- Major dependency version changes
- Systems requiring strong rollback guarantees
- LiveView or PubSub backend upgrades with minimal downtime

---

## 🛠️ Compatible Items

- [[Load Balancer]]
- [[DNS]]
- [[CI-CD]]
- [[Kubernetes]]
- [[Cloud Infrastructure]]
- [[Distributed Systems]]
- [[Microservices]]

---

## 🧭 Variants and Related Strategies

- **Blue-Green + Canary**: Canary inside Green before switch
- **Blue-Green with Feature Flags**: Logic toggled post-cutover
- **Regional Blue-Green**: Switch per region or cluster
- **Database Blue-Green**: Dual-write or versioned schemas

---

## 📑 Developer Tools

- Load balancers and traffic routers
- Kubernetes services and ingress
- CI/CD pipelines with promotion gates
- Feature flag platforms
- Health check and smoke test tooling

---

## 📑 Documentation and Support

- Cloud provider architecture guides
- Kubernetes deployment strategy docs
- SRE handbooks
- Infrastructure-as-Code references

---

## 🔗 Related Concepts / Notes

- [[Canary Deployment]]
- [[Rolling Deployment]]
- [[Shadow Deployment]]
- [[A-B Testing]]
- [[Feature Flags]]
- [[CI-CD]]
- [[Observability]]

---

## 📝 Summary

Blue-Green deployment prioritizes safety through isolation and fast rollback. While it lacks the gradual exposure of canary deployments, it excels in scenarios requiring deterministic cutovers and immediate reversibility. For RL systems and distributed platforms where downtime or partial failure is unacceptable, Blue-Green remains a foundational deployment strategy.
