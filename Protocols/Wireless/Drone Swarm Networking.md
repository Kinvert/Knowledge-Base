# Drone Swarm Networking

Drone swarm networking is communication for many vehicles that must coordinate position, trajectory, task allocation, and safety states with intermittent links. Design is less about raw throughput and more about determinism, prioritization, and failure semantics.

---

## ⚙️ Overview

Swarm networking usually has two layers:

- **Mission/data plane**: discovery, formation control, state sync, formation changes, task handoff.
- **Safety plane**: emergency status, collision avoidance triggers, geofencing updates, and heartbeat/fail-safety states.

Most practical swarms split channels or topics:
- low-rate periodic state,
- event-driven control updates,
- large payloads only when channel quality supports.

---

## 🧠 Design Patterns

- **Role-based topicing**: leader, relay, and leaf nodes with different publish rates.
- **State compression**: send delta position/velocity only.
- **Priority pre-emption**: keep safety packets ahead of map uploads.
- **Adaptive topology**: reduce peer degree as interference increases.
- **Clock discipline**: sync matters for coordinated actuation windows.

---

## 📊 Comparison Chart

| Pattern | Best Fit | Strength | Weakness | Typical Tech |
|---|---|---|---|---|
| Star + gateway | Dense ops, stable uplink | Simpler debugging | Gateway single-point failure | Wi-Fi/BTS + cellular |
| Full mesh | No infrastructure | Better reach, fewer dead zones | variable latency, routing churn | Meshcore/Meshtastic-like stacks |
| Hybrid star-mesh | Mixed terrain | Good resilience + manageability | complex policy routing | Multi-hop gateways |
| Broker pub/sub overlay | Large fleets | Easy filtering/policy | requires broker or root | DDS/ROS2 bridges |
| Duty-cycled opportunistic | Power constrained swarms | Long endurance | lower coordination precision | Custom low-rate links |

---

## 🛠 Use Cases

- Coordinated inspection where each drone relays neighbor state.
- Racing or obstacle tests where agents run in formation and swap roles.
- Autonomous delivery and search-and-rescue with dynamic leader election.

---

## ✅ Strengths

- Scales mission robustness; no single link owns all state.
- Reduces latency spikes from single-hop dependence.
- Supports graceful degradation when individual nodes drop.

---

## ❌ Weaknesses

- Hardest layer to verify formally for high-criticality operations.
- Message storms can occur without strict rate controls.
- Security posture is harder when more peers can route.

---

## 🔧 Relevant Protocol Candidates

- [[MAVLink]]
- [[Mesh Networking]]
- [[Reticulum]]
 - [[DDS]]
- [[CRSF]] for per-vehicle control, with out-of-band swarm overlay

---

## 🌐 External Resources

- [MAVLink / GCS ecosystem docs](https://mavlink.io/en/)
- [Mesh networking ecosystem references](https://www.opennetlinux.org)
- [IEEE robotics wireless coordination references](https://www.ieee.org)
