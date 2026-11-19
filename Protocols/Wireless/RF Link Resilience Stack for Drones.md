# RF Link Resilience Stack for Drones

This note describes a practical, reusable stack for making drone radio links survive real environments: interference, multipath, LOS breaks, and command path contention. It is designed around small-to-medium budgets and agent-driven parameter iteration.

---

## ⚙️ Overview

A resilient stack is usually split into five layers:

1. **PHY hardening**: AFH/FHSS/CSS, filter settings, power control.
2. **Link adaptation**: packet sizing, redundancy, adaptive rates, channel scoring.
3. **Transport and session policy**: heartbeat/sequence guards, retries, ACK budgets.
4. **Control semantics**: heartbeat timeout -> fail-safe decision tree.
5. **Operations and observability**: logs, per-channel stats, and replay-safe datasets.

Each layer has independent tuning points; that is the key to optimization loops.

---

## 🧠 Layered Stack

- **PHY**: choose modulation family + band + channel strategy.
- **MAC**: collision control, retransmission policy, duty-cycle limits.
- **Network/mesh**: relay policy and priority queues.
- **Application**: heartbeat, arming state, emergency state machine.
- **Automation**: supervisor that can test candidate settings and reject unstable regions.

---

## 📊 Layer Tradeoff Matrix

| Layer | Aggressive Setting | Safe Setting | What to Watch |
|---|---|---|---|
| PHY spread setting | High range, low throughput | Higher throughput | Control latency drift |
| Retry policy | High retries | Low retries | Latency inflation |
| Heartbeat timeout | Tight | Loose | False-fail risks vs delayed failover |
| Telemetry rate | Adaptive | Fixed low | Bandwidth starvation |
| Mesh degree | Dense mesh | Conservative peers | Power drain / congestion |

---

## 🛠 Agent Workflow Pattern

1. Define a config schema (JSON/INI) with all link knobs.
2. Run controlled grid search over:
   - center frequency,
   - hopset width,
   - retry count,
   - heartbeat timeout,
   - power and spreading.
3. Score each run by mission success, link uptime, and control latency.
4. Persist winning profiles by scenario (urban canyon, race track, open field).
5. Promote the top profile with explicit safety constraints.

---

## ✅ Strengths

- Keeps “what failed” measurable and reproducible.
- Supports rapid adaptation if environment shifts.
- Lets one architecture serve both pilot and autonomous workflows.

---

## ❌ Weaknesses

- Requires good telemetry and experiment discipline.
- Too many knobs create non-convex tuning surfaces.
- Human review is still needed for fail-safe behavior.

---

## 📚 Related Concepts

- [[Drone C2 Radio Protocols]]
- [[RF Jamming and Anti-Jamming]]
- [[Adaptive Frequency Hopping]]
- [[Mesh Networking]]
- [[Interference-Hardened Radio Design for UAVs]]

---

## 🌐 External Resources

- [OpenDroneID and safety message handling references](https://www.faa.gov/uas/recreational_fliers/remote_id/)
- [MAVLink heartbeat and system status semantics](https://mavlink.io/en/)
- [General resilient network design (control systems)](https://www.nist.gov/)

