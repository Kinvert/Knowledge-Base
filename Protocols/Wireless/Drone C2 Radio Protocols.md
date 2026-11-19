# Drone C2 Radio Protocols

Drone command-and-control (C2) stacks combine a radio physical layer with transport framing and telemetry/application protocols. Reliability, fail-safe behavior, and failover strategy are protocol-level properties, not just radio power.

---

## ⚙️ Overview

Most “C2 stack” discussions blur three layers:

1. **PHY/MAC layer** (how bits hop/chirp/handoff),
2. **Transport framing** (packet format, CRC, retransmit, signing),
3. **Vehicle/application protocol** (MAVLink, CRSF, DroneCAN, or manufacturer-specific messages).

For reliable autonomy loops, hard constraints often come from the application layer’s heartbeat/heartbeat timeout/mission abort behavior.

---

## 🧠 Core Layer Models

- **CRSF (Crossfire protocol family family context)**: low-latency RC oriented framing and channel behavior.
- **MAVLink**: widely used autopilot protocol with heartbeat, telemetry types, and command classes.
- **MAVLink signing**: optional message signing for spoofing resistance.
- **DroneCAN/DSDL systems**: real-time internal vehicle bus integration plus uplink adaptation.
- **ExpressLRS / other ELRS-based links**: modern RF transport with configurable PHY behavior and open ecosystem.

---

## 📊 Comparison Chart

| Link System | Primary PHY Style | Typical Update Latency | Telemetry Richness | Failsafe Control | Notes |
|---|---|---|---|---|---|
| CRSF-centric RC chains | FHSS variants / custom hopping | Very low | Minimal / RC-centric | Strong RC failsafe patterns | Best for direct pilot control |
| MAVLink over serial/UDP tunnels | Depends on transport | Medium | High | Mission mode + heartbeats + timeouts | Strong ecosystem/tooling |
| Proprietary DJI-like links | High-grade multi-stage PHY | Very low | High | Proprietary redundancy | Vendor lock-in |
| LoRa/LoRaWAN-type stacks | CSS | Medium | Low-Med | Depends on app profile | Long range, low-rate |
| Wi-Fi telemetry links | OFDM | Very low (local) | High | App-specific | Range and interference limited |

---

## 🛠 Use Cases

- Human-in-the-loop flight with hard real-time control.
- Autonomous swarms with supervisory links and mission policy update channels.
- Fail-open/return-to-home loops that require deterministic health timeout behavior.

---

## ✅ Strengths

- Layered approach lets teams mix link technologies.
- Heartbeat and mission timeout patterns can compensate for brief RF losses.
- Rich community tooling around MAVLink for simulation and SITL integration.

---

## ❌ Weaknesses

- Many stacks were built for pilot operation, not fully deterministic AI-agent loops.
- Proprietary ecosystems can block deep custom instrumentation.
- Message spam and high telemetry rate can saturate low-rate links unless filtered.

---

## 🔧 Practical Agent-Oriented Controls

- Explicitly model:
  - heartbeat period (`heartbeat_ms`),
  - packet timeout (`c2_link_loss_ms`),
  - max retransmit attempts,
  - command queue depth,
  - downlink telemetry priority.
- Keep a control-plane channel separate from bulk telemetry whenever possible.

---

## 📚 Related Concepts

- [[MAVLink]]
- [[CRSF]]
- [[ExpressLRS]]
- [[Drone Swarm Networking]]
- [[RF Link Resilience Stack for Drones]]

---

## 🌐 External Resources

- [MAVLink 2 Guide](https://mavlink.io/en/)
- [CRSF protocol repository/wiki](https://github.com/crsf-wg/crsf/wiki)
- [ExpressLRS documentation](https://www.expresslrs.org/)
- [MAVLink message signing details](https://mavlink.io/en/guide/message_signing.html)

