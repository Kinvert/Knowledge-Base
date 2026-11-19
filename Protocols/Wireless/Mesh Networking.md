# Mesh Networking

Mesh networking lets nodes forward each other’s packets without a fixed central base station. It increases path resilience and area coverage for fleets, but adds routing overhead and complexity compared to star topologies.

---

## ⚙️ Overview

In a mesh, each node can be both endpoint and router. That’s useful for UAV/UGV operations where line-of-sight is intermittent or ground infrastructure is unavailable. Common outcomes:

- Better coverage per deployed node,
- Better survivability when individual links fail,
- More uncertain latency and variable throughput versus fixed infrastructure.

For command/control, mesh is often used for low-rate messages only, with heavier traffic moved to a stronger out-of-band path.

---

## 🧠 Core Concepts

- **Routing strategy**: reactive (on-demand), proactive (table-driven), or geographic.
- **Flooding control**: TTL and duplicate suppression are critical.
- **Beaconing**: neighbor discovery and link health maintenance.
- **Power policy**: each relay function consumes extra energy.
- **Gateway egress strategy**: one or multiple gateways to internet/backhaul.

---

## 📊 Comparison Chart

| Feature | Meshcore | Meshtastic | BLE Mesh | 802.11s | Reticulum |
|---|---|---|---|---|---|
| Transport flexibility | High | Medium | BLE only | Wi-Fi | High |
| Topology | Multi-hop mesh | Multi-hop mesh | Managed mesh | Mesh STA | Opportunistic / mesh |
| Range profile | Moderate-extensive | Moderate | Short | Moderate | Long via hetero links |
| Latency predictability | Low | Low-Med | Low-Med | Low-Med | Low-Med |
| Security model | App-level crypto | Shared keys | AES mesh profile | WLAN security | End-to-end variants |
| Best use in drones | Long missions, swarms | Simpler field networks | Wearables + peripherals | Throughput-focused ad hoc | Tactical robust links |

---

## 🛠 Use Cases

- Swarm telecommand with intermittent relays.
- Disaster response when infrastructure is gone.
- Temporary relay chains between ground vehicles and drones.

---

## ✅ Strengths

- Reduces single point of failure risk.
- Extends reach beyond a single RF hop.
- Handles partial infrastructure outages gracefully.

---

## ❌ Weaknesses

- Non-deterministic end-to-end latency.
- Routing chatter can consume bandwidth and power.
- Complexity in QoS tuning for mixed criticality traffic.

---

## 📚 Related Concepts

- [[Drone Swarm Networking]]
- [[RF Link Resilience Stack for Drones]]
- [[Meshcore]]
- [[Meshtastic]]
- [[RF Jamming and Anti-Jamming]]

---

## 🌐 External Resources

- [Bluetooth Mesh Profile overview](https://www.bluetooth.com/bluetooth-resources/learn-about-bluetooth/bluetooth-mesh)
- [IETF RFCs for Ad-hoc networking families (high level)](https://www.rfc-editor.org/)
- [Meshtastic Mesh networking docs](https://meshtastic.org/docs/)
- [OpenWrt batman-adv mesh overview](https://www.open-mesh.org/projects/batman-adv/wiki)

