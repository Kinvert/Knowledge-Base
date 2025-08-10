# MeshCore

MeshCore is an open-source C++ mesh networking stack and firmware for embedded systems using LoRa and other packet radio transports. It enables secure, decentralized, off-grid communication using multi-hop routing—ideal for rugged, infrastructure-limited, or emergency environments.

---

## ⚙️ Overview

MeshCore provides a lightweight, modular alternative to Meshtastic, focusing on flexibility, low resource usage, and multi-transport support. It works without central servers or internet connectivity and is built for developers who need more control over network behavior.

---

## 🧠 Core Concepts

- **Multi-hop packet routing** – Messages are relayed across multiple nodes to extend range.
- **Flooding with deduplication & TTL** – Prevents message loops and controls network load.
- **Transport-agnostic design** – Works with LoRa, BLE, Wi-Fi, serial, and UDP.
- **End-to-end encryption** – Supports AES-256-GCM and ChaCha20-Poly1305.
- **Node roles** – Relay, endpoint, or hybrid operation modes.
- **Provisioning & discovery** – Secure onboarding and network mapping via Hello messages.

---

## 📊 Comparison Chart

| Feature                    | MeshCore                          | Meshtastic                        | Reticulum                      | BLE Mesh                       |
|----------------------------|------------------------------------|------------------------------------|---------------------------------|---------------------------------|
| Transport Support          | LoRa, BLE, Wi-Fi, serial, UDP      | LoRa only                          | Multiple (LoRa, TCP, UDP, AX.25)| BLE only                        |
| Routing Mechanism          | Flooding + deduplication, TTL      | Flooding with SNR heuristics       | Deterministic routing + discovery| Managed flooding                |
| Security                   | AES-256-GCM / ChaCha20-Poly1305    | AES-256 shared channel key         | End-to-end crypto (various)     | AES-CCM                         |
| App Ecosystem              | CLI + minimal Web UI               | Full Android/iOS + integrations   | Command-line & scripting tools  | Vendor-specific apps            |
| Power Usage                | Low                                | Low                                | Varies by transport             | Low                             |
| Complexity                  | Moderate (developer focused)      | Low–Medium (end-user friendly)     | High (complex config)           | Medium                          |

---

## 🛠 Use Cases

- Off-grid communications during disasters
- Long-range outdoor expeditions
- Rural IoT networks
- Industrial telemetry without cellular coverage
- Educational projects on mesh networking

---

## ✅ Strengths

- Lightweight and modular
- Multi-transport flexibility
- Strong encryption support
- Resilient to single-node failure
- Better developer control than Meshtastic

---

## ❌ Weaknesses

- Smaller user community than Meshtastic
- Requires embedded programming skills
- Limited polished app ecosystem
- Not optimized for high-throughput data

---

## 🔧 Compatible Items

- Heltec V3 LoRa32
- Heltec T114
- LilyGo T-Deck / T3S3
- Seeed Studio T1000-E
- RAK4631
- Xiao S3 / C3 boards
- Station G2

---

## 📚 Related Concepts

- [[LoRa]]
- [[Meshtastic]]
- [[Reticulum]]
- [[BLE Mesh]]
- [[Mesh Networking]]
- [[Off-grid Communication]]

---

## 🌐 External Resources

- [MeshCore Project Website](https://meshcore.co.uk)
- [MeshCore GitHub Repository](https://github.com/meshcore)
- [CNX Software Overview](https://www.cnx-software.com/2025/07/19/meshcore-lightweight-alternative-to-meshtastic-for-lora-based-off-grid-messaging/)
- [Broken Signal Technical Overview](https://brokensignal.tv/pages/what_is_meshcore.html)
- [Nordic Semiconductor Mesh SDK](https://developer.nordicsemi.com/)

---
