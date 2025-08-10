# Meshtastic

Meshtastic is an open-source, long-range mesh communication platform based on [[LoRa]] radios. It enables text and GPS messaging without reliance on cellular, Wi-Fi, or internet infrastructure. Nodes automatically form a mesh network to extend coverage in off-grid, emergency, or remote environments.

---

## ⚙️ Overview

Originating as a community-driven project, Meshtastic is designed for resilience and portability. Devices—often small ESP32-based LoRa boards—connect to phones or computers via Bluetooth or USB, allowing users to send encrypted messages over a mesh network. The system is optimized for long battery life and field use, with global adoption among outdoor enthusiasts, emergency responders, and IoT experimenters.

---

## 🧠 Core Concepts

- **LoRa Physical Layer** – Long-range, low-power wireless modulation.
- **Mesh Networking** – Multi-hop message relaying for extended range.
- **Encryption** – AES-256 end-to-end encryption between nodes.
- **Device Types** – Portable handhelds, fixed base stations, trackers.
- **Connectivity** – Bluetooth or USB to companion apps.
- **Self-Healing Network** – Automatic route optimization and fault tolerance.

---

## 📊 Comparison Chart

| Feature                  | Meshtastic     | [[BLE Mesh]]     | [[Zigbee]]     | [[Thread]]     | [[Wi-Fi HaLow]] | [[LoRaWAN]]     |
|--------------------------|----------------|------------------|----------------|----------------|-----------------|-----------------|
| Typical Range            | 2–20 km        | 10–100 m         | 10–100 m       | 10–100 m       | 100–1000 m      | 2–15 km         |
| Data Rate                | ≤ 300 bps      | ≤ 2 Mbps         | ≤ 250 kbps     | ≤ 250 kbps     | ≤ 78 Mbps       | ≤ 50 kbps       |
| Power Consumption        | Very Low       | Low              | Low            | Low            | Medium          | Very Low        |
| Topology                 | Mesh           | Mesh             | Mesh           | Mesh           | Star/Mesh       | Star/Mesh       |
| Internet Dependency      | No             | Optional         | Optional       | Optional       | Optional        | Yes (for WAN)   |
| Encryption               | AES-256        | AES-CCM          | AES-128        | AES-128        | WPA3/WPA2       | AES-128         |
| Hardware Cost (avg)      | $20–$60        | Varies           | $5–$30         | $5–$30         | $30–$100        | $20–$50         |

---

## 🛠 Use Cases

- Emergency/disaster communications
- Outdoor recreation (hiking, biking, boating)
- Off-grid community networking
- Wildlife and environmental telemetry
- Remote research stations

---

## ✅ Strengths

- Fully open-source software and firmware
- Long range with minimal power use
- Low-cost, off-the-shelf hardware support
- No reliance on external infrastructure
- Strong community development

---

## ❌ Weaknesses

- Very low bandwidth
- Higher latency in multi-hop routing
- Dependent on node density for reliability
- Limited payload size for messages

---

## 🔧 Compatible Items

- [[Heltec LoRa 32 V3]]
- [[TTGO T-Beam]]
- [[T-Echo]]
- [[Raspberry Pi]] (via USB LoRa dongles)
- [[ESP32]] boards with LoRa modules

---

## 📚 Related Concepts

- [[LoRa]]
- [[LoRaWAN]]
- [[Mesh Networking]]
- [[BLE Mesh]]
- [[Zigbee]]
- [[Thread]]

---

## 🌐 External Resources

- [Meshtastic Official Site](https://meshtastic.org)
- [Meshtastic GitHub Repository](https://github.com/meshtastic)
- [Community Forum](https://meshtastic.discourse.group)
- [Hardware Buying Guide](https://meshtastic.org/docs/hardware)

---
