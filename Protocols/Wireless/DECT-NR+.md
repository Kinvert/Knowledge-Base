# DECT-NR+ (Digital Enhanced Cordless Telecommunications – New Radio Plus)

DECT-NR+ is a wireless communication standard designed for industrial, IoT, and robotics applications, extending the original DECT standard to support massive machine-type communications (mMTC) and ultra-reliable low-latency communication (URLLC). It operates in a dedicated, license-free frequency band and is standardized by ETSI. DECT-NR+ is optimized for private networks, enabling secure, scalable, and interference-resilient connectivity for autonomous systems, industrial control, and sensor networks.

---

## ⚙️ Overview

DECT-NR+ modernizes DECT by incorporating 5G NR principles, delivering deterministic performance without depending on public cellular infrastructure. It supports mesh topologies, high node density, and predictable latency, making it suitable for robotics fleets, smart factories, and safety-critical automation.

---

## 📚 Core Concepts

- **Private spectrum** – Operates in globally harmonized 1.9 GHz band, reducing interference from common IoT bands (2.4 GHz, 868 MHz).
- **Deterministic scheduling** – Ensures time-sensitive data delivery.
- **Scalability** – Supports thousands of devices per network cell.
- **Security** – AES-based encryption with mutual authentication.
- **5G NR alignment** – Uses physical layer and MAC design principles from 5G.
- **Mesh networking** – Optional for extended coverage and redundancy.

---

## 📊 Comparison Chart – Protocol Landscape

| Protocol / Standard         | Frequency Band       | Max Range       | Max Data Rate | Latency     | Mesh Support | Security Level | Typical Use Case |
|-----------------------------|----------------------|-----------------|---------------|-------------|--------------|----------------|------------------|
| **DECT-NR+**                | 1.9 GHz              | ~1 km+          | ~10 Mbps      | <5 ms       | Yes          | High           | Industrial IoT, robotics fleets |
| [[XBee]] (Zigbee)           | 2.4 GHz              | ~100 m          | 250 kbps      | ~30 ms      | Yes          | Medium         | Sensor networks, home automation |
| [[Meshtastic]] (LoRa)       | 915/868 MHz          | 2–15 km         | ~100 kbps     | High        | Yes          | Medium         | Long-range text, GPS mesh |
| [[Meshcore]]                | 2.4 GHz              | ~2 km           | ~2 Mbps       | Low         | Yes          | Medium-High    | Tactical mesh networks |
| LoRaWAN                     | 915/868 MHz          | 2–15 km         | ~50 kbps      | High        | No           | Medium         | Remote sensing |
| [[Zigbee]]                  | 2.4 GHz              | ~100 m          | 250 kbps      | ~30 ms      | Yes          | Medium         | Home & building automation |
| Thread                      | 2.4 GHz              | ~100 m          | 250 kbps      | Low         | Yes          | Medium         | Smart home mesh |
| Wi-SUN                      | Sub-GHz              | 1–5 km          | ~300 kbps     | Medium      | Yes          | High           | Utility networks |
| [[BLE Mesh]]                | 2.4 GHz              | ~100 m/node     | ~2 Mbps       | Low         | Yes          | High           | Wearables, lighting |
| Wi-Fi 6 (802.11ax)          | 2.4/5/6 GHz          | ~100 m          | Gbps          | Low         | No*          | High           | High-bandwidth robotics |
| NR-U (5G in unlicensed)     | 5 GHz, 6 GHz         | ~1 km           | Gbps          | Very Low    | No           | High           | Private 5G |

\*Wi-Fi mesh possible with vendor-specific implementations.

---

## 📊 Secondary Comparison – DECT-NR+ vs Legacy DECT

| Feature                     | DECT (Legacy)        | DECT-NR+        |
|-----------------------------|----------------------|-----------------|
| Frequency Band              | 1.9 GHz              | 1.9 GHz         |
| Max Data Rate               | ~1 Mbps              | ~10 Mbps        |
| Mesh Support                | No                   | Yes             |
| 5G NR Principles            | No                   | Yes             |
| Security                    | Basic                | AES + Mutual Auth |
| Industrial IoT Ready        | No                   | Yes             |

---

## 🛠 Use Cases

- Autonomous mobile robot fleet coordination
- Industrial automation and SCADA
- Distributed sensor and actuator networks
- Private 5G-like campus deployments
- Smart manufacturing lines with deterministic control loops

---

## ✅ Strengths

- Operates in interference-resistant 1.9 GHz band
- Designed for ultra-reliable, low-latency communication
- Scales well for dense device deployments
- Built-in strong encryption and authentication
- Does not require licensed spectrum or public carrier dependency

---

## ⚠️ Limitations

- Less ubiquitous hardware availability compared to Wi-Fi or BLE
- Limited consumer ecosystem (industrial focus)
- Requires compatible DECT-NR+ chipsets

---

## 🔗 Related Concepts / Notes

- [[XBee]] (Zigbee/802.15.4 modules)
- [[LoRa]]
- [[Mesh Networking]]
- [[Wi-Fi 6]]
- [[BLE Mesh]]
- [[5G NR]]
- [[Thread]]
- [[SCADA]]

---

## 🧩 Compatible Items

- DECT-NR+ base stations and repeaters
- Industrial IoT gateways with DECT-NR+ modules
- Robotics controllers supporting URLLC over DECT-NR+

---

## 🗂 Variants

- **DECT-NR+** – Optimized for industrial IoT and robotics
- **DECT-ULE** – Ultra Low Energy DECT variant for home automation

---

## 📄 External Resources

- ETSI DECT-NR+ Standard: https://www.etsi.org
- DECT Forum: https://www.dect.org
- 5G-MAG Technical Overview: https://www.5g-mag.com

---

## 🛠 Developer Tools

- ETSI standard specifications and API references
- Vendor SDKs for DECT-NR+ modules
- Network simulation tools for mesh and URLLC testing

---

## 📚 Documentation and Support

- ETSI Technical Documentation
- Vendor application notes
- Industry consortium white papers

---
