# LoRa (Long Range)

LoRa is a proprietary wireless modulation technology designed for long-range, low-power communication. It is widely used in IoT applications requiring wide-area connectivity with minimal energy consumption. LoRa forms the physical layer foundation for networks like [[LoRaWAN]] and mesh systems such as [[Meshtastic]] and [[MeshCore]].

---

## ⚙️ Overview

Developed by Semtech, LoRa uses Chirp Spread Spectrum (CSS) modulation to achieve communication distances ranging from several kilometers in urban environments up to tens of kilometers in rural areas. Its low power consumption and robust signal penetration make it ideal for battery-powered devices deployed in remote or harsh conditions.

---

## 🧠 Core Concepts

- **Chirp Spread Spectrum (CSS)** – Modulation technique providing resilience to interference and multipath fading.
- **Spreading Factor (SF)** – Configurable parameter controlling range vs data rate trade-off.
- **Bandwidth** – Typically 125 kHz, 250 kHz, or 500 kHz channels.
- **Adaptive Data Rate (ADR)** – Dynamic adjustment of transmission parameters for optimized performance.
- **Star Topology** – LoRa typically uses star networks where devices communicate with gateways.
- **Unlicensed Spectrum** – Operates in sub-GHz ISM bands (e.g., 868 MHz in Europe, 915 MHz in North America).

---

## 📊 Comparison Chart

| Feature                  | LoRa           | BLE            | Zigbee         | Wi-Fi          | Cellular (LTE-M/NB-IoT) |
|--------------------------|----------------|----------------|----------------|----------------|-------------------------|
| Typical Range            | 2–15+ km       | 10–100 m       | 10–100 m       | 30–100 m       | Up to 10 km             |
| Data Rate                | 0.3–50 kbps    | Up to 2 Mbps   | Up to 250 kbps | Up to Gbps     | 100 kbps–1 Mbps         |
| Power Consumption        | Very Low       | Low            | Low            | High           | Moderate                |
| Topology                 | Star           | Star/Mesh      | Mesh           | Star/Mesh      | Star                    |
| Spectrum                 | Unlicensed Sub-GHz | 2.4 GHz      | 2.4 GHz        | 2.4/5 GHz      | Licensed                |
| Use Cases                | IoT sensors, asset tracking | Wearables, peripherals | Home automation | Video streaming, internet access | Wide-area IoT           |

---

## 🛠 Use Cases

- Smart agriculture sensors
- Environmental monitoring
- Asset tracking and logistics
- Smart cities (lighting, parking, waste management)
- Industrial IoT telemetry

---

## ✅ Strengths

- Very long range for low power
- Robust penetration through buildings and terrain
- Operates in unlicensed bands globally
- Scalable with existing LoRaWAN infrastructure
- Simple, low-cost end devices

---

## ❌ Weaknesses

- Low data throughput
- Star topology requires gateways for wide coverage
- Subject to duty cycle regulations in some regions
- Proprietary modulation limits interoperability outside LoRa ecosystem

---

## 🔧 Compatible Items

- Semtech SX1276/77/78/79 LoRa transceivers
- Heltec LoRa32 boards
- TTGO T-Beam and T-Call
- RAKwireless LoRa modules and gateways
- MultiTech Conduit gateways

---

## 📚 Related Concepts

- [[LoRaWAN]] (Network protocol layer on LoRa)
- [[Meshtastic]] (LoRa mesh network project)
- [[MeshCore]] (LoRa mesh networking stack)
- [[IEEE 802.15.4]] (Alternative low-power protocol)
- [[BLE]] (Bluetooth Low Energy)

---

## 🌐 External Resources

- [Semtech LoRa Technology Overview](https://www.semtech.com/lora)
- [LoRa Alliance](https://lora-alliance.org)
- [The Things Network](https://www.thethingsnetwork.org)
- [LoRaWAN Specification](https://lora-alliance.org/resource-hub/lorawanr-specification-v11)

---
