# RNode

RNode is a series of robust, open-source LoRa (Long Range) radio devices created by unsigned.io, designed to enable resilient, encrypted, and long-distance communication in mesh networks. These devices are especially well-suited for use with [[Meshtastic]] and [[Reticulum]] networking stacks, and they are engineered with higher build quality, greater RF output power, and more durable enclosures than typical hobby-grade LoRa boards. They are often used in scenarios where reliability, portability, and security are critical.

---

## 🛠 Overview
RNode devices integrate high-performance LoRa transceivers with rugged hardware and flexible firmware options, supporting both experimental and field-ready communication solutions. They are available in multiple form factors — handheld, portable micro units, and stationary desktop models — with varying frequency bands and power output levels.

---

## 📚 Core Concepts
- **LoRa Modulation**: Utilizes Chirp Spread Spectrum for long-range, low-power wireless communication.
- **Mesh Networking**: Natively compatible with [[Meshtastic]] and [[Reticulum]] to form peer-to-peer or multi-hop networks.
- **Encryption**: Built-in support for secure communication protocols.
- **Frequency Bands**: Models for 433 MHz, 868 MHz, and 915 MHz, depending on region.
- **High Output Power**: Up to 1 W (with respect to legal limits per region).
- **Field Durability**: Designed for harsh environments with reinforced casing.

---

## 📊 Comparison Chart

| Feature           | RNode                       | [[Heltec LoRa Dev Board]] | [[TTGO T-Beam]]             | [[XBee SX Pro]]           | [[RAK Wireless]] Modules |
|-------------------|-----------------------------|---------------------------|-----------------------------|---------------------------|--------------------------|
| Modulation        | LoRa                        | LoRa                      | LoRa + GPS                  | FHSS                       | LoRa                     |
| Output Power      | Up to 1 W                    | ~100 mW                   | ~100 mW                     | Up to 1 W                  | 14–30 dBm                |
| Frequency Bands   | 433 / 868 / 915 MHz          | 433 / 868 / 915 MHz        | 433 / 868 / 915 MHz          | 900 MHz ISM                | 433 / 868 / 915 MHz      |
| Firmware          | RNode / Meshtastic           | Arduino-based              | Meshtastic / Arduino-based   | Proprietary                 | AT Commands / LoRaWAN    |
| Encryption        | Yes (Reticulum, Meshtastic)  | Optional                   | Yes (Meshtastic)             | Proprietary                 | LoRaWAN security         |
| Durability        | Rugged, field-ready          | Minimalist PCB             | Basic case                   | Industrial-grade            | Industrial-grade         |
| Price Range       | $$$                          | $                          | $$                           | $$$$                        | $$–$$$                   |

---

## 🚀 Use Cases
- Off-grid personal communication networks.
- Emergency and disaster recovery mesh networking.
- Rural and remote area connectivity.
- Amateur radio and experimentation.
- Secure field communications for NGOs and researchers.

---

## ✅ Strengths
- High RF performance and range.
- Ruggedized, reliable hardware.
- Direct support for popular mesh protocols.
- Flexible firmware ecosystem.
- Suitable for professional and hobbyist use.

---

## ⚠️ Limitations
- More expensive than DIY boards.
- Larger and heavier than minimalist modules.
- Requires licensing for high-power operation in some regions.
- Single modulation type (LoRa only).

---

## 🔧 Compatible Items
- [[Meshtastic]] (Open-source mesh firmware)
- [[Reticulum]] (Networking stack)
- [[LoRa]] transceivers and modules
- [[PA]] (Power Amplifier) modules for range extension
- [[Heltec LoRa Dev Board]]
- [[TTGO T-Beam]]

---

## 🗂 Variants
- **RNode Handheld** — Portable with integrated battery and controls.
- **RNode Micro** — Compact USB device for embedding in projects.
- **RNode Desktop** — Stationary unit for network hubs or gateways.
- Frequency-specific models (433, 868, 915 MHz).

---

## 🔍 Related Concepts/Notes
- [[LoRa]] (Long Range wireless modulation)
- [[Meshtastic]] (Open-source LoRa mesh networking)
- [[Reticulum]] (Encrypted networking stack)
- [[PA]] (Power Amplifier)
- [[802.15.4]] (Low-rate WPAN standard)
- [[BLE Mesh]] (Bluetooth Low Energy mesh networking)

---

## 📄 Documentation and Support
- [Official RNode Documentation](https://unsigned.io/rnode)
- [Reticulum Networking Stack](https://reticulum.network/)
- [Meshtastic Project](https://meshtastic.org/)
- Community support via Discord, forums, and GitHub issues.

---

## 📦 External Resources
- Unsigned.io Store: https://unsigned.io
- GitHub: https://github.com/markqvist
- Meshtastic GitHub: https://github.com/meshtastic

---

## 🏆 Key Highlights
- Professionally engineered LoRa hardware.
- Reliable for mission-critical mesh networks.
- Flexible for both professional deployments and hobbyist experimentation.
