# IEEE 802.15.4

IEEE 802.15.4 is a low-rate wireless personal area network (LR-WPAN) standard that defines the physical (PHY) and media access control (MAC) layers for low-power, short-range communications. It serves as the foundation for higher-level protocols like [[Zigbee]], [[Thread]], and [[6LoWPAN]].

---

## ⚙️ Overview

First ratified in 2003 by the IEEE, 802.15.4 targets embedded and IoT applications that require low data rates, low power consumption, and robust operation in noisy environments. It operates in multiple unlicensed frequency bands and supports star, peer-to-peer, and mesh topologies through higher-layer protocols.

---

## 🧠 Core Concepts

- **PHY Layer** – Defines modulation, data rates, and frequency bands.
- **MAC Layer** – Handles frame formatting, addressing, channel access (CSMA/CA), and acknowledgments.
- **Frequency Bands** – 868 MHz (EU), 915 MHz (US), 2.4 GHz (global).
- **Data Rates** – From 20 kbps (868 MHz) to 250 kbps (2.4 GHz).
- **Device Types**:
  - **Full-Function Device (FFD)** – Can serve as coordinator or router.
  - **Reduced-Function Device (RFD)** – Simple, end-node devices.
- **Addressing** – 16-bit short addresses or 64-bit extended IEEE addresses.

---

## 📊 Comparison Chart

| Feature                  | IEEE 802.15.4 | BLE           | Wi-Fi (802.11) | LoRa          | Z-Wave        |
|--------------------------|---------------|---------------|----------------|---------------|---------------|
| Typical Range            | 10–100 m      | 10–100 m      | 30–100 m       | 2–15 km       | 30–100 m      |
| Data Rate                | ≤ 250 kbps    | ≤ 2 Mbps      | 11–9600 Mbps   | ≤ 50 kbps     | ≤ 100 kbps    |
| Power Consumption        | Very Low      | Low           | Medium–High    | Very Low      | Low           |
| Topology                 | Star/mesh via upper layers | Star/Mesh | Star | Star/Mesh    | Mesh          |
| Common Protocols on Top  | Zigbee, Thread, 6LoWPAN | GATT, BLE Mesh | TCP/IP | LoRaWAN | Z-Wave protocol |
| Licensed Spectrum        | No            | No            | No             | No            | No            |

---

## 🛠 Use Cases

- Home automation networks (Zigbee, Thread devices)
- Industrial sensor networks
- Smart metering and utility monitoring
- Agricultural IoT deployments
- Building environmental control systems

---

## ✅ Strengths

- Very low power consumption
- Global unlicensed spectrum support
- Flexible topology (via higher-layer protocols)
- Strong ecosystem support through Zigbee, Thread, etc.

---

## ❌ Weaknesses

- Low data rate unsuitable for high-bandwidth applications
- Requires upper-layer protocol for full networking features
- Limited range without mesh extensions

---

## 🔧 Compatible Items

- [[CC2530]] (TI IEEE 802.15.4 SoC)
- [[nRF52840]] (supports 802.15.4 and BLE)
- [[Atmel AT86RF233]] transceiver
- [[Silicon Labs EFR32MG]]
- [[OpenThread]]-capable devices

---

## 📚 Related Concepts

- [[Zigbee]]
- [[Thread]]
- [[6LoWPAN]]
- [[LoRa]]
- [[Mesh Networking]]
- [[IoT Protocols]]

---

## 🌐 External Resources

- [IEEE 802.15.4 Standard Overview – IEEE](https://standards.ieee.org/standard/802_15_4-2020.html)
- [OpenThread Documentation](https://openthread.io)
- [TI SimpleLink 802.15.4 Stack](https://www.ti.com/wireless-connectivity/simplelink-solutions/overview.html)
- [NXP 802.15.4 Transceivers](https://www.nxp.com/wireless)

---
