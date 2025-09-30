# 6LoWPAN

**6LoWPAN (IPv6 over Low-Power Wireless Personal Area Networks)** is a networking adaptation layer that enables IPv6 packets to be sent over low-power, low-data-rate wireless networks, particularly those based on IEEE 802.15.4. It is widely used in IoT (Internet of Things), sensor networks, and robotics where constrained devices need efficient, scalable communication.

---

## ⚙️ Overview

6LoWPAN bridges the gap between the resource-heavy IPv6 protocol and constrained wireless devices with limited bandwidth and processing power. By compressing and fragmenting IPv6 packets, it makes it feasible to run IP-based communication directly on low-power networks. This enables seamless integration of embedded devices into the wider Internet.

---

## 🧠 Core Concepts

- **Adaptation Layer**: Adds a layer between IPv6 and IEEE 802.15.4 MAC/PHY
- **Header Compression**: Reduces IPv6 header overhead from 40 bytes down to a few bytes
- **Fragmentation**: Splits IPv6 packets into smaller frames suitable for 802.15.4
- **Mesh Networking**: Supports multi-hop routing in low-power networks
- **Energy Efficiency**: Designed for devices with constrained power budgets
- **Interoperability**: Enables IoT devices to be first-class IP citizens

---

## 🏆 Key Features

- IPv6 connectivity over IEEE 802.15.4
- Header compression and fragmentation
- Low-power and resource-efficient operation
- Supports star and mesh network topologies
- Standardized by the IETF

---

## 📊 Comparison Chart

| Feature                | 6LoWPAN                  | Zigbee                 | Thread                 | Bluetooth Low Energy (BLE) | Wi-Fi HaLow (802.11ah) |
|------------------------|--------------------------|------------------------|------------------------|----------------------------|------------------------|
| **Based On**           | IPv6 over 802.15.4       | 802.15.4 w/ Zigbee NWK | 6LoWPAN + IPv6         | Bluetooth stack            | Wi-Fi PHY/MAC          |
| **IP Support**         | Yes (IPv6)               | No (proprietary)       | Yes (IPv6, mesh)       | No (proprietary stack)     | Yes (IPv4/IPv6)        |
| **Range**              | 10–100 m                 | 10–100 m               | 10–100 m               | ~50 m                      | Up to 1 km             |
| **Data Rate**          | 250 kbps (802.15.4)      | 250 kbps               | 250 kbps               | 125 kbps – 2 Mbps          | Up to 347 Mbps         |
| **Power Efficiency**   | High                     | High                   | High                   | High                       | Moderate               |
| **Topology**           | Star, Mesh               | Mesh                   | Mesh                   | Star                       | Star                   |
| **Use Cases**          | IoT, WSN, Robotics       | Home automation        | Smart home, IoT        | Wearables, peripherals     | Industrial IoT         |

---

## 🛠️ Use Cases

- Wireless Sensor Networks (WSN)
- Robotics swarms and distributed sensing
- Smart agriculture and environmental monitoring
- Smart buildings and home automation
- Industrial IoT and asset tracking
- Healthcare and medical devices

---

## ✅ Strengths

- Brings IPv6 to constrained devices
- Seamless integration with IP-based networks
- Low energy consumption
- Standardized by IETF and widely supported
- Supports both star and mesh topologies

---

## ❌ Weaknesses

- Limited data rate (250 kbps)
- Higher latency compared to Wi-Fi
- Requires adaptation for existing IPv6 stacks
- More complex than proprietary IoT protocols for beginners

---

## 🔧 Compatible Items

- [[IEEE 802.15.4]]
- [[Zigbee]]
- [[Thread]]
- [[IoT Devices]]
- [[Wireless Sensor Networks]]
- [[Robotics Wireless Communication]]

---

## 📚 Related Concepts

- [[IPv6]]
- [[LoRa]]
- [[BLE]] (Bluetooth Low Energy)
- [[Wi-Fi]]
- [[Mesh Networking]]
- [[IoT]]

---

## 🌐 External Resources

- [IETF RFC 4944 – Transmission of IPv6 Packets over IEEE 802.15.4 Networks](https://datatracker.ietf.org/doc/html/rfc4944)
- [IETF 6LoWPAN Working Group](https://datatracker.ietf.org/wg/6lowpan/documents/)
- [Thread Group](https://www.threadgroup.org/)

---

## 📝 Summary

6LoWPAN enables IPv6 communication over constrained wireless networks, making it possible to connect small, low-power devices directly to the Internet. By leveraging header compression and fragmentation, it allows resource-constrained nodes to participate in IP-based ecosystems. This makes 6LoWPAN a foundational technology for IoT, robotics, and sensor network applications where scalability and interoperability are essential.
