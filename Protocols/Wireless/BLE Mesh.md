# BLE Mesh (Bluetooth Low Energy Mesh Networking)

BLE Mesh is an extension of the Bluetooth Low Energy standard that enables many-to-many (m:m) device communication over a mesh topology. It allows BLE devices to relay messages through intermediate nodes, vastly extending coverage and enabling robust, large-scale networks without requiring high power consumption.

---

## ⚙️ Overview

Introduced by the Bluetooth SIG in 2017, BLE Mesh was designed for large networks such as industrial automation, smart buildings, and sensor arrays. Unlike point-to-point or star topologies in standard BLE, mesh networking uses a flooding-based message relay system where nodes forward messages to others, ensuring data reaches all relevant devices.

---

## 🧠 Core Concepts

- **Nodes** – Devices participating in the mesh.
- **Relay Nodes** – Forward messages to extend network coverage.
- **Friend Nodes & Low Power Nodes (LPN)** – Friend nodes store messages for battery-powered LPNs to retrieve later.
- **Publish/Subscribe Model** – Devices publish messages to specific addresses and subscribe to messages of interest.
- **Managed Flooding** – Messages propagate via multiple hops without complex routing tables.
- **Provisioning** – Secure process of adding devices to a mesh network.

---

## 📊 Comparison Chart

| Feature                  | BLE Mesh | Standard BLE | Zigbee Mesh | Thread | Wi-Fi Mesh |
|--------------------------|----------|--------------|-------------|--------|------------|
| Topology                 | Mesh     | Star         | Mesh        | Mesh   | Mesh       |
| Range per Node           | 10–100 m | 10–100 m     | 10–100 m    | 10–100 m | ~30 m |
| Coverage Expansion       | Yes (multi-hop) | No  | Yes | Yes | Yes |
| Power Consumption        | Low      | Very Low     | Low         | Low    | High |
| Scalability              | 1000s of nodes | <8 devices typical | 100s | 100s | 10s–100s |
| Throughput               | Low      | Low–Medium   | Low         | Low    | Medium–High |
| Target Applications      | IoT, building automation | Wearables, peripherals | Home automation | IPv6 IoT | Home networking |

---

## 🛠 Use Cases

- Smart lighting systems in large buildings
- Distributed environmental sensing (temperature, air quality)
- Industrial equipment monitoring
- Multi-room access control systems
- Robot swarm coordination

---

## ✅ Strengths

- Large network scalability (up to thousands of nodes)
- Robust against single node failures
- Low power consumption suitable for battery-powered nodes
- Interoperability across BLE Mesh-certified devices

---

## ❌ Weaknesses

- Higher latency than point-to-point BLE
- Lower throughput due to message flooding
- Requires provisioning and configuration overhead
- Not ideal for large continuous data transfers

---

## 🔧 Compatible Items

- [[ESP32]] (with BLE Mesh firmware support)
- [[nRF52]] series (Nordic BLE SoCs with mesh SDK)
- [[BlueZ]] (Linux stack supporting BLE Mesh)
- [[Zephyr RTOS]] (Built-in BLE Mesh stack)
- [[Silicon Labs EFR32]]

---

## 📚 Related Concepts

- [[BLE]] (Bluetooth Low Energy)
- [[Zigbee]] (Alternative mesh protocol)
- [[Thread]] (IPv6-based mesh networking)
- [[Mesh Networking]] (General mesh concepts)
- [[IoT Protocols]]
- [[GATT]] (Generic Attribute Profile, not used directly in BLE Mesh)

---

## 🌐 External Resources

- [Bluetooth SIG – Mesh Specifications](https://www.bluetooth.com/specifications/mesh-specifications/)
- [Nordic Semiconductor Mesh SDK](https://developer.nordicsemi.com/nRF5_SDK_for_Mesh/)
- [Zephyr Project BLE Mesh Documentation](https://docs.zephyrproject.org/latest/services/bluetooth/mesh.html)

---
