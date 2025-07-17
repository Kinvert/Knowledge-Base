# DigiMesh

🧠 **Overview**  
DigiMesh is a proprietary mesh networking protocol developed by Digi International. It is designed for reliable, low-power, peer-to-peer wireless communications where all nodes can act as routers. Unlike Zigbee’s coordinator/router/end-device model, DigiMesh uses a single firmware that enables every node to be equal, simplifying design and deployment.

⚙️ **Key Features**
- 🔄 **Peer-to-Peer Mesh**: All devices are routers; no central coordinator required.
- 🔋 **Low Power Support**: Sleep functionality available even in routing nodes.
- 📡 **Self-Healing Network**: Nodes dynamically reroute around failures.
- 🛠️ **Over-the-Air (OTA) Configuration**: Supports firmware upgrades and settings adjustment via wireless.
- 📶 **Range and Frequency**: Commonly used on 2.4 GHz and 900 MHz radios (e.g. XBee modules).
- 🔐 **Security**: AES 128-bit encryption support.

📊 **Comparison Table**

| Feature                     | DigiMesh                    | Zigbee                      | Bluetooth Mesh         | Thread                     |
|----------------------------|-----------------------------|-----------------------------|------------------------|----------------------------|
| Topology                   | Mesh (peer-to-peer)         | Mesh (coordinator-based)    | Mesh                   | Mesh                       |
| Coordinator Needed         | ❌ No                        | ✅ Yes                      | ❌ No                  | ✅ Border Router            |
| Node Equality              | ✅ All nodes are routers     | ❌ Roles vary               | ✅ All nodes participate| ❌ Roles vary               |
| Power Saving (on routers)  | ✅ Yes                       | ❌ Limited                  | ✅ Limited             | ✅ Yes                      |
| Protocol Standard          | ❌ Proprietary (Digi)        | ✅ IEEE 802.15.4            | ✅ Bluetooth SIG        | ✅ IETF (IPv6-based)        |
| Interoperability           | ❌ Only Digi devices         | ✅ Zigbee Certified Devices | ✅ BLE devices         | ✅ IP-compatible            |
| Encryption                 | ✅ AES 128-bit               | ✅ AES 128-bit              | ✅ AES-CCM             | ✅ AES                      |
| Firmware Type              | One for all roles           | Different for each role     | Unified               | Role-based images          |

💡 **Pros**
- Simplifies firmware management due to single firmware image.
- All nodes can route and sleep, enabling robust and power-efficient networks.
- Excellent for networks with no fixed infrastructure.

⚠️ **Cons**
- Only works with Digi radios (XBee).
- Not an open standard – vendor lock-in.
- Not supported on as many 3rd-party platforms as Zigbee or Thread.

🔧 **Common Use Cases**
- Agricultural sensor networks
- Industrial automation
- Environmental monitoring
- Remote data logging

🖥️ **Setup & Tools**
- Use **[[XCTU]]** for configuration and testing of DigiMesh-enabled radios.
- Typical modules: **XBee 900HP**, **XBee SX**, **XBee 3 (DigiMesh firmware)**.
- Set PAN ID, channel, and destination addresses for each node in XCTU.

🔁 **Firmware Notes**
- Firmware must be set explicitly to DigiMesh (e.g., `XBP9X-DM`, `XB3-24-DM`) in XCTU.
- OTA updates can be performed when nodes are awake.

🔌 **Power Management**
- Configure sleep modes (`SM` command) in XCTU.
- Use pin or cyclic sleep modes for battery-powered devices.

🔐 **Security Tips**
- Always set an encryption key (`KY` command).
- Disable AT command mode after setup for better protection.

🔍 **Related Notes**
- [[XCTU]]
- [[XBee]]
- [[Zigbee]]
- [[Sensor Networks]]
- [[IoT]]

📚 **References**
- DigiMesh Product Manual: [Digi Official Docs](https://www.digi.com/resources/documentation/digidocs/pdfs/90002173.pdf)
- XCTU Software: [https://www.digi.com/xctu](https://www.digi.com/xctu)
