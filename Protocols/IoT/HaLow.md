# HaLow (Wi-Fi HaLow / IEEE 802.11ah)

**Wi-Fi HaLow** (IEEE 802.11ah) is a low-power, long-range extension of Wi-Fi designed for IoT, robotics, and industrial applications. Operating in the sub-1 GHz spectrum, HaLow supports energy-efficient communication with better penetration through obstacles compared to traditional Wi-Fi bands (2.4/5 GHz).

---

## ⚙️ Overview

HaLow extends Wi-Fi into the sub-GHz ISM bands (typically 750–950 MHz), enabling kilometer-scale coverage while maintaining compatibility with IP-based networking. It is ideal for IoT sensors, robotics fleets, and low-bandwidth control systems. Compared to Bluetooth or Zigbee, HaLow supports larger networks and higher throughput.

---

## 🧠 Core Concepts

- **Sub-1 GHz Band**: Longer range, better wall penetration.
- **Low Power Consumption**: Optimized for battery-powered devices.
- **IoT Scalability**: Supports thousands of devices per access point.
- **Wi-Fi Compatibility**: Uses standard IP networking, unlike some IoT-specific protocols.
- **OFDM & Narrow Channels**: Flexible channel widths (1–16 MHz) for range/throughput trade-offs.

---

## 📊 Comparison with Other Wireless Standards

| Feature                   | Wi-Fi HaLow (802.11ah) | Classic Wi-Fi (802.11n/ac/ax) | Bluetooth LE | Zigbee | LoRaWAN |
|----------------------------|------------------------|-------------------------------|--------------|--------|---------|
| Frequency Band             | Sub-1 GHz (750–950 MHz)| 2.4/5/6 GHz                   | 2.4 GHz      | 2.4 GHz| Sub-1 GHz |
| Range                      | Up to 1 km+           | ~100 m indoors                | ~10–30 m     | ~100 m | 2–15 km |
| Data Rate                  | 150 kbps – 15 Mbps    | 10 Mbps – Gbps                | ~1 Mbps      | ~250 kbps | ~0.3–50 kbps |
| Devices per Access Point   | 8000+                 | ~200                          | ~10–20       | ~65k   | Very High |
| Power Efficiency           | High                  | Medium–Low                    | Very High    | High   | Very High |
| IP Stack Support           | ✅ Yes                | ✅ Yes                         | ❌ Limited   | ❌ No  | ❌ No  |

---

## 🔧 Use Cases

- **Robotics**: Fleet coordination in warehouses, long-range telemetry.
- **Smart Agriculture**: Sensor networks covering large fields.
- **Smart Cities**: Parking sensors, streetlight control, environmental monitoring.
- **Industrial IoT**: Machine monitoring, predictive maintenance.
- **Healthcare IoT**: Remote monitoring with low power needs.

---

## ✅ Strengths

- Long-range wireless communication.
- Standard Wi-Fi/IP stack compatibility.
- Low energy consumption suitable for IoT.
- Supports massive device scalability.
- Operates in less congested sub-GHz bands.

---

## ❌ Weaknesses

- Lower throughput compared to modern Wi-Fi.
- Hardware availability is still limited compared to mainstream Wi-Fi.
- Competes with LoRaWAN, Zigbee, and Bluetooth for IoT dominance.
- Regional regulatory restrictions (frequency varies by country).

---

## 🔗 Related Concepts

- [[IoT]]
- [[OpenWrt]] (supports Wi-Fi HaLow on some hardware)
- [[Mesh Networking]]
- [[Meshtastic]
- [[Robotics and Industrial Protocols]]
- [[LoRa]]
- [[Zigbee]]

---

## 🧩 Compatible Items

- Wi-Fi HaLow chipsets (e.g., Morse Micro, Newracom)
- IoT devices with sub-1 GHz radios
- Routers/Access Points supporting 802.11ah
- Embedded Linux systems ([[Raspberry Pi]], [[BeagleBone]]) with HaLow modules

---

## 📚 External Resources

- [Wi-Fi Alliance – HaLow Overview](https://www.wi-fi.org/discover-wi-fi/wi-fi-halow)
- [IEEE 802.11ah Standard](https://standards.ieee.org)
- Morse Micro and Newracom developer kits
- OpenWrt support for 802.11ah drivers
- https://www.youtube.com/watch?v=ofR7GFNZzJY

---

## 🧰 Developer Tools

- OpenWrt builds with 802.11ah support
- Linux wireless tools (`iw`, `hostapd`)
- Packet sniffers and analyzers (Wireshark with HaLow support)
- Vendor SDKs for HaLow chipsets

---

## 🌟 Key Highlights

- Extends Wi-Fi into sub-1 GHz for long-range, low-power IoT.
- Supports thousands of devices per AP.
- Bridges the gap between short-range IoT (Bluetooth/Zigbee) and ultra-long-range LPWAN (LoRaWAN).
- Emerging technology with growing chipset and router support.
