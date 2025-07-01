# 📡 NB-IoT (Narrowband Internet of Things)

**NB-IoT (Narrowband IoT)** is a low-power wide-area network (LPWAN) radio technology standardized by 3GPP (Release 13) to enable efficient communication for devices that transmit small amounts of data over long periods and long ranges.

---

## 🧠 Summary

- LPWAN technology using existing LTE infrastructure.
- Designed for applications needing low data throughput, deep coverage, and long battery life.
- Operates on licensed spectrum (unlike LoRa or Sigfox).
- Often bundled with LTE-M in modern modules.

---

## ⚙️ Technical Details

| Feature                     | Value                                      |
|-----------------------------|--------------------------------------------|
| Standard                    | 3GPP Release 13                            |
| Spectrum                    | Licensed LTE bands (e.g., Band 8, 20, 28) |
| Bandwidth                   | 180 kHz                                    |
| Max Data Rate               | ~26 Kbps (DL), ~66 Kbps (UL)              |
| Latency                     | ~1.5 to 10 seconds (not real-time)        |
| Power Consumption           | Extremely low (supports PSM and eDRX)     |
| Device Complexity           | Very low (compared to LTE)                |
| Range                       | Up to 10 km (urban), 35+ km (rural)       |
| Mobility                    | Limited (optimized for stationary nodes)  |

---

## 📦 Typical Use Cases

- Smart meters (water, gas, electricity)
- Environmental sensors
- Parking sensors
- Industrial monitoring
- Agriculture (e.g., soil and crop sensors)
- Smart city infrastructure

---

## 🌍 Global Availability

NB-IoT is supported by many carriers around the world but coverage may vary. It is widely deployed in:

- Europe
- China
- Australia
- Parts of North America (select carriers)

---

## 🏆 Strengths

- Long battery life (up to 10 years)
- Great penetration (e.g. basements, underground)
- Cost-effective modules and low complexity
- Uses licensed spectrum → less interference
- Integrates with LTE base stations

---

## ⚠️ Weaknesses

- Low bandwidth (not for high-throughput needs)
- High latency (not suitable for time-sensitive tasks)
- Not ideal for mobile applications (e.g., tracking moving vehicles)
- Fragmented carrier support (compared to LTE-M)

---

## 🔄 NB-IoT vs Other LPWAN Technologies

| Feature              | NB-IoT          | LTE-M             | LoRaWAN            | Sigfox           |
|----------------------|------------------|-------------------|--------------------|------------------|
| Bandwidth            | Narrow (180 kHz) | 1.4 MHz           | Very narrow (~125 kHz) | Very narrow (~100 Hz) |
| Spectrum             | Licensed         | Licensed          | Unlicensed ISM     | Unlicensed ISM   |
| Mobility             | Stationary       | Mobile            | Poor               | Poor             |
| Latency              | High             | Low               | Variable           | High             |
| Max Data Rate        | ~66 Kbps         | ~1 Mbps           | ~50 Kbps           | ~100 bps         |
| Battery Life         | 5–10 years       | 2–5 years         | 5–10 years         | 5–10 years       |
| Global Adoption      | Moderate         | Moderate          | High (DIY/hobbyist) | Declining        |
| Best Use Case        | Static sensors   | Mobile assets     | Remote sensors     | Legacy telemetry |

---

## 🔌 Example Modules and Chips

| Module               | Cellular         | GNSS | MCU             | Notes                                      |
|---------------------|------------------|------|------------------|--------------------------------------------|
| **Quectel BC95-G**   | NB-IoT only      | ❌   | External needed  | Very low-power and widely deployed         |
| **u-blox SARA-N2**   | NB-IoT only      | ❌   | External needed  | Early NB-IoT offering, now superseded      |
| **SIM7000G**         | LTE-M / NB-IoT   | ✅   | External MCU     | Very popular, hobbyist-friendly            |
| **GM02SP (Murata)**  | LTE-M / NB-IoT   | ✅   | External MCU     | Used in [[Walter]], compact and certified  |
| **Nordic nRF9160**   | LTE-M / NB-IoT   | ✅   | Cortex-M33       | Full SoC, ideal for embedded developers    |

---

## 🧪 Development Tips

- Check SIM card and carrier support for NB-IoT bands.
- Use modules with onboard TCP/IP stack for simpler MCU integration.
- Evaluate power consumption carefully using PSM and eDRX modes.
- Avoid mobile use cases—NB-IoT excels at stationary deployments.
- Consider pairing with MQTT or CoAP for data transmission.

---

## 🔗 Related Topics

- [[LTE-M]]
- [[LoRa]]
- [[IoT]]
- [[Walter]]
- [[Murata GM02SP]]
- [[Quectel Modules]]

---

## 🌐 External References

- [3GPP NB-IoT Specs](https://www.3gpp.org/release-13)
- [NB-IoT Overview from u-blox](https://www.u-blox.com/en/technology/cellular/narrowband-iot-nb-iot)
- [Murata GM02SP Datasheet](https://wireless.murata.com/eng/products/gm02sp.html)

---
