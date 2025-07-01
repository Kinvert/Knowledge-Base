# 📡 LTE-M (Long Term Evolution for Machines)

**LTE-M** (also known as Cat-M1) is a low-power wide-area (LPWA) cellular technology developed by 3GPP in Release 13, designed specifically for Internet of Things (IoT) applications requiring moderate data throughput, mobility, and extended battery life.

---

## 🧠 Summary

- Operates on licensed LTE bands.
- Fully supports cellular handover and voice (VoLTE).
- Higher throughput than [[NB-IoT]] with lower latency.
- Designed for both stationary and mobile IoT devices.

---

## ⚙️ Technical Overview

| Feature                 | Value                                      |
|------------------------|--------------------------------------------|
| Standard               | 3GPP Release 13 (LTE Cat-M1)               |
| Bandwidth              | 1.4 MHz                                    |
| Max Data Rate          | ~375 Kbps DL / ~1 Mbps UL (theoretical)    |
| Latency                | ~100–150 ms (much lower than NB-IoT)       |
| Mobility Support       | Full cell handover                         |
| Voice Support          | Yes (VoLTE)                                |
| Power Saving Features  | PSM, eDRX                                  |
| Modulation             | QPSK / 16QAM                               |

---

## 📦 Typical Use Cases

- Asset tracking (vehicles, containers)
- Wearables and smartwatches
- Health monitoring devices
- Alarm systems
- Smart city and smart grid infrastructure

---

## 🏆 Strengths

- **Mobile-friendly**: Supports handovers for moving assets.
- **VoLTE capable**: Can support voice communication.
- **Lower latency**: Good for more interactive applications.
- **Broad support**: Supported by many major carriers globally.

---

## ⚠️ Weaknesses

- **Slightly higher power consumption** than NB-IoT.
- **More complex hardware** required compared to NB-IoT.
- **More expensive modules** on average.
- **Higher spectrum requirements** (1.4 MHz vs 180 kHz for NB-IoT).

---

## 🔄 LTE-M vs Other LPWAN Technologies

| Feature              | LTE-M             | [[NB-IoT]]         | [[LoRaWAN]]        | Sigfox           |
|----------------------|-------------------|--------------------|--------------------|------------------|
| Spectrum             | Licensed           | Licensed           | Unlicensed ISM     | Unlicensed ISM   |
| Max Data Rate        | ~1 Mbps            | ~66 Kbps           | ~50 Kbps           | ~100 bps         |
| Mobility             | Full handover      | Limited             | Poor               | Poor             |
| Latency              | Low                | High                | Medium             | High             |
| VoLTE Support        | Yes                | No                  | No                 | No               |
| Battery Life         | 2–5 years          | 5–10 years          | 5–10 years         | 5–10 years       |
| Best Use Case        | Wearables, Tracking| Smart Meters        | Rural Sensors      | Legacy telemetry |

---

## 📦 Popular LTE-M Modules

| Module                | LTE-M | NB-IoT | GNSS | MCU Onboard | Notes                               |
|----------------------|-------|--------|------|--------------|-------------------------------------|
| **SIM7000G**         | ✅    | ✅     | ✅   | ❌           | Hobbyist-friendly                   |
| **nRF9160**          | ✅    | ✅     | ✅   | Cortex-M33   | Full SoC solution                   |
| **Quectel BG96**     | ✅    | ✅     | ✅   | ❌           | Widely used, robust module          |
| **Murata 1SC**       | ✅    | ✅     | ✅   | ❌           | Compact, certified                  |
| **u-blox SARA-R4**   | ✅    | ❌     | ✅   | ❌           | LTE-M only, voice support           |

---

## 🧪 Development Tips

- Verify your **SIM card supports LTE-M**, not all LTE SIMs do.
- Use **eDRX and PSM** to reduce power usage in sleep mode.
- Evaluate firmware/driver support for **modem AT commands**.
- Be aware of regional LTE bands (e.g., Band 12, 13, 20, etc).
- Consider tools like **[[CVAT]]** and **[[MQTT]]** for sensor data labeling and transmission.

---

## 🔗 Related Topics

- [[NB-IoT]]
- [[LoRa]]
- [[MQTT]]
- [[IoT]]
- [[Murata GM02SP]]
- [[Walter]]

---

## 🌐 External References

- [LTE-M Overview (GSMA)](https://www.gsma.com/iot/lte-m/)
- [nRF9160 by Nordic](https://www.nordicsemi.com/Products/nRF9160)
- [SIMCom SIM7000 Series](https://simcom.ee/modules/lte/sim7000g/)
- [u-blox LTE Modules](https://www.u-blox.com/en/lp/lte-cat-m1-nb1)

---
