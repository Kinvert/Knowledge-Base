# 🖥️ MIPI Display Interface (DSI)

**MIPI Display Serial Interface (DSI)** is a high-speed, low-power interface standard developed by the MIPI Alliance for connecting displays to host processors in mobile and embedded systems. It is widely used in smartphones, tablets, wearables, and increasingly in embedded systems like SBCs and development boards.

---

## 🧠 Summary

- DSI = **Display Serial Interface**
- Designed for **low power** and **high-speed** communication between processors and displays.
- **Differential signaling** over **MIPI D-PHY** or **C-PHY**.
- Supports resolutions up to and beyond 4K, depending on lane configuration and clock speeds.

---

## 🔧 Technical Details

| Feature                 | Description                                                |
|-------------------------|------------------------------------------------------------|
| Protocol Name           | MIPI DSI (Display Serial Interface)                        |
| MIPI Spec               | **MIPI DSI v1.0–v2.0**, governed by the **MIPI Alliance**  |
| Physical Layer          | **MIPI D-PHY** (most common), also supports C-PHY          |
| Signaling               | **Differential pair**, serialized                         |
| Data Lanes              | 1 to 4 (some implementations support 8+)                   |
| Clock Lane              | 1 dedicated differential pair                              |
| Transfer Mode           | High Speed (HS) & Low Power (LP)                           |
| Max Bandwidth per lane  | ~1 Gbps to over 4.5 Gbps depending on spec                 |
| Topology                | Point-to-point (Host ↔ Display)                            |

---

## 🧰 Common Use Cases

- Smartphone LCD/OLED displays
- Embedded touch screen displays
- Smart watches and AR/VR headsets
- Raspberry Pi and Jetson-based touch panels
- Automotive dashboards and infotainment systems

---

## 📦 Physical Characteristics

- Uses **flat flexible cables (FFC/FPC)** with 15, 22, 30, 40 pins commonly
- Often interfaces via connectors like **FH28**, **FH19**, or **custom board-to-board**

---

## 🧩 DSI vs. HDMI vs. eDP

| Feature             | MIPI DSI             | HDMI                 | eDP (Embedded DP)       |
|---------------------|----------------------|----------------------|-------------------------|
| Target Use          | Mobile, Embedded     | Consumer electronics | Laptops, Embedded       |
| Data Type           | Serialized video     | TMDS (Parallel→Serial)| DisplayPort over internal|
| Max Lanes           | 4–8                  | 3 channels            | Up to 4 lanes (DP lanes) |
| Protocol Layer      | MIPI D-PHY, C-PHY    | TMDS/CEC             | AUX, Main Lane (DP)     |
| Cable Type          | FPC/FFC              | HDMI                 | eDP (internal connector) |
| Power Consumption   | ✅ Low                | ❌ Higher             | ✅ Moderate              |
| Display Types       | LCD, OLED, AMOLED    | LCD, TV, Monitor     | Laptop, internal display |
| Touch Integration   | ✅ Easy               | ❌ Rare               | ✅ Sometimes             |

---

## 🔌 Connector and Cable Notes

- Ribbon cables: typically 0.5mm pitch
- Common with **Raspberry Pi DSI** connectors (15 or 22 pin)
- Touch signals and power often **integrated in same cable**
- Compatible displays must match **voltage and timing** of host (beware mismatched specs)

---

## 📟 Popular SBCs and Devices Supporting MIPI DSI

| Board / Platform         | MIPI DSI Support     | Notes                                             |
|--------------------------|----------------------|---------------------------------------------------|
| **Raspberry Pi 4/5**     | ✅ 1× or 2× 4-lane    | Dual DSI on RPi 5                                 |
| **Jetson Nano / Xavier / Orin** | ✅ 1× or more        | Through expansion headers or board connectors     |
| **BeagleBone AI-64**     | ✅                   | Limited official display support                  |
| **Orange Pi / Radxa**    | ✅                   | Mixed support depending on model                  |
| **Smartphones / Tablets**| ✅                   | Most mobile displays are MIPI DSI                 |

---

## 🧪 Troubleshooting Tips

- **No image**: check DSI pinout, power voltage, and timing compatibility
- **White screen / flashing**: mismatched timing, incorrect init sequence
- **Touch not working**: may use **I2C** or **SPI**, often via separate pins
- **Cable length**: keep short, under 10–15cm ideally to avoid signal loss
- Check **device tree overlays** on Linux-based SBCs (e.g., Raspberry Pi)

---

## ✅ Pros

- Low power
- High throughput
- Flexible integration (e.g., display + touch)
- Small form factor
- Becoming standard for embedded displays

---

## ⚠️ Cons

- Fewer plug-and-play displays available compared to HDMI
- Initialization sequences may vary (sometimes undocumented)
- Board-specific software support can vary

---

## 🔗 Related Notes

- [[MIPI CSI-2 Protocol]]
- [[Raspberry Pi Camera]]
- [[Jetson Family]]
- [[Display Interfaces]]
- [[Embedded Displays]]

---

## 🌐 External Resources

- [MIPI Alliance - DSI Overview](https://www.mipi.org/specifications/dsi)
- [Raspberry Pi DSI Docs](https://www.raspberrypi.com/documentation/computers/display.html)
- Jetson forums, Arducam documentation for DSI-compatible touch displays

---
