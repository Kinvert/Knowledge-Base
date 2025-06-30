# 📷 Caddx Infra V2

The **Caddx Infra V2** is a compact thermal camera module designed primarily for FPV drones, embedded systems, and small robotics platforms. It provides thermal imaging capabilities with onboard processing and easy integration, making it accessible for hobbyists and professionals alike.

---

## 🧠 Summary

- Compact thermal camera module with analog and digital outputs.
- Designed for lightweight applications like FPV drones.
- Based on the **Infiray 256x192 thermal sensor**.
- Offers **pseudo-color thermal visualization** and **multiple color palettes**.
- Compatible with standard FPV equipment and some embedded platforms.

---

## 🔧 Key Specifications

| Feature                 | Specification                                     |
|------------------------|---------------------------------------------------|
| **Sensor**             | Infiray 256x192 thermal sensor                    |
| **Resolution**         | 256 × 192                                         |
| **Frame Rate**         | 25 Hz                                             |
| **Output**             | Analog video (CVBS) and digital UART              |
| **Lens**               | 13mm or 19mm FOV lens options                     |
| **Weight**             | ~10g                                              |
| **Dimensions**         | ~19x19x20 mm (varies slightly by version)         |
| **Power Supply**       | 5V typical (compatible with drone power rails)    |
| **Interface**          | UART for digital control, OSD config support      |
| **Mounting**           | 19x19mm mounting hole spacing (typical of FPV cams) |

---

## 🎨 Visualization Options

- Supports multiple color palettes:
  - White hot
  - Black hot
  - Ironbow
  - Rainbow
- Can switch palettes via OSD menu or UART.

---

## ⚙️ Typical Use Cases

- FPV night flying
- Thermal inspection (pipes, HVAC, etc.)
- Search and rescue drones
- Basic robotics thermal awareness
- Education and prototyping in embedded vision

---

## 🧪 Integration Notes

- **Analog Output** can be connected directly to analog video receivers (VTX for FPV).
- **UART Control** allows you to interface with microcontrollers or SBCs for advanced settings.
- Can be paired with a **video capture card** or **AV-to-USB adapter** for use with a PC.
- Works well with **OSD controllers**, such as BetaFlight and iNav systems.

---

## 🔗 Related Notes

- [[Infrared Cameras]]
- [[Thermal Imaging]]
- [[FPV Systems]]
- [[UART]]
- [[MIPI CSI-2 Protocol]]
- [[sensor_msgs]]

---

## ⚠️ Pros & Cons

### ✅ Pros

- Lightweight and compact
- Affordable for a thermal imager
- Easy to integrate into drone/embedded setups
- Multiple visualization modes

### ❌ Cons

- Low thermal resolution compared to industrial cameras
- Limited API access for advanced image processing
- Analog output requires capture conversion for digital systems

---

## 🔄 Comparison Table

| Feature                | Caddx Infra V2       | FLIR Lepton 3.5        | Seek Thermal Compact    | InfiRay P2 Pro            |
|------------------------|----------------------|-------------------------|--------------------------|----------------------------|
| Resolution             | 256×192              | 160×120                 | 320×240                  | 256×192                    |
| Frame Rate             | 25 Hz                | <9 Hz (due to export limits) | <15 Hz               | 25 Hz                      |
| Output Type            | Analog / UART        | SPI                     | USB-C                    | USB-C                      |
| Integration            | FPV drones, UART MCU | Microcontrollers        | Smartphone               | Smartphone / PC            |
| Price (approx.)        | ~$150–$200           | ~$200                   | ~$250                    | ~$250                      |
| API / Dev Support      | Limited              | Good (via SDK)          | Basic SDK                | App-driven, limited SDK    |

---

## 🌐 External Resources

- [Caddx Official Product Page](https://www.caddxfpv.com/products/caddx-infra-v2)
- [User Reviews and Benchmarks](https://www.youtube.com/results?search_query=caddx+infra+v2)
- [Integration Example – BetaFlight Setup](https://oscarliang.com/thermal-camera-fpv/)

---
