# 🌡️ Thermal Cameras

**Thermal cameras** detect infrared radiation (heat) instead of visible light, enabling the visualization of temperature differences across a scene. They are widely used in fields ranging from industrial inspection and firefighting to search and rescue and hobby robotics.

---

## 🧠 Summary

- Detect heat signatures by capturing infrared radiation.
- Useful in complete darkness and for identifying heat anomalies.
- Come in a range of resolutions and form factors.
- Increasingly accessible for hobbyists via modules like FLIR Lepton and Caddx Infra V2.

---

## 🧪 Common Use Cases

- 🔧 Industrial diagnostics (HVAC, electrical panels)
- 🚁 Drone-based search and rescue
- 🦾 Robotics for human detection, heat tracking
- 🔬 Scientific research (wildlife, thermodynamics)
- 🧰 Home energy audits

---

## 🔍 Sensor Characteristics

| Feature              | Description                                                      |
|----------------------|------------------------------------------------------------------|
| **Spectral Range**   | Long-wave infrared (8–14 μm)                                     |
| **Resolution**       | Typically lower than visible light sensors (e.g., 80x60 to 640x512) |
| **Frame Rate**       | Often limited to <9 Hz for export compliance (unless licensed)   |
| **Calibration**      | May require flat-field correction (FFC) for accuracy             |
| **Output Formats**   | Analog video, SPI, USB, UART, MIPI, etc.                         |

---

## 🔗 Related Topics

- [[Caddx Infra V2]]
- [[FLIR Lepton]]
- [[Seek Thermal]]
- [[MIPI CSI-2 Protocol]]
- [[Cameras]]
- [[sensor_msgs]]
- [[UART]]
- [[USB Protocol]]
- [[Edge Computing]]

---

## 🔄 Comparison Table

| Camera               | Resolution     | Interface         | Frame Rate | Price    | Notes                                  |
|----------------------|----------------|-------------------|------------|----------|----------------------------------------|
| **Caddx Infra V2**   | 256×192        | Analog, UART      | 25 Hz      | ~$150    | Lightweight FPV/embedded option        |
| **FLIR Lepton 3.5**  | 160×120        | SPI               | <9 Hz      | ~$200    | Module-based; popular in DIY builds    |
| **Seek Compact Pro** | 320×240        | USB-C             | ~15 Hz     | ~$250    | Phone-based; limited SDK               |
| **InfiRay P2 Pro**   | 256×192        | USB-C             | 25 Hz      | ~$250    | High portability, limited dev access   |
| **FLIR Boson 640**   | 640×512        | USB, MIPI         | 30–60 Hz   | ~$2500+  | Professional-grade with SDK            |

---

## ⚙️ Integration Tips

- **Embedded Use:** Choose UART, SPI, or analog models that work with microcontrollers or SBCs (e.g. Raspberry Pi, Jetson).
- **PC Integration:** Use USB variants or AV-to-USB converters for analog cams.
- **Software:** Look for compatibility with OpenCV, ROS, or vendor SDKs.

---

## ✅ Pros

- Operates in complete darkness
- Ideal for non-contact temperature detection
- Increasing hobbyist availability

---

## ❌ Cons

- Lower resolution than visible light cameras
- Export-controlled frame rates in many regions
- Some models have limited development tools

---

## 🌐 External References

- [FLIR Lepton Integration Guide](https://www.flir.com/products/lepton/)
- [Thermal Imaging in Drones – Oscar Liang](https://oscarliang.com/thermal-camera-fpv/)
- [Seek Thermal Developer Site](https://www.thermal.com/)

---
