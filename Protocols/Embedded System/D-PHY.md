# D-PHY (MIPI D-PHY)

D-PHY is a physical layer specification defined by the MIPI Alliance. It provides a high-speed serial interface primarily used for connecting cameras and displays in embedded systems. While most commonly associated with [[CSI-2]] (Camera Serial Interface) and [[DSI]] (Display Serial Interface), it is a general-purpose PHY layer that can be used by other MIPI protocols.

---

## ⚙️ Overview

D-PHY defines the electrical and timing characteristics of the physical connection between SoCs and peripherals. It is designed to balance **low-power** operation with **high-speed** data transfer, which is essential in mobile, robotics, and embedded platforms where both performance and efficiency matter.

---

## 🧠 Core Concepts

- **Lanes**: Composed of one clock lane and one or more data lanes.  
- **Low-Power (LP) Mode**: Used for control and signaling at low speeds with low energy usage.  
- **High-Speed (HS) Mode**: Enables gigabit-per-second data transfer for video and sensor data.  
- **Source-Synchronous Signaling**: The clock lane provides synchronization for data lanes.  
- **Scalability**: Multiple data lanes can be used in parallel to increase throughput.

---

## 📊 Comparison Chart

| Feature             | D-PHY              | C-PHY                  | LVDS                    | SLVS-EC                  | USB 3.x PHY             |
|---------------------|--------------------|------------------------|-------------------------|--------------------------|-------------------------|
| Standard Body       | MIPI Alliance      | MIPI Alliance          | Open/JEDEC              | JEDEC                    | USB-IF                  |
| Typical Use Cases   | CSI-2, DSI         | CSI-2 (alt mode)       | Legacy displays, cameras| Industrial imaging, AI   | General I/O             |
| Signaling           | Differential pair  | 3-phase encoding       | Differential pair       | Differential pair        | Differential pair       |
| Data Rate           | ~4.5 Gbps/lane     | Higher efficiency/lane | Up to ~1 Gbps/lane      | Up to ~2.5 Gbps/lane     | Up to 20 Gbps total     |
| Low-Power Mode      | Yes                | Yes                    | No                      | No                       | Limited                 |

---

## 🚀 Use Cases

- **Cameras**: Interfacing image sensors to SoCs via [[CSI-2]].  
- **Displays**: Driving high-resolution panels with [[DSI]].  
- **Robotics**: Efficient, low-latency sensor integration.  
- **Mobile Devices**: Smartphones, tablets, AR/VR headsets.  
- **Automotive**: Camera-based ADAS (Advanced Driver Assistance Systems).  

---

## ✅ Strengths

- Optimized for mobile and embedded use cases.  
- Dual-mode operation (LP + HS).  
- Widely adopted and supported in SoCs (e.g., [[Jetson Nano]], Raspberry Pi).  
- Scalable bandwidth via multiple lanes.  

---

## ❌ Weaknesses

- Range is typically limited to PCB traces or short flex cables.  
- Not designed for long-distance transmission.  
- Requires strict signal integrity and PCB layout rules.  
- Mostly tied to MIPI ecosystem (less general-purpose than USB/PCIe).  

---

## 🔧 Compatible Items

- [[CSI-2]] (Camera Serial Interface 2)  
- [[DSI]] (Display Serial Interface)  
- [[Jetson Family]] (uses D-PHY lanes for camera/display integration)  
- [[SBCs]] (various boards with camera connectors use MIPI D-PHY)  
- [[Sensors]] (image sensors leveraging CSI-2)  

---

## 📚 Related Concepts

- [[C-PHY]] (Alternative MIPI physical layer)  
- [[LVDS]] (Low-Voltage Differential Signaling)  
- [[SLVS-EC]] (Scalable Low-Voltage Signaling for Embedded Clock)  
- [[I2C]] (Used alongside for configuration/control of CSI-2 devices)  

---

## 🌐 External Resources

- MIPI Alliance D-PHY Specification  
- MIPI CSI-2 and DSI overview docs  
- JEDEC SLVS-EC specifications for comparison  

---

## 🏗️ How It Works

1. Devices connect via one differential clock lane and multiple differential data lanes.  
2. During **Low-Power Mode**, the link handles command and control.  
3. When streaming video/data, the PHY switches into **High-Speed Mode**.  
4. Clock lane synchronizes transitions on data lanes to ensure timing alignment.  

---

## 📖 Summary

D-PHY is the foundational MIPI physical layer that underpins most modern camera and display interfaces. While most often encountered in CSI-2 and DSI contexts, its design is more general-purpose, balancing high throughput with energy efficiency. In robotics and embedded vision, D-PHY serves as a critical enabler for high-resolution, low-latency sensor integration.
