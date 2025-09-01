# FPD-Link (Flat Panel Display Link)

**FPD-Link (Flat Panel Display Link)** is a high-speed digital video interface developed by Texas Instruments. Originally designed for connecting graphics processors to flat-panel displays, FPD-Link has evolved into a key serializer-deserializer (SerDes) technology used in automotive, robotics, and industrial systems. It enables long-distance, high-bandwidth transmission of video, control, and data signals over a single cable.

---

## ⚙️ Overview

FPD-Link transmits serialized video and data over differential pairs, with deserializers reconstructing the original signals at the receiving end. Newer generations, such as **FPD-Link II** and **FPD-Link III**, add features like bidirectional control channels, embedded clocking, and power-over-coax. In robotics and autonomous systems, FPD-Link is widely used to connect cameras and displays to central processors across several meters.

---

## 🧠 Core Concepts

- **SerDes Technology**: Converts parallel data into serial streams for transmission, then back again.  
- **Differential Signaling**: Uses LVDS (Low-Voltage Differential Signaling) for noise immunity.  
- **Embedded Clocking**: Later generations removed the need for a separate clock line.  
- **Long Cable Support**: Enables reliable data transmission over coaxial or shielded twisted pair.  
- **Power-over-Coax (PoC)**: Supplies power and data through a single cable.  
- **Scalability**: Supports multiple video streams in advanced automotive/robotic systems.  

---

## 📊 Comparison Chart

| Feature / Aspect       | FPD-Link III | GMSL (Gigabit Multimedia Serial Link) | HDMI | MIPI CSI-2 | USB Cameras |
|------------------------|--------------|---------------------------------------|------|------------|-------------|
| Latency                | ✅ Low       | ✅ Low                                | ❌ Higher | ✅ Very Low | ❌ Higher |
| Cable Length           | ✅ Long (>15m) | ✅ Long (>15m)                       | ❌ Short (<5m) | ❌ Very Short (<30cm) | ✅ Moderate |
| Power-over-Cable       | ✅ Yes       | ✅ Yes                                | ❌ No | ❌ No | ✅ Sometimes |
| Automotive/Robotics Use| ✅ Very High | ✅ Very High                          | ❌ Rare | ✅ Prototyping | ✅ Prototyping |
| Bandwidth              | ✅ High      | ✅ Very High                          | ✅ High | ✅ High | ❌ Limited |
| Noise Immunity         | ✅ High      | ✅ High                               | ❌ Lower | ❌ Medium | ❌ Medium |

---

## 🔧 Use Cases

- Connecting cameras to central processing units in autonomous vehicles  
- Display links for in-vehicle infotainment and robotics HMIs  
- Long-distance camera connections in industrial and agricultural robots  
- High-speed video transfer with simultaneous control signaling  
- PoC applications to reduce cabling complexity  

---

## ✅ Strengths

- Supports long cable runs with high signal integrity  
- Power and data over a single coaxial cable  
- Very low latency suitable for real-time robotics and ADAS  
- Automotive-grade reliability and EMI resistance  

---

## ❌ Weaknesses

- Proprietary (TI-specific ecosystem)  
- Requires dedicated serializers and deserializers  
- Less common outside automotive/industrial use cases  
- Costlier than short-range interfaces like [[MIPI CSI-2]]  

---

## 🛠️ Compatible Items

- [[Jetson Xavier]] and [[Jetson Orin]] (via serializer/deserializer bridges)  
- [[Cameras]] designed with FPD-Link III support  
- [[Display Interfaces]] in automotive robotics  
- [[SerDes]] chipsets from Texas Instruments  

---

## 📚 Related Concepts

- [[GMSL]] (Gigabit Multimedia Serial Link)  
- [[MIPI CSI-2]]  
- [[NVCSI]]  
- [[SerDes]]  
- [[HDMI]]  

---

## 🌐 External Resources

- Texas Instruments FPD-Link Product Page  
- TI Application Notes on FPD-Link III in automotive cameras  
- Whitepapers comparing FPD-Link III and GMSL  

---

## 📝 Summary

FPD-Link is a robust SerDes technology for transmitting high-speed video and control signals over long distances. Its evolution into FPD-Link III with embedded clocking and PoC makes it a backbone for modern automotive and robotic vision systems, enabling reliable and simplified camera and display connections.
