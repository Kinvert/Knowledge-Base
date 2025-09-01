# GMSL2

**GMSL2 (Gigabit Multimedia Serial Link, Generation 2)** is a high-speed SerDes (Serializer/Deserializer) interface developed by Maxim Integrated (now Analog Devices). It is widely used in automotive and robotics applications for transmitting uncompressed video, audio, control data, and power over a single coaxial cable or shielded twisted pair (STP). Its low latency and long-distance capabilities make it an enabling technology for advanced vision-based systems.

---

## ⚙️ Overview

GMSL2 enables robust, high-bandwidth data transmission between sensors (such as cameras) and central processing units (such as ECUs or robotics computers). It supports resolutions up to 4K, multiple data streams, and power delivery over the same physical link. This reduces cabling complexity while ensuring reliable data integrity in noisy environments.

---

## 🧠 Core Concepts

- **Serializer/Deserializer**: A serializer converts parallel data (e.g., from a camera) into a serialized high-speed stream, while the deserializer restores it to parallel form for processing.  
- **Medium**: Operates over coaxial cables (up to ~15 m) or STP (up to ~10 m).  
- **Multiplexing**: Can carry video, audio, I²C/GPIO control signals, and power simultaneously.  
- **Power over Coax (POC)**: Eliminates separate power cabling for sensors.  
- **Low Latency**: <10 µs, making it suitable for real-time robotics and ADAS.  

---

## 📊 Comparison Chart

| Feature / Standard  | GMSL2 | GMSL1 | [[FPD-Link]] III | [[MIPI CSI-2 Protocol]] | [[Ethernet AVB]] |
|----------------------|-------|-------|------------------|----------------|-----------------|
| Max Bandwidth        | 6 Gbps/lane | ~3 Gbps/lane | ~4 Gbps/lane | 6–18 Gbps (multi-lane) | 1–10 Gbps |
| Cable Type           | Coax/STP | Coax/STP | Coax/STP | Short PCB flex | Twisted pair |
| Max Distance         | 10–15 m | ~15 m | ~15 m | <30 cm | 100 m+ |
| Power over Cable     | ✅ Yes | ✅ Yes | ✅ Yes | ❌ No | ❌ No |
| Latency              | <10 µs | <15 µs | <10 µs | <1 µs | Variable |
| Typical Use          | Automotive, Robotics | Legacy automotive | Automotive | Internal camera/sensor buses | Industrial, AV networks |

---

## 🔧 Use Cases

- **Robotics**: Long-distance camera integration with [[Jetson]] or other vehicle computing platforms.  
- **Automotive**: Advanced driver-assistance systems (ADAS), surround view, rear cameras, LiDAR.  
- **Industrial**: Machine vision systems requiring low-latency video links.  
- **Drones/UAVs**: High-resolution cameras with minimal cabling.  

---

## ✅ Strengths

- High bandwidth for uncompressed video transmission  
- Long cable support with robust EMI immunity  
- POC reduces cabling complexity  
- Very low latency for real-time control  
- Multi-stream support (video + control + audio)  

---

## ❌ Weaknesses

- Proprietary technology (vendor lock-in with Analog Devices)  
- Requires specialized serializer/deserializer ICs  
- More expensive compared to simpler interfaces like [[I2C]] or [[SPI]]  
- Limited ecosystem compared to [[MIPI CSI-2 Protocol]]  

---

## 🛠️ Compatible Items

- [[Jetson Xavier]] and [[Jetson Orin]] platforms (via carrier boards with GMSL2 deserializers)  
- Automotive-grade cameras (Omnivision, ON Semiconductor, Leopard Imaging)  
- GMSL2 SerDes chipsets from Maxim/Analog Devices  
- Robotics carrier boards with integrated GMSL2 support  

---

## 🧩 Variants

- **GMSL1**: First-generation, lower bandwidth (~3 Gbps)  
- **GMSL2**: Enhanced bandwidth, lower latency, improved robustness  
- Future iterations may converge with automotive Ethernet standards  

---

## 📚 Related Concepts

- [[FPD-Link]]  
- [[MIPI CSI-2 Protocol]]  
- [[SerDes]]  
- [[I2C]]  
- [[Jetson]]  
- [[ADAS]]  

---

## 🌐 External Resources

- Analog Devices GMSL2 Product Page  
- Leopard Imaging Camera Modules with GMSL2 Support  
- NVIDIA Jetson Partner Ecosystem (GMSL2 Cameras)  
- Maxim Integrated Application Notes on GMSL2  

---

## 📝 Summary

GMSL2 is a powerful SerDes interface enabling high-speed, long-distance video and data transmission for robotics and automotive systems. Its ability to combine video, control, and power in a single cable reduces complexity while ensuring reliability in challenging environments. Though proprietary, it remains a dominant standard in automotive-grade sensor connectivity.
