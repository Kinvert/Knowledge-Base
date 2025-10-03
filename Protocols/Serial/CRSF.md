# CRSF (Crossfire Serial Protocol)

Crossfire Serial Protocol (CRSF) is a high-performance, low-latency digital communication protocol primarily used in radio control (RC) systems. It was introduced by Team BlackSheep (TBS) for long-range RC applications and has since become a standard for reliable communication between transmitters, receivers, and other hardware in robotics, drones, and UAVs.

---

## ⚙️ Overview

CRSF is designed to be faster and more efficient than traditional RC protocols such as SBUS or PPM. It uses a bidirectional serial link with high update rates, making it suitable for latency-sensitive applications like drone racing, autonomous robotics, and industrial systems requiring precise control.

---

## 🧠 Core Concepts

- **Bidirectional Communication**: Allows not only control signals to be sent but also telemetry data to be received.
- **Low Latency**: Sub-millisecond latency in optimized setups.
- **Compact Frames**: Efficient binary protocol with minimal overhead.
- **Standardized Extensions**: Supports telemetry, sensor data, and configuration commands.

---

## 📊 Comparison Chart

| Protocol | Latency | Channel Count | Telemetry | Typical Use |
|----------|---------|---------------|-----------|-------------|
| CRSF     | ~3 ms   | 12–16+        | Yes       | Long-range RC, robotics, UAVs |
| SBUS     | ~15 ms  | 16            | Limited   | Drones, hobby RC |
| PPM      | ~20+ ms | 8             | No        | Legacy RC |
| DSMX     | ~22 ms  | 6–12          | Limited   | RC hobby aircraft |
| MAVLink  | 50+ ms  | Many          | Yes       | UAV autopilot, telemetry systems |

---

## 🔧 Use Cases

- Long-range drone communication
- Robotics with strict latency requirements
- Industrial telemetry systems
- Research platforms needing robust wireless control

---

## ✅ Strengths

- Extremely low latency compared to older protocols  
- High reliability and long-range capability  
- Full-duplex communication (control + telemetry)  
- Wide adoption in high-performance UAV communities  

---

## ❌ Weaknesses

- Proprietary origins (developed by TBS, though widely adopted)  
- Requires compatible hardware (not universal like UART)  
- Higher complexity compared to simple PPM/SBUS  

---

## 🔗 Related Concepts

- [[UART]] (Universal Asynchronous Receiver-Transmitter)
- [[MAVLink]] (Micro Air Vehicle Link)
- [[DDS]] (Data Distribution Service)
- [[I2C]] (Inter-Integrated Circuit)
- [[SPI]] (Serial Peripheral Interface)
- [[ExpressLRS]]
- [[DroneForge Nimbus]]

---

## 🧩 Compatible Items

- TBS Crossfire TX/RX modules  
- ExpressLRS (open-source alternative with CRSF support)  
- Betaflight, iNav, and Ardupilot flight controllers  
- CRSF-capable telemetry sensors  

---

## 🌱 Variants

- **CRSF Standard**: Original protocol used in TBS Crossfire  
- **CRSFShot**: A variant optimized for very high refresh rates (popular in FPV drone racing)  
- **ELRS CRSF**: ExpressLRS implementation using CRSF for compatibility  

---

## 📚 External Resources

- [Team BlackSheep CRSF Protocol Documentation](https://www.team-blacksheep.com/)
- [ExpressLRS Documentation](https://www.expresslrs.org/)
- [ArduPilot CRSF Integration](https://ardupilot.org/)
- [Betaflight Wiki on CRSF](https://github.com/betaflight/betaflight/wiki)
- https://github.com/nleve/joydrone

---

## 🛠 Developer Tools

- Serial analyzers (e.g., logic analyzers for UART)  
- OpenTX/EdgeTX radio firmware with CRSF support  
- Flight controller firmware (Betaflight, iNav, ArduPilot)  

---

## 📖 Summary

CRSF has become a cornerstone protocol in modern RC and robotics communication, offering unmatched latency and reliability. Its combination of control and telemetry in a single stream makes it a natural fit for UAVs, autonomous systems, and robotics platforms requiring efficient, robust, and long-range communication.
