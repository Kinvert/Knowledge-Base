
## 📡 Serial Protocols Comparison

This document summarizes and compares commonly used serial communication protocols. These are often used for microcontroller-to-device or device-to-device communication across short distances, with tradeoffs in speed, complexity, and use case.

---

### 📋 Protocol Overview Table

| Protocol       | Type              | Wires  | Speed (Typical)        | Master/Slave | Full Duplex | Distance      | Use Case                               |
| -------------- | ----------------- | ------ | ---------------------- | ------------ | ----------- | ------------- | -------------------------------------- |
| **[[UART]]**   | Async             | 2      | Up to ~1 Mbps          | N/A          | Yes         | ~15m (RS-232) | Simple point-to-point comms            |
| **[[RS-232]]** | Async             | 3+     | Up to 115.2 kbps       | N/A          | Yes         | ~15m          | PC to modem/serial device              |
| **[[RS-485]]** | Differential      | 2-3    | Up to 10 Mbps          | Optional     | Half        | ~1200m        | Industrial networks                    |
| **[[SPI]]**    | Sync              | 4+     | 10–50 Mbps (common)    | Yes          | Yes         | <1m           | High-speed peripheral interface        |
| **[[I2C]]**    | Sync              | 2      | 100k / 400k / 3.4 Mbps | Yes          | No          | ~1m           | Sensor/EEPROM interfaces               |
| **[[CAN]]**    | Differential      | 2      | Up to 1 Mbps           | Bus (multi)  | No          | ~40m–1km      | Automotive/industrial, fault-tolerant  |
| **LIN**        | UART-based        | 1      | 20 kbps                | Yes          | No          | ~40m          | Automotive low-speed communication     |
| **1-Wire**     | Half-duplex       | 1      | 16 kbps                | Master/Slave | No          | ~30m          | Very low-power sensors (e.g., DS18B20) |
| **[[Modbus]]** | Application Layer | Varies | 9600–115200 bps        | Master/Slave | No          | Varies        | Industrial devices (serial or TCP)     |

---

### 🧠 Notes

- **UART** is a hardware block; protocols like RS-232 or RS-485 build on top of it.
    
- **I²C** and **SPI** are common in embedded systems; I²C allows multi-master, SPI is much faster.
    
- **CAN** is fault-tolerant and designed for noise-heavy environments (like cars).
    
- **RS-485** allows for longer distances than RS-232 and is often used with Modbus.
    
- **1-Wire** can deliver both data and power, but it's slow.
    
- **LIN** is used in automotive contexts where CAN is overkill.
    
- **Modbus** is a higher-level protocol and can be layered on RS-485 or TCP/IP.
