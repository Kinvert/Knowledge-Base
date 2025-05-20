# UART (Universal Asynchronous Receiver-Transmitter)

## 📝 Summary

**UART** is a **hardware communication protocol** used to send and receive serial data asynchronously between two digital devices. It is one of the simplest and most widely used serial interfaces in embedded systems.

UART does **not require a shared clock** between sender and receiver. Instead, it uses agreed-upon baud rates and start/stop bits to synchronize data transmission.

---

## 📚 OSI Model Placement

| Layer   | Name         | UART's Role                         |
|---------|--------------|-------------------------------------|
| Layer 1 | Physical     | Defines voltage levels, connections |
| Layer 2 | Data Link    | Handles start/stop bits, parity     |

UART spans both the **Physical** and **Data Link** layers. It's a foundational protocol used at the hardware level for device-to-device communication.

---

## 🧰 Components of UART Communication

- **TX (Transmit)** and **RX (Receive)** lines
- **Baud rate**: Common values include 9600, 115200, etc.
- **Start bit**: Indicates beginning of a data frame
- **Data bits**: Usually 7 or 8
- **Optional parity bit**
- **Stop bit(s)**: Marks end of frame

---

## 🔄 How UART Works

1. Sender converts parallel data into serial bits and transmits via TX.
2. Receiver listens on RX and reconstructs the parallel byte.
3. Both sides must use **identical settings** (baud rate, parity, stop bits).
4. **No clock line** is shared, so timing must match closely.

---

## 📦 Use Cases

| Use Case                          | Description                                      |
|----------------------------------|--------------------------------------------------|
| Debugging embedded systems       | UART ports often provide serial console access  |
| Microcontroller communication    | Simple MCU-to-MCU or MCU-to-PC links            |
| GPS and sensor modules           | Many use UART for data output                   |
| Serial terminals                 | Legacy command interfaces (e.g., RS-232 over UART) |

---

## 🔧 Hardware & Standards

| Spec / Standard     | Description                                 |
|---------------------|---------------------------------------------|
| RS-232              | Commonly implemented over UART (voltage levels differ) |
| TTL-Level UART      | Used in MCUs like Arduino, STM32, ESP32     |
| RS-485              | Differential signaling, often uses UART framing |
| IEEE 1284           | Not directly UART, but relevant to serial interfacing |

---

## 🔍 Key Characteristics

| Feature              | UART                          |
|----------------------|-------------------------------|
| Communication type   | Serial                        |
| Clocking             | Asynchronous (no shared clock)|
| Pins needed          | 2 (TX, RX) or 3–4 (with GND/CTS/RTS) |
| Data direction       | Full-duplex (simultaneous TX/RX) |
| Max speed            | Varies (commonly up to 1 Mbps) |
| Distance             | Short (<15m at high baud rates) |

---

## 🔄 Comparison Table

| Protocol | Type       | Clock Line | Duplex     | Wires | Speed      | Complexity |
|----------|------------|------------|------------|-------|------------|------------|
| UART     | Serial     | ❌ No      | ✅ Full     | 2–4   | Medium     | ✅ Simple   |
| SPI      | Serial     | ✅ Yes     | ✅ Full     | 4+    | Very High  | ❌ Complex  |
| I2C      | Serial     | ✅ Yes     | ✅ Half     | 2     | Medium     | ⚠️ Moderate |
| CAN      | Serial     | ✅ Yes     | ✅ Full     | 2     | High       | ❌ Complex  |
| USB      | Serial     | ✅ Yes     | ✅ Full     | 4     | Very High  | ❌ Complex  |

---

## ✅ Pros

- Very simple and low overhead
- Supported natively on almost all MCUs
- Great for debugging
- Doesn't require a clock line

## ❌ Cons

- Not multi-master or multi-slave (1:1 only)
- No formal addressing or protocol structure
- Easily desynchronized with mismatched baud rate
- Short communication range

---

## 🧠 Strengths & Weaknesses

**Strengths:**
- Simplicity and ease of use
- Widely supported
- Asynchronous nature reduces pin usage

**Weaknesses:**
- Lack of protocol framing
- Poor scalability
- No error correction beyond optional parity

---

## 🧭 See Also

- [[SPI]]
- [[I2C]]
- [[RS-232]]
- [[RS-485]]
- [[Serial Protocols Comparison]]
- [[USART]]
