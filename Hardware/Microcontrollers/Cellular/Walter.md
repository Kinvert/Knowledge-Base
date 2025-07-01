# 🟣 Walter – Multi‑Radio ESP32‑S3 Cellular Module

**Walter** is a compact, open‑source IoT System‑on‑Module built around the **ESP32‑S3** MCU and a **Sequans GM02SP LTE‑M/NB‑IoT modem** with GPS. It offers Wi‑Fi, Bluetooth, GNSS, and cellular connectivity in a pre‑certified, production‑ready package—ideal for battery‑powered and remote IoT applications.

---

## 🧠 Summary

- Combines:
  - **ESP32‑S3** (dual‑core Xtensa LX7 + RISC‑V coprocessor, 16 MB flash, 2 MB PSRAM)
  - **Sequans GM02SP** LTE‑M / NB‑IoT modem with integrated GNSS (GPS+Galileo)
- Built‑in **Wi‑Fi b/g/n**, **Bluetooth 5**, extensive GPIO interface; open hardware (KiCad), schematics under GPLv3 :contentReference[oaicite:0]{index=0}
- CE / FCC / UKCA / IC / RCM certifications simplify commercialization :contentReference[oaicite:1]{index=1}
- Ultra‑low deep‑sleep current: ~9.8 μA :contentReference[oaicite:2]{index=2}

---

## 🔩 Connectivity & I/O

- **Cellular**: LTE‑M & NB‑IoT via u.FL antenna connector
- **GNSS**: GPS + Galileo via u.FL connector
- **Wi‑Fi / BLE**: pcb antenna on board
- **General‑purpose I/O**: 24 GPIO pins, re‑assignable esp32-s3 pins (UART, SPI, I²C, CAN, I²S, ADC, DAC, PWM) :contentReference[oaicite:3]{index=3}
- **USB‑C**: Power, flashing, console access
- **Nano‑SIM slot**
- **Peripheral power control**: 3.3 V switched output :contentReference[oaicite:4]{index=4}

---

## ⚡ Power & Form Factor

- Input: 5 V via USB‑C or 3 – 5.5 V via Vin :contentReference[oaicite:5]{index=5}
- Deep sleep: ~9.8 μA :contentReference[oaicite:6]{index=6}
- Size: 55 × 24.8 mm, Pin‑compatible with Pycom GPy; breadboard‑friendly :contentReference[oaicite:7]{index=7}
- Ten‑year availability guarantee :contentReference[oaicite:8]{index=8}

---

## 🛠️ Software & Ecosystem

- Supported toolchains: **ESP‑IDF**, **Arduino**, **MicroPython**, **Toit** :contentReference[oaicite:9]{index=9}
- Open‑source libraries for AT‑command control (NB‑IoT, LTE‑M, GNSS, MQTT/CoAP offloaded to modem) :contentReference[oaicite:10]{index=10}
- No need for special hardware—program via USB; fully GPLv3 with KiCad files :contentReference[oaicite:11]{index=11}

---

## 🧪 Use Cases

- Remote sensors and asset trackers in harsh or low‑connectivity environments
- Battery‑powered data loggers and environmental monitoring
- Cellular‑connected wearables and industrial IoT nodes
- Pycom GPy replacement for existing designs :contentReference[oaicite:12]{index=12}

---

## ⚖️ Pros & Cons

### ✅ Pros
- Rich connectivity (Wi‑Fi / BLE / Cellular / GNSS)
- Pre‑certified, reducing certification burden
- Open source hardware and software
- Very low deep‑sleep current
- Pin‑compatible with GPy (easy adoption)

### ⚠️ Cons
- Sequans modem adds to design complexity
- LTE‑M / NB‑IoT data rates and coverage depend on carrier support
- Not designed for heavy compute—ESP32‑S3 is MCU, not MCU + Linux SBC

---

## 📊 Comparison with Similar Modules

| Module / Board       | MCU / SoC        | Cellular             | GNSS       | Wi‑Fi / BT   | Open Source | Certifications | Notes                                           |
|----------------------|------------------|----------------------|------------|--------------|-------------|----------------|------------------------------------------------|
| **Walter**           | ESP32‑S3         | LTE‑M / NB‑IoT (GM02SP) | GPS + Galileo | Wi‑Fi + BLE | ✅ (HW+SW)   | ✅ (CE/FCC/UKCA) | Low power, GPy-compatible, rich I/O            |
| Pycom GPy            | ESP32            | LTE‑CAT M1/NB1 (Sequans) | No         | Wi‑Fi + BLE | ❌ (HW)      | CE/FCC         | No GPS, not fully open, older chip             |
| Blues Wireless Notecard | STM32 + M.2     | LTE‑M / NB‑IoT       | Yes        | ❌            | ✅ (partially) | ✅              | MCU hidden, not programmable; for integration  |
| nRF9160 DK           | nRF9160 (Cortex‑M33) | LTE‑M / NB‑IoT    | GPS        | ❌            | ✅ (HW+SW)   | ✅              | Nordic SDK; dev board, not SoM                 |
| Particle Boron       | nRF52840 + u‑blox SARA | LTE‑M / NB‑IoT | No         | BLE          | ❌ (HW)      | ✅              | Tight integration with Particle Cloud          |
| Sixfab Base HAT      | Uses Raspberry Pi | LTE via HAT modem   | Optional   | Yes via Pi   | ✅           | Depends on Pi   | Not a standalone module                        |
| Seeed Wio Tracker 1110 | STM32WL          | LoRa / GNSS          | GPS        | ❌            | ✅           | ✅              | No LTE‑M/NB, uses LoRaWAN                      |

---

## 📝 Notes

- **Walter** distinguishes itself by combining full programmability, open design, and four radios (Wi‑Fi, BLE, GNSS, LTE‑M/NB‑IoT) in a small module.
- **Blues Notecard** focuses on ease of integration via JSON over serial, but does not expose its MCU.
- **nRF9160 DK** and **Boron** are more suited to cloud‑native applications but require their own ecosystems or tools.
- **Seeed Wio Tracker** is a good alternative for LoRa applications, not LTE.

---

## 🔗 Related Notes

- [[ESP32]]
- [[MQTT]]
- [[NB‑IoT]]
- [[LTE‑M]]
- [[GNSS]]
- [[Pycom GPy]]
- [[IoT Edge Computing]]

---

## 🌐 External Links

- [Official product page & docs] :contentReference[oaicite:13]{index=13}  
- [CrowdSupply listing] :contentReference[oaicite:14]{index=14}  
- [Embedded Computing write‑up] :contentReference[oaicite:15]{index=15}  
- [Wiki page on NB‑IoT context] :contentReference[oaicite:16]{index=16}

---
