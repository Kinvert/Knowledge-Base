# 🟦 pyftdi

**pyftdi** is a pure Python library that allows control of FTDI chips, especially the versatile [[FT232H]] and similar devices, over USB. It supports multiple protocols such as **I2C**, **SPI**, **UART**, and **GPIO**, and uses **MPSSE** (Multi-Protocol Synchronous Serial Engine) mode internally for advanced interfacing.

---

## 🧠 Summary

- Pythonic interface to FTDI USB bridges.
- Runs in **user space**, no special drivers required beyond `libusb`.
- Compatible with Linux, macOS, and Windows (via `libusb-win32` or `zadig`).
- Often used for embedded development, device scripting, and automated testing.

---

## ⚙️ Supported Protocols

| Protocol | Description                                    |
|----------|------------------------------------------------|
| I2C      | Master mode only, clock stretching supported   |
| SPI      | Full-duplex, variable clock polarity and phase |
| UART     | Standard UART serial                           |
| GPIO     | Individual pin control                         |
| BitBang  | Raw pin-level control                          |

---

## 🔧 Key Features

- Easy-to-use Python API.
- Detects devices by serial number, URL-style notation.
- No need for kernel-mode drivers (uses libusb backend).
- Supports multiple FTDI chips including FT232H, FT2232H, FT4232H.

---

## 📦 Installation

Install via pip (no code blocks in Obsidian!):
`pip install pyftdi`

---

## 🛠️ Use Cases

- Flashing EEPROM/flash chips over SPI or I2C.
- Bit-banging GPIOs for testing or automation.
- Quick script-based communication with embedded devices.
- Interfacing with sensors or boards without dedicated drivers.

---

## 🏆 Strengths

- No compilation, no C/C++ dependency.
- Works on all major OSes.
- Fast prototyping and debugging.
- Much easier than writing MPSSE commands manually.

---

## ⚠️ Limitations

- No official support for JTAG (unlike [[OpenOCD]]).
- Only supports FTDI chips with MPSSE.
- USB latency can limit very high-speed applications.

---

## 🔗 Related Notes

- [[FT232H]]
- [[I2C]]
- [[SPI]]
- [[UART]]
- [[Bus Pirate]]
- [[CH341A]]

---

## 🌐 External Links

- [pyftdi GitHub](https://github.com/eblot/pyftdi)
- [pyftdi Documentation](https://eblot.github.io/pyftdi/)

---
