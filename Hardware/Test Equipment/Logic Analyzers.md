# Logic Analyzers

## 🧠 Summary

A **Logic Analyzer** is an electronic instrument that captures and displays multiple digital signals from a system or circuit. It's primarily used for debugging and analyzing **digital protocols**, **timing relationships**, and **bus communications** such as I2C, SPI, UART, CAN, and more.

Unlike an **oscilloscope**, which visualizes analog waveforms, a logic analyzer interprets **digital logic levels** (typically 0 or 1) and provides **protocol decoding**, **triggering**, and **timing correlation** across multiple channels.

---

## 🎯 Use Cases

- Debugging I2C, SPI, UART, CAN, etc.
- Analyzing timing relationships between microcontroller pins
- Capturing rare communication faults
- Reverse-engineering hardware protocols
- Verifying firmware and hardware interface logic
- Developing or debugging custom embedded protocols

---

## 🔩 Key Features to Consider

- **Channel Count** – Number of simultaneous digital signals captured (8, 16, 32+)
- **Sample Rate** – Higher is better (usually in MSa/s or GSa/s)
- **Memory Depth** – How much data it can buffer
- **Protocol Decoding** – Built-in or software-based decoding for common protocols
- **Triggering** – Conditional signal capture based on events
- **Form Factor** – Standalone device vs USB dongle
- **Software** – UI, ease of use, platform support (Windows/Linux/macOS)

---

## 💡 Popular Protocols Supported

- I2C
- SPI
- UART
- CAN / LIN
- 1-Wire
- Parallel buses
- Manchester encoding
- USB (some models)

---

## 📊 Comparison Table – Popular Logic Analyzers

| Device                     | Channels | Sample Rate     | Protocol Support       | Platform        | Price (USD) | Notes                                      |
|----------------------------|----------|------------------|-------------------------|------------------|-------------|--------------------------------------------|
| Saleae Logic 8             | 8        | 100 MSa/s        | 100+ (via Saleae SW)    | Win/Mac/Linux    | ~$449       | Best-in-class software, easy to use        |
| Saleae Logic Pro 16        | 16       | 500 MSa/s (digital), 50 MSa/s (analog) | Extensive         | Cross-platform | ~$999       | High-end, analog + digital capture         |
| DreamSourceLab DSLogic Plus| 16       | 400 MSa/s        | I2C, SPI, UART, etc.    | Win/Linux        | ~$89        | Great value, open source software          |
| OpenBench Logic Sniffer    | 16       | 200 MSa/s        | Basic protocols         | Community driven | ~$50        | Good for hackers, less polished software   |
| Kingst LA1010              | 8        | 24 MSa/s         | UART, I2C, SPI          | Windows only     | ~$20        | Very cheap entry point                     |
| Digilent Digital Discovery | 32       | 800 MSa/s        | Extensive + analog       | Win/Linux        | ~$199       | Excellent for FPGA debugging               |
| Analog Discovery 2         | 16       | 100 MSa/s        | With WaveForms software | Win/Mac/Linux    | ~$279       | Combo device (logic + scope + more)        |

---

## ✅ Pros and ❌ Cons

### ✅ Pros

- Non-intrusive digital debugging
- Captures **timing issues** invisible to a software debugger
- Essential for decoding communication protocols
- Often far more **affordable** than high-end oscilloscopes
- Portable options available (USB-powered)
- Excellent for embedded systems, robotics, automotive ECUs, etc.

### ❌ Cons

- Not suitable for analog signal analysis
- May require **external clock sync** for long sessions
- Entry-level models can miss fast signals (low sample rate)
- Some require proprietary or clunky software
- Poor compatibility with **very high-speed** buses (USB 3.0, HDMI)

---

## 🧠 Tips for Choosing One

- **Hobbyist or occasional use?** → DreamSourceLab DSLogic or Kingst
- **Pro-level embedded work?** → Saleae Logic Pro or Digilent Discovery
- **Need analog & digital?** → Saleae Pro or Analog Discovery 2
- **Working on FPGAs or wide buses?** → Get at least 16+ channels with high sample rate

---

## 🔗 Related Notes

- [[Oscilloscope]]
- [[UART]]
- [[I2C]]
- [[SPI]]
- [[CAN]]
- [[Protocol Debugging Tools]]
- [[Embedded Systems Testing]]
- [[Saleae Logic]]
