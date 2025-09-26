# ExpressLRS

**ExpressLRS (ELRS)** is an open-source, high-performance, and low-latency radio control link for RC applications such as drones, planes, and robotics. It has rapidly become a popular alternative to proprietary systems due to its low cost, strong community support, and high update rates. ELRS builds on LoRa (Long Range) modulation to achieve both long range and high reliability while maintaining impressive responsiveness.

---

## ⚙️ Overview

ExpressLRS was designed to replace traditional RC protocols by delivering:
- Lower latency than most commercial systems
- Higher update rates (up to 1000 Hz)
- Longer range and stronger link reliability
- Affordable hardware with open-source firmware

It is widely used in FPV drone racing, long-range RC flying, and increasingly in robotics where fast and deterministic wireless control is important.

---

## 🧠 Core Concepts

- **LoRa modulation**: Provides long range and robust communication
- **Packet rates**: 25 Hz to 1000 Hz, allowing trade-offs between range and latency
- **Frequency bands**: 2.4 GHz and 900 MHz commonly used
- **Telemetry**: Bidirectional link supports telemetry data (voltage, GPS, sensor data)
- **Open source**: Community-driven development, no vendor lock-in
- **Cross-compatibility**: Works with a variety of hardware manufacturers

---

## 🏆 Key Features

- Latency as low as ~2 ms
- Configurable update rates (25 Hz to 1000 Hz)
- Adaptive frequency hopping for reliability
- Flexible power output settings for range vs. battery trade-offs
- Wide ecosystem of compatible receivers and transmitters
- Firmware easily flashable via Wi-Fi, UART, or USB

---

## 📊 Comparison Chart

| Feature                | ExpressLRS         | Crossfire (TBS)   | FrSky R9        | Ghost (ImmersionRC) | DJI FPV RC         |
|------------------------|-------------------|------------------|----------------|---------------------|-------------------|
| **Open Source**        | Yes               | No               | No             | No                  | No                |
| **Latency**            | ~2 ms (1000 Hz)   | ~4-10 ms         | ~20 ms         | ~5-8 ms             | ~7-10 ms          |
| **Range**              | Very High         | Very High        | High           | High                | Moderate          |
| **Update Rate**        | Up to 1000 Hz     | 150 Hz           | 100 Hz         | 500 Hz              | 700 Hz            |
| **Frequency Bands**    | 2.4 GHz, 900 MHz  | 900 MHz, 2.4 GHz | 900 MHz        | 2.4 GHz             | 5.8 GHz           |
| **Cost**               | Low               | High             | Moderate       | High                | High              |
| **Telemetry**          | Yes               | Yes              | Yes            | Yes                 | Yes               |

---

## 🛠️ Use Cases

- **FPV Drone Racing**: Low latency control is crucial for responsiveness
- **Long-Range Flying**: Strong link reliability over kilometers
- **Robotics**: Deterministic wireless control for mobile robots
- **Research**: Open-source protocol ideal for academic and experimental use
- **DIY Projects**: Affordable, hackable hardware ecosystem

---

## ✅ Strengths

- Open-source and community-driven
- Extremely low latency and high update rates
- Long range with robust LoRa modulation
- Affordable compared to proprietary systems
- Wide support across hardware vendors

---

## ❌ Weaknesses

- Requires some technical knowledge to flash and configure
- Smaller ecosystem compared to older established proprietary systems
- Less user-friendly for beginners compared to plug-and-play systems
- Regulatory considerations depending on frequency and region

---

## 🔧 Compatible Items

- [[LoRa]]
- [[RC Transmitters]]
- [[RC Receivers]]
- [[FPV Drones]]
- [[Telemetry Systems]]
- [[Robotics Wireless Communication]]

---

## 📚 Related Concepts

- [[LoRa]] (Long Range radio modulation)
- [[Crossfire]] (TBS protocol)
- [[FrSky]] (Popular RC brand and protocol family)
- [[Ghost]] (ImmersionRC protocol)
- [[Telemetry Systems]]
- [[Wireless Communication in Robotics]]

---

## 🌐 External Resources

- [ExpressLRS Official GitHub](https://github.com/ExpressLRS/ExpressLRS)
- [ExpressLRS Documentation](https://www.expresslrs.org/)
- [Community Wiki](https://www.expresslrs.org/quick-start/)
- FPV forums and community groups

---

## 📝 Summary

ExpressLRS has quickly established itself as a leading open-source RC control system, offering extremely low latency, long range, and affordability. Its flexibility and community-driven development make it especially attractive not just for FPV pilots, but also for robotics engineers who need deterministic wireless control. By leveraging LoRa modulation and open firmware, ExpressLRS delivers performance on par with or better than many proprietary alternatives at a fraction of the cost.
