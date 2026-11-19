# MIMO for Wireless Links

MIMO (Multiple-Input Multiple-Output) uses multiple antennas to improve throughput, resilience, or coverage through spatial multiplexing, diversity, and beamforming. For drones, it can be valuable on high-throughput paths (FPV/data uplink), but it is less universally useful for minimal command links constrained by link budget and size.

---

## ⚙️ Overview

In practical terms, MIMO works when transmit and receive antennas observe independent paths. In compact UAV airframes, this independence is harder:

- Small aperture = limited antenna spacing,
- Body shadowing and fuselage interaction,
- Rapid orientation changes changing polarization and path profile.

So MIMO is most often used selectively:
- high-power/large-payload craft, base-station-to-UAV links, or
- larger gateways and relay nodes.

---

## 🧠 Core Concepts

- **Spatial multiplexing**: send multiple streams over one channel.
- **Receive diversity**: combine copies to improve reliability.
- **Beamforming / precoding**: steer energy toward intended receiver.
- **MIMO-OFDM coupling**: common in Wi-Fi and LTE classes.
- **Feedback channel dependency**: CSI (channel state information) can become a latency burden.

---

## 📊 Comparison Chart

| Aspect | SISO | MIMO | When MIMO Helps in UAV C2 |
|---|---|---|---|
| Throughput | Baseline | Higher (multiplexing) | Video + map updates + telemetry |
| Link reliability | Baseline | Better (diversity) | Low-SNR control telemetry in stable orientation |
| Hardware cost | Low | Medium-High | Ground stations, larger drones |
| Complexity | Low | High | Requires calibration and CSI logic |
| Power use | Low | Medium-High | Often avoided on tiny flight controllers |

---

## 🛠 Use Cases

- UAV video downlinks for HD + telemetry in one stream.
- Ground station uplink to drone swarms where gateway has size/power budget.
- Dynamic routing gateways where MIMO gain helps against multipath.

---

## ✅ Strengths

- Better spectral efficiency on challenging channels.
- Potentially lower outage probability for same power budget.
- Can coexist with advanced channel coding and beamforming.

---

## ❌ Weaknesses

- Adds antenna placement and calibration burden.
- Gains collapse if element spacing is too small or orientation swings fast.
- On strict latency RC links, added processing can be a risk.

---

## 📚 Related Concepts

- [[Mesh Networking]]
- [[RF Jamming and Anti-Jamming]]
- [[RF Link Resilience Stack for Drones]]
- [[Adaptive Frequency Hopping]]

---

## 🌐 External Resources

- [ITU MIMO principles in wireless systems (overview)](https://www.itu.int/en/ITU-T)
- [3GPP MIMO documentation entry points](https://www.3gpp.org)
- [Wi-Fi MIMO vendor documentation portals](https://www.wi-fi.org/)

