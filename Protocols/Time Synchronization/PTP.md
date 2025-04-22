---
title: PTP
tags: [ptp, timing, networks, synchronization, robotics, autonomous]
aliases: [Precision Time Protocol, IEEE 1588]
---

# 🕰️ PTP in Networked Sensor Timing

## What is PTP?

**Precision Time Protocol (PTP)**, defined by **IEEE 1588**, is a protocol used to synchronize clocks throughout a computer network. It enables sub-microsecond synchronization—far more precise than NTP (Network Time Protocol)—making it critical for distributed systems like robotics, industrial automation, and autonomous vehicles.

---

## Why It Matters in Driverless Trucks

Sensor fusion, LIDARs, radars, cameras, IMUs, and actuators **must operate on a shared time base** to ensure:

- Accurate fusion of sensor data (especially for moving platforms).
- Coordinated actuation (steering, braking).
- Consistent log timestamps for debugging and analysis.
- Seamless cooperation between ECUs (Electronic Control Units).

---

## How PTP Works

PTP designates devices as:

- **Master Clock**: Provides the reference time.
- **Slave Clocks**: Synchronize to the master.
- **Transparent Clocks** (optional): Improve accuracy by compensating for switch delays.

PTP uses hardware timestamps to determine **delay** and **offset** between clocks.

### Message Types

- `Sync`: Sent by master to initiate timestamping.
- `Follow_Up`: Provides the precise transmission timestamp.
- `Delay_Request`: Sent by slave to measure round-trip time.
- `Delay_Response`: Returned by master with timestamp info.

### Sync Process Flow (simplified):

1. Master sends `Sync` + `Follow_Up`
2. Slave timestamps receipt
3. Slave sends `Delay_Request`
4. Master responds with `Delay_Response`
5. Slave computes offset and adjusts clock

---

## Hardware Support

To achieve high accuracy (sub-microsecond), **hardware timestamping** is required in:

- Network Interface Cards (NICs)
- Switches (Transparent or Boundary Clocks)
- FPGAs / SOCs / microcontrollers (if used as time-aware nodes)

---

## PTP Versions

| Version | Spec | Notes |
|--------|------|-------|
| v1     | IEEE 1588-2002 | Initial release, limited adoption |
| v2     | IEEE 1588-2008 | Most common, supports transparent clocks |
| v2.1   | IEEE 1588-2019 | Added security, improvements |

---

## Configuration Tips

- Use **hardware timestamping** when possible.
- Ensure only **one master** clock is elected (monitor BMC algorithm).
- Avoid network congestion between PTP devices.
- Isolate PTP traffic if needed (VLANs, QoS).
- Use **boundary or transparent switches** to avoid delay jitter.

---

## Troubleshooting

| Symptom | Potential Cause | Fix |
|--------|------------------|-----|
| Clock drift | Missing hardware timestamping | Use PTP-capable NIC |
| Random jumps | Multiple masters present | Lock down master election |
| High jitter | Congested network | Isolate PTP on VLAN or QoS |
| Incorrect offsets | Switches add delay | Use transparent/boundary clocks |
| Sync failures | Firewall or switch blocking PTP packets | Allow UDP ports (typically 319, 320) |

---

## Similar Systems

| Protocol | Description | Use Case |
|----------|-------------|----------|
| **NTP** | Network Time Protocol | Millisecond accuracy, not ideal for robotics |
| **White Rabbit** | CERN extension of PTP with optical links | Nanosecond accuracy |
| **IRIG-B** | Analog/digital timing used in aerospace/defense | Often over coax/fiber |
| **TTEthernet** | Time-Triggered Ethernet (DO-178/AFDX variant) | Used in avionics |
| **SyncE** | Ethernet-level frequency synchronization | Telco-grade networks |

---

## PTP vs NTP

| Feature | PTP | NTP |
|--------|-----|-----|
| Accuracy | Sub-microsecond | Millisecond |
| Hardware Support | Yes | No (software only) |
| Jitter tolerance | Low | High |
| Traffic Type | Multicast / Unicast | Unicast |
| Use Case | Robotics, Industrial | General-purpose |

---

## Best Practices for Autonomous Systems

- Use **Boundary Clock Switches** between sensor networks.
- Sync ECUs to a common Grandmaster (e.g., GPS-disciplined clock).
- Use **gPTP** (802.1AS) for TSN-based real-time Ethernet.
- Log offsets and timestamps for forensic debugging.
- Ensure all subsystems can tolerate temporary loss of sync gracefully.

---

## See Also

- [[Ethernet]]
- [[Synchronized Sensor Fusion]]
- [[CAN Time Synchronization]]
- [[Autonomous Vehicle Architectures]]
- [[Timestamping Hardware]]

---

## External Links

- [IEEE 1588 Official](https://ieee1588.nist.gov)
- [Linux PTP Project](https://linuxptp.sourceforge.net/)
- [PTP vs NTP Comparison – Meinberg](https://www.meinbergglobal.com/english/info/ntp-vs-ptp.htm)

---
