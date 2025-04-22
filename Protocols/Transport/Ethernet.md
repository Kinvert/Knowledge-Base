---
title: Ethernet
tags: [networking, protocols, osi, hardware, physical-layer, data-link-layer]
---

# Ethernet

**Ethernet** is a family of wired networking technologies used for local area networks (LANs), metropolitan area networks (MANs), and wide area networks (WANs). It defines how devices on a network format and transmit data at the physical and data link layers of the **OSI model**.

---

## 🧭 Overview

- Standardized as **IEEE 802.3**
- Operates at **OSI Layer 1 (Physical)** and **Layer 2 (Data Link)**
- Most common networking technology in use today
- Supports **half-duplex** and **full-duplex** communication
- Implements **CSMA/CD** (Carrier Sense Multiple Access with Collision Detection) for shared medium access (mainly legacy use)

---

## 🌐 Key Acronyms

| Acronym | Meaning |
|--------|---------|
| **LAN** | Local Area Network |
| **WAN** | Wide Area Network |
| **IEEE** | Institute of Electrical and Electronics Engineers |
| **NIC** | Network Interface Card |
| **OSI** | Open Systems Interconnection model |
| **LLC** | Logical Link Control |
| **MAC** | Media Access Control |
| **CSMA/CD** | Carrier Sense Multiple Access with Collision Detection |

---

## 🧱 OSI Layers Involved

### 🔌 Layer 1: Physical Layer

- Defines hardware transmission of raw bits over a medium
- Includes:
  - **Cable types**: Twisted pair (Cat5e, Cat6, etc.), fiber optics, coaxial (older)
  - **Connectors**: Primarily **RJ45** for twisted pair, LC/ST/SC for fiber
  - **Signaling**: Voltage levels, timing, modulation
  - **Speeds**: 10 Mbps → 100 Mbps → 1 Gbps → 10/40/100 Gbps+
  - **Duplexing**:
    - **Half-duplex**: Communication in one direction at a time (legacy hubs)
    - **Full-duplex**: Simultaneous bidirectional communication (modern switches)

---

### 🔗 Layer 2: Data Link Layer

- Responsible for framing, MAC addressing, and error detection
- Divided into:
  - **LLC (Logical Link Control)**: Error checking, flow control
  - **MAC (Media Access Control)**: Defines frame format, MAC addressing
- Uses **MAC addresses** to identify network devices
- Implements **CSMA/CD**:
  - Nodes "listen" before transmitting
  - If collision is detected, nodes back off and retry
  - Mainly used in older, half-duplex environments

---

## 🧮 Ethernet Frame Format

| Field             | Size (bytes) |
|------------------|--------------|
| Preamble          | 7            |
| Start Frame Delimiter | 1        |
| Destination MAC   | 6            |
| Source MAC        | 6            |
| EtherType / Length| 2            |
| Payload           | 46–1500      |
| CRC (FCS)         | 4            |

> Note: Jumbo frames can support payloads >1500 bytes depending on network equipment.

---

## 🧵 Cable Types

### 🟦 Twisted Pair (Copper)
- Common: **Cat5e**, **Cat6**, **Cat6a**, **Cat7**
- Maximum distances vary (~100 meters typical for Cat5e/Cat6)
- Uses **RJ45** connectors
- Cost-effective and easy to work with

### 🟣 Fiber Optic
- Supports much longer distances and higher bandwidth
- Immune to electromagnetic interference
- Uses connectors like **LC**, **SC**, **ST**
- Single-mode vs multi-mode

---

## 🖧 Common Ethernet Variants

| Name         | Speed     | Medium         | Notes                         |
|--------------|-----------|----------------|-------------------------------|
| 10BASE-T     | 10 Mbps   | Twisted pair   | Legacy, half/full-duplex      |
| 100BASE-TX   | 100 Mbps  | Twisted pair   | Fast Ethernet                 |
| 1000BASE-T   | 1 Gbps    | Twisted pair   | Gigabit Ethernet              |
| 10GBASE-T    | 10 Gbps   | Twisted pair   | Higher-end networking         |
| 1000BASE-X   | 1 Gbps    | Fiber optic    | Enterprise & backbone use     |
| 10GBASE-SR/LR| 10 Gbps   | Fiber optic    | Short/Long Reach (Datacenter) |

---

## 🧠 Related Topics

- [[OSI]]
- [[TCP]]
- [[TCP/IP]] [Internet Protocol Suite](https://en.wikipedia.org/wiki/Internet_protocol_suite)
- [[RJ45]]
- [[MAC Address]]
- [[CSMA/CD]]
- [[Twisted Pair]]
- [[Fiber Optics]]
- [[Switches]]
- [[NIC]]
