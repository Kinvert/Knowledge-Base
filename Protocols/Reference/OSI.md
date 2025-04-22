---
title: OSI Model
tags: [networking, protocols, osi, reference]
---

# OSI Model

The **OSI Model** (Open Systems Interconnection Model) is a conceptual framework that standardizes the functions of a telecommunication or computing system into seven abstraction layers. It helps in understanding and designing interoperable network protocols and systems.

---

## 🧱 Overview

- Developed by the **[ISO](https://www.iso.org/ics/35.100/x/)** (International Organization for Standardization)
- Standard reference model for communication between systems
- Divides networking into **7 layers**
- Promotes interoperability and modular development

---

## 📚 Layers Summary

| Layer | Name         | Function                                  | Examples                                             |
| ----- | ------------ | ----------------------------------------- | ---------------------------------------------------- |
| 7     | Application  | Interface for the end user                | [[HTTP]], [[FTP]], [[DNS]], [[SMTP]], [[WebSockets]] |
| 6     | Presentation | Data translation, encryption, compression | SSL/TLS, JPEG, MP3, ASCII, EBCDIC                    |
| 5     | Session      | Establishes, manages sessions             | NetBIOS, RPC, PPTP                                   |
| 4     | Transport    | Reliable data transfer, flow control      | [[TCP]], [[UDP]]                                     |
| 3     | Network      | Routing, addressing                       | IP, ICMP, IPsec, OSPF                                |
| 2     | Data Link    | MAC addressing, error detection           | [[Ethernet]], PPP, MAC, LLC                          |
| 1     | Physical     | Transmission media, bit-level             | Cables, Hubs, Fiber, Electrical signals              |

---

## 🔍 Layer-by-Layer Breakdown

### 🟪 [Layer 7](https://www.iso.org/ics/35.100.70/x/) – [Application](https://en.wikipedia.org/wiki/Application_layer)
- Closest to the end user
- Provides network services to applications
- **Examples**: HTTP, HTTPS, FTP, SMTP, WebSocket, DNS, Telnet

---

### 🟨 [Layer 6](https://www.iso.org/ics/35.100.60/x/) – [Presentation](https://en.wikipedia.org/wiki/Presentation_layer)
- Translates data between application and network formats
- Handles data **encryption**, **compression**, and **serialization**
- **Examples**: TLS, SSL, JPEG, MPEG, JSON, XML, ASCII, EBCDIC

---

### 🟧 [Layer 5](https://www.iso.org/ics/35.100.50/x/) – [Session](https://en.wikipedia.org/wiki/Session_layer)
- Manages sessions between applications
- Responsible for session **establishment**, **maintenance**, and **termination**
- **Examples**: RPC, NetBIOS, SMB (Session layer part)

---

### 🟥 [Layer 4](https://www.iso.org/ics/35.100.40/x/) – [Transport](https://en.wikipedia.org/wiki/Transport_layer)
- Ensures reliable data transmission
- Performs **segmentation**, **error checking**, **flow control**
- Supports **connection-oriented** (TCP) and **connectionless** (UDP) communication
- **Examples**: TCP, UDP, SCTP

---

### 🟩 [Layer 3](https://www.iso.org/ics/35.100.30/x/) – [Network](https://en.wikipedia.org/wiki/Network_layer)
- Determines **paths** for data using logical addressing
- Handles **routing**, **forwarding**, **fragmentation**
- **Examples**: IP, ICMP, IGMP, IPsec, OSPF, BGP

---

### 🟦 [Layer 2](https://www.iso.org/ics/35.100.20/x/) – [Data Link](https://en.wikipedia.org/wiki/Data_link_layer)
- Establishes and maintains reliable links between nodes
- Adds MAC addresses, detects/corrects errors in Layer 1
- Divided into two sublayers:
  - **LLC** (Logical Link Control)
  - **MAC** (Media Access Control)
- **Examples**: Ethernet, PPP, ARP, MAC, VLAN

---

### ⚫ [Layer 1](https://www.iso.org/ics/35.100.10/x/) – [Physical](https://en.wikipedia.org/wiki/Physical_layer)
- Transmits **raw bitstream** over physical medium
- Defines **electrical**, **optical**, and **mechanical** characteristics
- Concerned with voltage levels, cable types, connectors, signaling
- **Examples**: RJ45, Fiber Optics, Hubs, Repeaters, Twisted Pair, BNC

---

## 🧠 Related Acronyms

| Acronym | Meaning |
|--------|---------|
| **MAC** | Media Access Control |
| **LLC** | Logical Link Control |
| **IP** | Internet Protocol |
| **TCP** | Transmission Control Protocol |
| **UDP** | User Datagram Protocol |
| **ICMP** | Internet Control Message Protocol |
| **FTP** | File Transfer Protocol |
| **SSL/TLS** | Secure Sockets Layer / Transport Layer Security |

---

## 🧮 Mnemonics

### Bottom-Up (Layer 1 → 7):
> **"Please Do Not Throw Sausage Pizza Away"**

### Top-Down (Layer 7 → 1):
> **"All People Seem To Need Data Processing"**
