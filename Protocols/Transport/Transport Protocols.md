---
title: Transport Protocols
tags: [protocols, networking, transport-layer, osi]
aliases: [Transport Layer Protocols, OSI Layer 4 Protocols]
---

# 🚚 Transport Protocols

## 🧭 Overview

**Transport protocols** operate at the **transport layer (Layer 4)** of the OSI model. They are responsible for end-to-end communication between devices, ensuring data is reliably delivered (or not, depending on the protocol) across a network. Transport protocols manage tasks like error detection, flow control, and session multiplexing.

The two most widely used transport protocols are **TCP (Transmission Control Protocol)** and **UDP (User Datagram Protocol)**, but other protocols exist for specialized use cases.

---

## 🛠️ Key Features of Transport Protocols

1. **End-to-End Communication**:
   - Provide a direct communication channel between applications on different devices.

2. **Reliability**:
   - Some protocols (e.g., TCP) ensure reliable delivery of data, while others (e.g., UDP) prioritize speed over reliability.

3. **Multiplexing**:
   - Allow multiple applications to share the same network connection using ports.

4. **Error Detection and Correction**:
   - Include mechanisms to detect and correct errors in transmitted data.

5. **Flow Control**:
   - Manage the rate of data transmission to prevent overwhelming the receiver.

---

## 📦 Common Transport Protocols

### [[TCP]] (Transmission Control Protocol)
- **Purpose**: Reliable, connection-oriented communication.
- **Key Features**:
  - Ensures data is delivered in order and without errors.
  - Uses a three-way handshake to establish connections.
  - Implements flow control and congestion control.
- **Use Cases**:
  - Web browsing (HTTP/HTTPS).
  - File transfers (FTP).
  - Email (SMTP, IMAP, POP3).

---

### [[UDP]] (User Datagram Protocol)
- **Purpose**: Fast, connectionless communication.
- **Key Features**:
  - No guarantee of delivery, order, or error correction.
  - Lightweight and low-latency.
- **Use Cases**:
  - Video streaming.
  - Online gaming.
  - DNS queries.

---

### [[QUIC]] (Quick UDP Internet Connections)
- **Purpose**: Modern transport protocol designed to improve the performance of HTTP/3.
- **Key Features**:
  - Built on UDP but adds reliability, encryption, and multiplexing.
  - Reduces connection setup latency compared to TCP.
- **Use Cases**:
  - HTTP/3 communication.
  - Real-time applications requiring low latency.

---

### [[SCTP]] (Stream Control Transmission Protocol)
- **Purpose**: Reliable, message-oriented communication.
- **Key Features**:
  - Supports multi-streaming (multiple independent streams within a single connection).
  - Provides multi-homing (multiple IP addresses for redundancy).
  - Combines features of TCP and UDP.
- **Use Cases**:
  - Telephony signaling (e.g., SS7 over IP).
  - Real-time applications requiring reliability and low latency.

---

### [[DCCP]] (Datagram Congestion Control Protocol)
- **Purpose**: Connection-oriented protocol for unreliable data delivery.
- **Key Features**:
  - Adds congestion control to UDP-like communication.
  - Suitable for applications that can tolerate some data loss but need congestion management.
- **Use Cases**:
  - Streaming media.
  - VoIP (Voice over IP).

---

### [[RTP]] (Real-Time Transport Protocol)
- **Purpose**: Protocol for delivering real-time data, such as audio and video.
- **Key Features**:
  - Works with UDP for low-latency communication.
  - Includes timestamps and sequence numbers for synchronization.
- **Use Cases**:
  - Video conferencing.
  - Live streaming.
  - Online gaming.

---

### [[SPX]] (Sequenced Packet Exchange)
- **Purpose**: Legacy protocol for reliable communication.
- **Key Features**:
  - Connection-oriented, similar to TCP.
  - Used in older Novell NetWare systems.
- **Use Cases**:
  - Legacy systems and applications.

---

## ✅ Pros and ❌ Cons of Transport Protocols

### ✅ Advantages
- **Reliability**: Protocols like TCP ensure data integrity and delivery.
- **Flexibility**: Different protocols are optimized for different use cases (e.g., UDP for speed, TCP for reliability).
- **Scalability**: Transport protocols enable efficient communication across large networks.

### ❌ Disadvantages
- **Overhead**: Reliable protocols like TCP introduce additional latency and resource usage.
- **Complexity**: Managing multiple transport protocols in a system can increase complexity.
- **Specialization**: Some protocols (e.g., SCTP, DCCP) are not widely supported, limiting their adoption.

---

## 🆚 Comparisons of Transport Protocols

| **Protocol**   | **Type**            | **Reliability** | **Latency** | **Use Cases**                     | **Strengths**                     | **Weaknesses**                   |
|-----------------|---------------------|-----------------|-------------|------------------------------------|------------------------------------|-----------------------------------|
| **TCP**        | Connection-Oriented | Reliable        | High        | Web browsing, file transfers       | Reliable, in-order delivery       | High overhead, slower            |
| **UDP**        | Connectionless      | Unreliable      | Low         | Streaming, gaming, DNS             | Lightweight, low latency          | No reliability or error recovery |
| **QUIC**       | Connection-Oriented | Reliable        | Very Low    | HTTP/3, real-time apps             | Low latency, built-in encryption  | Limited adoption                 |
| **SCTP**       | Connection-Oriented | Reliable        | Moderate    | Telephony, real-time apps          | Multi-streaming, multi-homing     | Limited support                  |
| **DCCP**       | Connection-Oriented | Unreliable      | Low         | Streaming, VoIP                    | Congestion control, low latency   | Rarely used                      |
| **RTP**        | Connectionless      | Unreliable      | Very Low    | Video conferencing, live streaming | Real-time delivery, synchronization | Requires additional protocols    |

---

## 🔗 Related Topics

- [[Protocols]]
- [[Application Protocols]]
- [[Networking Basics]]
- [[OSI Model]]

---

## 📚 Further Reading

- [RFC 793: TCP](https://datatracker.ietf.org/doc/html/rfc793)
- [RFC 768: UDP](https://datatracker.ietf.org/doc/html/rfc768)
- [RFC 9000: QUIC](https://datatracker.ietf.org/doc/html/rfc9000)
- [RFC 4960: SCTP](https://datatracker.ietf.org/doc/html/rfc4960)
- [RFC 3550: RTP](https://datatracker.ietf.org/doc/html/rfc3550)
- [RFC 4340: DCCP](https://datatracker.ietf.org/doc/html/rfc4340)

---

## 🧠 Summary

Transport protocols are critical for enabling reliable and efficient communication between devices. While TCP and UDP dominate most use cases, protocols like QUIC, SCTP, and RTP address specific needs for modern applications. Understanding the strengths and weaknesses of each protocol is essential for designing robust and scalable networked systems.
