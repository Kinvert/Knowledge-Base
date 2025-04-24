---
title: IPv4 (Internet Protocol Version 4)
tags: [protocols, networking, ipv4, osi, internet]
aliases: [Internet Protocol Version 4, IPv4 Addressing, IPv4 Protocol]
---

# 🌐 IPv4 (Internet Protocol Version 4)

## 🧭 Overview

**IPv4 (Internet Protocol Version 4)** is the fourth version of the Internet Protocol and one of the core protocols of the Internet. It operates at the **Network Layer (Layer 3)** of the OSI model and is responsible for delivering packets across interconnected networks using logical addressing.

IPv4 uses **32-bit addresses**, allowing for approximately **4.3 billion unique addresses**. Despite its limitations in address space, IPv4 remains the most widely deployed protocol for internet communication.

---

## 🛠️ Key Features

1. **Logical Addressing**:
   - Uses 32-bit addresses, typically represented in dotted decimal format (e.g., `192.168.1.1`).

2. **Connectionless Protocol**:
   - Operates without establishing a dedicated connection between sender and receiver.

3. **Fragmentation and Reassembly**:
   - Splits large packets into smaller fragments for transmission and reassembles them at the destination.

4. **Routing**:
   - Supports routing across multiple networks using routing protocols like OSPF, BGP, and RIP.

5. **Header Structure**:
   - Includes fields for source and destination addresses, time-to-live (TTL), and checksum for error detection.

6. **Broadcasting**:
   - Supports broadcast communication to send packets to all devices in a network.

---

## 📦 Common Use Cases

1. **Internet Communication**:
   - Core protocol for delivering data across the internet.

2. **Local Area Networks (LANs)**:
   - Used for communication within private networks.

3. **Routing**:
   - Enables data to travel across multiple networks using routers.

4. **Network Address Translation (NAT)**:
   - Extends IPv4 address space by allowing multiple devices to share a single public IP address.

---

## ✅ Pros and ❌ Cons

### ✅ Advantages
- **Simplicity**: Easy to configure and widely supported.
- **Compatibility**: Supported by virtually all devices and networks.
- **Efficiency**: Lightweight protocol with minimal overhead.

### ❌ Disadvantages
- **Limited Address Space**: Only ~4.3 billion addresses, leading to exhaustion.
- **Security**: No built-in encryption or authentication mechanisms.
- **Fragmentation Overhead**: Fragmentation can reduce performance and increase complexity.

---

## 🆚 Comparisons with IPv6

| Feature                | IPv4               | IPv6               |
|------------------------|--------------------|--------------------|
| **Address Size**       | 32-bit             | 128-bit            |
| **Address Space**      | ~4.3 billion       | ~340 undecillion   |
| **Header Complexity**  | Simple             | More fields, but optimized |
| **Security**           | No built-in encryption | Built-in IPsec    |
| **Broadcasting**       | Supported          | Replaced by multicast |
| **Deployment**         | Widely deployed    | Growing adoption   |

---

## 🛠️ How IPv4 Works

1. **Addressing**:
   - IPv4 addresses are divided into **network** and **host** portions using a subnet mask (e.g., `255.255.255.0`).

2. **Packet Delivery**:
   - Data is encapsulated into IPv4 packets, which include source and destination IP addresses.

3. **Routing**:
   - Routers use the destination IP address to forward packets to the appropriate network.

4. **Fragmentation**:
   - If a packet exceeds the maximum transmission unit (MTU) of a network, it is fragmented into smaller packets.

---

## 📜 IPv4 Address Classes

| Class   | Address Range       | Default Subnet Mask | Use Case                     |
|---------|---------------------|---------------------|------------------------------|
| **A**   | `0.0.0.0` - `127.255.255.255` | `255.0.0.0`       | Large networks              |
| **B**   | `128.0.0.0` - `191.255.255.255` | `255.255.0.0`     | Medium-sized networks       |
| **C**   | `192.0.0.0` - `223.255.255.255` | `255.255.255.0`   | Small networks              |
| **D**   | `224.0.0.0` - `239.255.255.255` | N/A               | Multicast                   |
| **E**   | `240.0.0.0` - `255.255.255.255` | N/A               | Reserved for future use     |

---

## 🔗 Related Topics

- [[IP]] (Internet Protocol)
- [[IPv6]]
- [[NAT]] (Network Address Translation)
- [[Routing Protocols]]
- [[OSI Model]]

---

## 📚 Further Reading

- [RFC 791: Internet Protocol](https://datatracker.ietf.org/doc/html/rfc791)
- [IPv4 Addressing and Subnetting](https://www.cisco.com/c/en/us/support/docs/ip/address-resolution-protocol-arp/13711-3.html)
- [IPv4 vs IPv6](https://www.cloudflare.com/learning/network-layer/ipv4-vs-ipv6/)
- [IANA IPv4 Address Space](https://www.iana.org/assignments/ipv4-address-space/ipv4-address-space.xhtml)

---

## 🧠 Summary

IPv4 is the backbone of modern internet communication, providing logical addressing and routing for billions of devices. Despite its limitations, such as address exhaustion, it remains widely used due to its simplicity and compatibility. The transition to IPv6 aims to address these limitations while ensuring the scalability of the internet.
