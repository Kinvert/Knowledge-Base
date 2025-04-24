---
title: Routing Protocols
tags: [protocols, routing, networking, osi, internet]
aliases: [Network Routing Protocols, Internet Routing Protocols]
---

# 🛣️ Routing Protocols

## 🧭 Overview

**Routing protocols** are used to determine the best path for data to travel across a network. They enable routers to exchange information about network topology and dynamically adapt to changes, ensuring efficient and reliable data delivery.

Routing protocols are essential for both small-scale networks (e.g., LANs) and large-scale networks (e.g., the internet). They are categorized into **interior gateway protocols (IGPs)** for routing within an autonomous system (AS) and **exterior gateway protocols (EGPs)** for routing between autonomous systems.

---

## 🛠️ Key Features of Routing Protocols

1. **Dynamic Path Selection**:
   - Automatically determine the best path based on metrics like distance, bandwidth, or delay.

2. **Scalability**:
   - Support networks of varying sizes, from small LANs to global internet routing.

3. **Fault Tolerance**:
   - Adapt to network changes, such as link failures or topology updates.

4. **Convergence**:
   - Ensure all routers in the network have consistent routing information.

5. **Interoperability**:
   - Standardized protocols enable communication between devices from different vendors.

---

## 📦 Common Routing Protocols

### [[RIP]] (Routing Information Protocol)
- **Purpose**: Distance-vector protocol for small networks.
- **Key Features**:
  - Uses hop count as the metric (maximum: 15 hops).
  - Periodic updates every 30 seconds.
  - Simple but slow convergence.
- **Use Cases**:
  - Small LANs.
  - Legacy systems.

---

### [[OSPF]] (Open Shortest Path First)
- **Purpose**: Link-state protocol for large and complex networks.
- **Key Features**:
  - Uses Dijkstra's algorithm to calculate the shortest path.
  - Divides networks into areas for scalability.
  - Fast convergence and supports authentication.
- **Use Cases**:
  - Enterprise networks.
  - Campus networks.
  - Dynamic routing within an autonomous system.

---

### [[EIGRP]] (Enhanced Interior Gateway Routing Protocol)
- **Purpose**: Cisco proprietary hybrid protocol combining distance-vector and link-state features.
- **Key Features**:
  - Uses Diffusing Update Algorithm (DUAL) for fast convergence.
  - Supports unequal-cost load balancing.
  - Easy to configure and scalable.
- **Use Cases**:
  - Cisco-based enterprise networks.
  - Dynamic routing in large networks.

---

### [[BGP]] (Border Gateway Protocol)
- **Purpose**: Path-vector protocol for routing between autonomous systems.
- **Key Features**:
  - Uses path attributes to determine the best path.
  - Highly scalable and policy-driven.
  - Backbone of the internet.
- **Use Cases**:
  - Internet backbone routing.
  - Large-scale enterprise networks.
  - Multi-homed networks.

---

### [[IS-IS]] (Intermediate System to Intermediate System)
- **Purpose**: Link-state protocol for large networks.
- **Key Features**:
  - Similar to OSPF but operates at Layer 2.
  - Supports large-scale networks with hierarchical design.
  - Widely used by ISPs.
- **Use Cases**:
  - Service provider networks.
  - Large enterprise networks.

---

### [[IGRP]] (Interior Gateway Routing Protocol)
- **Purpose**: Legacy Cisco proprietary distance-vector protocol.
- **Key Features**:
  - Uses composite metrics (bandwidth, delay, reliability, load).
  - Replaced by EIGRP.
- **Use Cases**:
  - Legacy Cisco networks.

---

### [[Static Routing]]
- **Purpose**: Manually configured routes for small or simple networks.
- **Key Features**:
  - No dynamic updates or overhead.
  - Requires manual configuration and maintenance.
  - Best for predictable, fixed networks.
- **Use Cases**:
  - Small networks.
  - Backup routes.
  - Simple point-to-point connections.

---

### [[Dynamic Routing]]
- **Purpose**: Automatically updates routes based on network changes.
- **Key Features**:
  - Uses routing protocols like OSPF, EIGRP, or BGP.
  - Adapts to topology changes.
  - Reduces administrative overhead.
- **Use Cases**:
  - Large, complex networks.
  - Networks with frequent topology changes.

---

## ✅ Pros and ❌ Cons of Routing Protocols

### ✅ Advantages
- **Dynamic Adaptation**: Automatically adjust to network changes.
- **Scalability**: Support networks of varying sizes and complexities.
- **Efficiency**: Optimize data delivery paths based on metrics.

### ❌ Disadvantages
- **Complexity**: Dynamic routing protocols can be difficult to configure and troubleshoot.
- **Overhead**: Protocols like OSPF and BGP consume bandwidth and processing power.
- **Convergence Time**: Some protocols (e.g., RIP) have slow convergence times.

---

## 🆚 Comparisons of Routing Protocols

| **Protocol**   | **Type**            | **Metric**          | **Convergence** | **Scalability** | **Use Cases**                     | **Strengths**                     | **Weaknesses**                   |
|-----------------|---------------------|---------------------|-----------------|-----------------|------------------------------------|------------------------------------|-----------------------------------|
| **RIP**        | Distance-Vector     | Hop Count           | Slow            | Low             | Small LANs, legacy systems         | Simple, easy to configure          | Limited scalability, slow        |
| **OSPF**       | Link-State          | Cost (bandwidth)    | Fast            | High            | Enterprise, campus networks        | Fast convergence, scalable         | Complex configuration            |
| **EIGRP**      | Hybrid              | Composite Metric    | Fast            | High            | Cisco-based enterprise networks    | Fast, supports unequal-cost paths  | Cisco proprietary                |
| **BGP**        | Path-Vector         | Path Attributes     | Moderate        | Very High       | Internet backbone, ISPs            | Highly scalable, policy-driven     | Complex setup                    |
| **IS-IS**      | Link-State          | Cost                | Fast            | High            | ISP networks, large enterprises    | Scalable, hierarchical design      | Limited vendor support           |
| **Static**     | Manual              | N/A                 | Instant         | Low             | Small, fixed networks              | Simple, no overhead                | No dynamic adaptation            |

---

## 🔗 Related Topics

- [[Protocols]]
- [[Networking Basics]]
- [[Transport Protocols]]
- [[Internet Architecture]]

---

## 📚 Further Reading

- [RIP Specification (RFC 2453)](https://datatracker.ietf.org/doc/html/rfc2453)
- [OSPF Specification (RFC 2328)](https://datatracker.ietf.org/doc/html/rfc2328)
- [BGP Specification (RFC 4271)](https://datatracker.ietf.org/doc/html/rfc4271)
- [IS-IS Overview](https://www.cisco.com/c/en/us/support/docs/ip/integrated-intermediate-system-to-intermediate-system-is-is/5739-isis.html)
- [EIGRP Overview](https://www.cisco.com/c/en/us/td/docs/ios-xml/ios/iproute_eigrp/configuration/xe-16/ire-xe-16-book/ire-overview.html)

---

## 🧠 Summary

Routing protocols are the backbone of modern networking, enabling efficient and reliable data delivery across networks of all sizes. From simple protocols like RIP to advanced options like OSPF and BGP, each protocol is optimized for specific use cases. Understanding their strengths and limitations is essential for designing scalable and resilient networks.
