---
title: XTSS
tags: [xtss, ptp, gptp, ieee1588, ieee802.1as, time synchronization, b-plus, aveto, adas, autonomous vehicles]
aliases: [XTSS, eXtended Time Synchronization Services]
---

# 🕰️ XTSS – eXtended Time Synchronization Services

## Overview

**XTSS (eXtended Time Synchronization Services)** is a time synchronization solution developed by **[[B-Plus]] technologies GmbH**. Designed for advanced driver-assistance systems (ADAS) and autonomous driving applications, XTSS ensures precise time alignment across distributed systems, facilitating accurate data correlation and system coordination.

---

## 🏢 About b-plus

**b-plus technologies GmbH** specializes in hardware and software solutions for the automotive industry, focusing on data logging, time synchronization, and system integration for ADAS and autonomous driving. Their **AVETO Toolbox** integrates XTSS to provide comprehensive time synchronization capabilities.

---

## 🧩 XTSS Components

XTSS comprises two main services:

### 1. **CTSS – Cluster Time Synchronization Service**

- **Function**: Synchronizes time across multiple devices in a network.
- **Standards**:
  - **IEEE 1588v2**: Precision Time Protocol ([[PTP]])
  - **IEEE 802.1AS-2020**: Generalized Precision Time Protocol ([[gPTP]])
- **Use Case**: Ensures all devices in a cluster operate on a unified time base, crucial for synchronized data acquisition and processing.

### 2. **PTSS – Platform Time Synchronization Service**

- **Function**: Aligns internal clocks within a single device or measurement platform.
- **Mechanism**: Utilizes hardware-based synchronization methods to achieve high precision.
- **Use Case**: Maintains internal consistency, ensuring that all subsystems within a device are time-aligned.

---

## 📜 Supported Standards and Protocols

| Standard/Protocol | Description |
|-------------------|-------------|
| **IEEE 1588v2**   | Precision Time Protocol for synchronizing clocks over a network. |
| **IEEE 802.1AS-2020** | Generalized PTP for time-sensitive applications, ensuring precise timing over Ethernet. |

---

## ⚙️ Working Clock Domain

- **Purpose**: Provides a stable and interference-free time base for measurement and synchronization tasks.
- **Characteristics**:
  - Operates independently of global time sources like GPS.
  - Utilizes gPTP (IEEE 802.1AS) for synchronization over Ethernet.
  - Integrated into **domain 1** as defined in IEEE 802.1AS-2020.

---

## 🛠️ Setup and Configuration

1. **Hardware Requirements**:
   - Devices supporting hardware timestamping.
   - Network infrastructure compatible with PTP/gPTP protocols.

2. **Software Installation**:
   - Deploy XTSS components (CTSS and PTSS) on the respective devices.
   - Configure services according to the network topology and synchronization requirements.

3. **Network Configuration**:
   - Ensure network switches and routers support PTP/gPTP.
   - Configure Quality of Service (QoS) to prioritize synchronization traffic.

4. **Validation**:
   - Use diagnostic tools to verify synchronization accuracy.
   - Monitor for any discrepancies or drift in time alignment.

---

## 🧪 Compatibility

| Component | Compatible | Notes |
|-----------|------------|-------|
| **Operating Systems** | Yes | Compatible with both Windows and Linux platforms. |
| **Hardware Timestamping** | Yes | Requires NICs and switches that support hardware timestamping for optimal performance. |
| **Network Infrastructure** | Yes | Compatible with networks supporting IEEE 1588v2 and IEEE 802.1AS-2020. |

---

## 🧠 Challenges Addressed

- **Multi-Device Synchronization**: Ensures all devices in a distributed system operate on a unified time base.
- **High-Precision Requirements**: Achieves sub-microsecond synchronization accuracy.
- **Scalability**: Supports complex network topologies with multiple devices and subsystems.
- **Interference Mitigation**: Operates independently of external time sources, reducing susceptibility to interference.

---

## 🔄 Comparison with Similar Solutions

| Feature | XTSS | NTP | SyncE | TTEthernet |
|---------|------|-----|-------|------------|
| **Protocol** | PTP/gPTP | NTP | Synchronous Ethernet | Time-Triggered Ethernet |
| **Accuracy** | Sub-microsecond | Millisecond | Sub-microsecond | Sub-microsecond |
| **Hardware Support** | Required | Not required | Required | Required |
| **Use Case** | Automotive, ADAS | General-purpose | Telecom | Avionics, Industrial |
| **Scalability** | High | High | Medium | Medium |
