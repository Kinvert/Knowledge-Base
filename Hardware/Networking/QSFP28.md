# QSFP28

**QSFP28** (Quad Small Form-factor Pluggable 28) is a high-speed optical/electrical transceiver module commonly used in data centers, high-performance computing, and networking equipment. It supports data rates up to 100 Gbps and is designed for scalability, offering high bandwidth in a compact form factor.

---

## 🔎 Overview

QSFP28 is part of the **QSFP family** of transceiver standards. The "28" refers to the fact that each electrical lane supports up to 28 Gbps, and with 4 lanes combined, it delivers 100 Gbps of throughput. It is widely used in switches, routers, and storage systems to provide efficient, high-density connectivity.

---

## 🧠 Core Concepts

- **Form Factor**: Pluggable module standardized for high-density ports.
- **Lane Configuration**: 4 lanes × 25/28 Gbps each = 100 Gbps aggregate.
- **Compatibility**: Backward compatible with QSFP+, QSFP10, and other QSFP standards.
- **Media Types**: Available in DAC (Direct Attach Copper), AOC (Active Optical Cable), and optical transceivers (SR4, LR4, CWDM4, etc.).
- **Applications**: Primarily for 100G Ethernet, but also supports InfiniBand and OTU4 (Optical Transport).

---

## 📊 Comparison Chart

| Standard    | Max Data Rate | Lanes | Use Case                          | Typical Distance |
|-------------|---------------|-------|-----------------------------------|------------------|
| **QSFP+**   | 40 Gbps       | 4×10G | Data center, short-haul           | <150m (optical)  |
| **QSFP28**  | 100 Gbps      | 4×25G | High-speed networking, backbone   | Up to 10 km+     |
| **QSFP-DD** | 400 Gbps      | 8×50G | Next-gen data center connectivity | Up to 10 km+     |
| **SFP28**   | 25 Gbps       | 1×25G | Access layer, short links         | <10 km           |
| **CFP4**    | 100 Gbps      | 4×25G | Legacy 100G deployments           | <10 km           |
| **OSFP**    | 400–800 Gbps  | 8×50/100G | Hyperscale data centers         | <10 km+          |

---

## 🛠️ Use Cases

- 100G Ethernet interconnects in data centers
- High-performance computing clusters
- Storage networks
- Long-haul optical transport (LR4, CWDM4, ER4 variants)
- Short-range interconnects with DAC/AOC

---

## ✅ Strengths

- High bandwidth density
- Backward compatibility with earlier QSFP standards
- Supports multiple media types (copper, optical, AOC)
- Broad ecosystem support in networking gear
- Standardized and widely available

---

## ❌ Weaknesses

- Limited to 100 Gbps compared to newer standards (QSFP-DD, OSFP)
- Power consumption higher than SFP28 for similar per-lane speeds
- Some variants require cooling solutions in dense deployments

---

## 🧩 Related Concepts

- [[QSFP+]] (40 Gbps predecessor)
- [[QSFP-DD]] (400 Gbps successor)
- [[OSFP]] (Next-gen optical form factor)
- [[SFP28]] (25 Gbps smaller form factor)
- [[Ethernet]] (Networking protocol family)
- [[InfiniBand]] (High-performance interconnect)
- [[Fiber Optics]] (Transmission medium)

---

## 🔗 Compatible Items

- Works in 100G-capable switches, NICs, and routers
- Compatible with DAC/AOC cables and fiber modules
- Interoperable with [[Ethernet]] and [[InfiniBand]] standards
- Can be used in breakout configurations (4×25G → 100G)

---

## 🧪 Variants

- **QSFP28 SR4**: Short-range (100 m over MMF)
- **QSFP28 LR4**: Long-range (10 km over SMF)
- **QSFP28 CWDM4**: 2 km reach, lower cost than LR4
- **QSFP28 ER4**: Extended range (30–40 km)
- **QSFP28 DAC**: Passive copper, up to 5 m
- **QSFP28 AOC**: Active optical cable, up to 100 m

---

## 📚 External Resources

- MSA (Multi-Source Agreement) specifications
- Ethernet Alliance: https://ethernetalliance.org/
- FS QSFP28 overview: https://community.fs.com
- IEEE 802.3ba/802.3bj Ethernet standards

---

## 📝 Summary

QSFP28 is a compact, high-speed, and widely adopted transceiver standard enabling **100G networking** in modern data centers and HPC environments. It offers a good balance between density, cost, and performance, with multiple variants available for short- to long-range deployments.

---
