# PCIe

**PCI Express (PCIe)** is a high-speed serial expansion bus standard used to connect various hardware components directly to the CPU and motherboard. It is critical in robotics, AI/ML, and high-performance engineering systems, as it connects **GPUs**, **NVMe storage**, **FPGAs**, **network cards**, and more.

The number of lanes (e.g., x1, x4, x8, x16) and generation (Gen 3, Gen 4, Gen 5, Gen 6) determine bandwidth and performance characteristics. PCIe is integral to system throughput, especially when transferring large amounts of data between accelerators and the CPU.

---

## 🧠 Overview

- **Lane-based architecture**: Each lane has two pairs of wires (send + receive)  
- **Full-duplex communication** per lane  
- **Backward and forward compatible** with most modern motherboards  
- Core interconnect for **GPUs**, **NVMe SSDs**, **capture cards**, **NICs**, and more  
- Evolves rapidly with newer generations offering double the bandwidth of the previous

---

## 📊 PCIe Versions Comparison

| Generation | Year | Bandwidth / Lane (x1) | Bandwidth (x16) | Typical Use Cases                |
|------------|------|------------------------|------------------|----------------------------------|
| PCIe 3.0   | 2010 | 1 GB/s                 | 16 GB/s          | GPUs, SSDs, general peripherals  |
| PCIe 4.0   | 2017 | 2 GB/s                 | 32 GB/s          | High-speed SSDs, ML accelerators |
| PCIe 5.0   | 2019 | 4 GB/s                 | 64 GB/s          | AI/ML, data center GPUs          |
| PCIe 6.0   | 2022 | 8 GB/s                 | 128 GB/s         | Future AI, simulation clusters   |

*Bandwidth figures are per direction (full-duplex). Actual throughput may be lower depending on device/controller overhead.*

---

## 🔧 Key Features

- **Point-to-point communication** for direct device connection  
- **Switching fabric** architecture supports complex topologies  
- **Hot-pluggable** (in some enterprise configurations)  
- **Flexible lane counts**: 1x, 2x, 4x, 8x, 16x, and 32x in theory  
- **Low latency** and **high bandwidth** for accelerator-to-CPU paths

---

## 🧪 Use Cases in Engineering

- Connecting [[GPUs]] for deep learning and simulation  
- PCIe-based AI accelerators like TPUs, FPGAs  
- Real-time robotic vision with high-speed framegrabbers  
- NVMe SSD arrays for fast dataset loading or logging  
- Robotics platforms needing high-speed sensor interfaces (e.g., LiDAR capture cards)  
- ROS2 high-bandwidth data streaming over custom PCIe cards

---

## ✅ Pros

- Extremely fast and scalable  
- Standardized and broadly supported  
- Supports both consumer and enterprise hardware  
- Minimal overhead and low latency

---

## ❌ Cons

- Limited total number of lanes per CPU/socket  
- Lane sharing may reduce bandwidth per device  
- High-power, high-heat devices require adequate cooling and power delivery  
- Software stack configuration for DMA / memory mapping can be complex in some systems

---

## 🔗 Related Concepts

- [[CPUs]]  
- [[GPUs]]  
- [[DDR5]]  
- [[CPU Socket]]  

---

## 📚 Further Reading

- [PCI-SIG Official Specs](https://pcisig.com)  
- [PCIe Architecture Explained (AnandTech)](https://www.anandtech.com/show/14511/understanding-pcie-4-0-everything-you-need-to-know)  
- [Wikipedia: PCI Express](https://en.wikipedia.org/wiki/PCI_Express)  

---
