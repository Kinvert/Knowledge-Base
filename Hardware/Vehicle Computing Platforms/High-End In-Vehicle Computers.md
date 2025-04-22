---
title: High-End In-Vehicle Computers for ADAS and Autonomous Driving
tags: [ADAS, autonomous driving, in-vehicle computers, high-performance computing, b-plus, EB Assist, Bosch, Acrosser]
aliases: [ADAS Computers, Autonomous Vehicle Computers, In-Vehicle HPC]
---

# 🚗 High-End In-Vehicle Computers for ADAS and Autonomous Driving

## Overview

High-end in-vehicle computers are specialized systems designed to handle the intensive computational requirements of ADAS and autonomous driving. These systems integrate powerful processors, extensive I/O capabilities, and robust storage solutions to process data from various sensors in real-time.

---

## 🖥️ Notable Systems

### 1. **b-plus DATALynx ATX4**

- **Manufacturer**: b-plus technologies GmbH
- **Form Factor**: 4U / 19-inch vehicle server
- **Power Supply**: Custom 100A DC with integrated liquid cooling
- **Expansion**: Supports 6–7 PCIe cards
- **Networking**: Onboard 10Gb LAN
- **Operating Temperature**: -10°C to +60°C without throttling
- **Use Case**: High-performance data logging and processing for in-vehicle applications
- **Standards**: IEEE 1588v2 (PTP), IEEE 802.1AS-2020 (gPTP)
- **Peripherals**: Compatible with Brickplus NVMe hot-swap hard drives
- **Reference**: [b-plus DATALynx ATX4 Datasheet](https://www.b-plus.com/fileadmin/data_storage/Data_Storage/Produkte/Produktdatenblaetter_AE/Datasheet_DATALynx-ATX4_EN_0.3.pdf)

### 2. **EB Assist ADAS Logger**

- **Manufacturer**: Elektrobit
- **Power Supply**: 6–32V DC automotive PSU with ignition support
- **Memory**: Up to 1024GB DDR4-3200 ECC RDIMM
- **Storage**: 10 exchangeable SATA SSDs
- **Expansion**: 7 PCIe slots (5x PCIe 4.0 x16, 2x PCIe 4.0 x8)
- **Processors**: AMD EPYC™ 7002/7003, Intel® Xeon® Scalable
- **Cooling**: Liquid-cooled processors and power supply
- **Use Case**: Automated driving validation and data logging
- **Reference**: [EB Assist Logger Solution](https://www.elektrobit.com/products/automated-driving/eb-assist/logger-solution/)

### 3. **Bosch ADAS Integration Platform**

- **Manufacturer**: Bosch Mobility
- **Design**: SoC-based ASIL D for safety-critical applications
- **Computing Power**: 10 to 1000 TOPS
- **Interfaces**: Multiple CAN and Ethernet
- **Use Case**: Driver assistance systems up to SAE Level 2
- **Reference**: [Bosch ADAS Integration Platform](https://www.bosch-mobility.com/en/solutions/vehicle-computer/adas-integration-platform/)

### 4. **Acrosser AAD-C622A**

- **Manufacturer**: Acrosser
- **GPU Support**: Up to 5 GPU cards
- **Features**:
  - Combination of high-end server and rugged in-vehicle computer
  - Rich I/O capacity
  - Wide-range DC power system
  - Hot-swappable SSD modules
- **Use Case**: Autonomous driving and edge AI computing
- **Reference**: [Acrosser AAD-C622A](https://www.acrosser.com/en/Products/Autonomous-Driving-Controller/Autonomous-Driving-Controller/AAD-C622AX)

### 5. **b-plus MI5**

- **Manufacturer**: b-plus technologies GmbH
- **Form Factor**: Modular in-vehicle computer
- **Processor Support**: Intel® Core™ i7/i5/i3
- **Memory**: Up to 32GB DDR4
- **Storage**: Supports multiple SSDs
- **Expansion**: Multiple PCIe slots for add-on cards
- **Networking**: Multiple Ethernet interfaces
- **Use Case**: Data logging and processing for ADAS applications
- **Standards**: IEEE 1588v2 (PTP), IEEE 802.1AS-2020 (gPTP)
- **Peripherals**: Compatible with Brickplus NVMe hot-swap hard drives
- **Reference**: b-plus MI5 Product Page

### 6. **b-plus MI5-Refresh**

- **Manufacturer**: b-plus technologies GmbH
- **Form Factor**: Updated modular in-vehicle computer
- **Processor Support**: Latest Intel® Core™ processors
- **Memory**: Enhanced memory capacity
- **Storage**: Improved storage options with faster access
- **Expansion**: Additional PCIe slots for increased flexibility
- **Networking**: Upgraded Ethernet interfaces
- **Use Case**: Advanced data logging and processing for autonomous driving
- **Standards**: IEEE 1588v2 (PTP), IEEE 802.1AS-2020 (gPTP)
- **Peripherals**: Compatible with Brickplus NVMe hot-swap hard drives
- **Reference**: b-plus MI5-Refresh Product Page

### 7. **CarPC**
- **Manufacturer**: Various (e.g., CappuccinoPC, Mini-Box)
- **Form Factor**: Compact in-vehicle PC
- **Processor Support**: Intel® Core™ i3/i5/i7
- **Memory**: Varies by model
- **Storage**: SSDs with varying capacities
- **Expansion**: Limited expansion options
- **Networking**: Standard Ethernet interfaces
- **Use Case**: Navigation, infotainment, and basic data logging
- **Standards**: Basic compliance with automotive standards
- **Peripherals**: Compatible with standard automotive peripherals
- **Reference**: CappuccinoPC CarPC Solutions

---

## 📊 Comparison Table

| Feature                     | DATALynx ATX4 | EB Assist Logger | Bosch ADAS Platform | Acrosser AAD-C622A | MI5 | MI5-Refresh | CarPC |
|-----------------------------|---------------|------------------|---------------------|--------------------|-----|-------------|-------|
| **Form Factor**             | 4U / 19-inch  | Custom chassis   | SoC-based module    | Rugged server      | Modular       | Modular        | Compact        |
| **Processor Support**       | High-end CPUs | AMD EPYC / Intel Xeon | Proprietary SoC | Supports multiple GPUs | Intel® Core™ i7/i5/i3 | Latest Intel® Core™ | Intel® Core™ i3/i5/i7 |
| **Memory Capacity**         | Not specified | Up to 1024GB DDR4 | Not specified       | Not specified      | Up to 32GB DDR4 | Enhanced capacity | Varies by model |
| **Storage Options**         | NVMe hot-swap | 10 SATA SSDs     | Not specified       | Hot-swappable SSDs | Multiple SSDs | Improved SSD options | SSDs with varying capacities |
| **Expansion Slots**         | 6–7 PCIe      | 7 PCIe slots     | Not specified       | Up to 5 GPU cards  | Multiple PCIe | Additional PCIe slots | Limited |
| **Networking**              | 10Gb LAN      | Multiple Ethernet | Multiple Ethernet   | Rich I/O capacity  | Multiple Ethernet interfaces | Upgraded Ethernet interfaces | Standard Ethernet interfaces |
| **Standards Compliance**    | IEEE 1588v2, IEEE 802.1AS-2020 | Not specified | ASIL D | Not specified | IEEE 1588v2, IEEE 802.1AS-2020 | IEEE 1588v2, IEEE 802.1AS-2020 | Basic automotive standards |

---

## 🧩 Key Features and Standards

- **Time Synchronization**: Support for IEEE 1588v2 (PTP) and IEEE 802.1AS-2020 (gPTP) ensures precise time alignment across systems.
- **Safety Standards**: Compliance with ASIL D design principles for safety-critical applications.
- **High-Speed Interfaces**: Multiple CAN and Ethernet interfaces for sensor and actuator connectivity.
- **Scalability**: Modular designs allowing for expansion with additional GPUs and storage as needed.

---

## 🔌 Peripheral Compatibility

- **Storage**: Support for hot-swappable NVMe and SATA SSDs, facilitating quick data retrieval and replacement.
- **Expansion**: Multiple PCIe slots for adding GPUs, FPGAs, or other specialized cards.
- **Networking**: High-speed Ethernet ports for rapid data transfer between systems.
- **Power Supply**: Wide-range DC inputs with automotive-grade surge protection.
