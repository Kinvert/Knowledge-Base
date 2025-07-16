# CPU Socket

A **CPU socket** is a physical and electrical interface on a motherboard that allows a CPU to be installed and communicate with the rest of the system. It plays a critical role in determining **CPU compatibility**, **platform features**, **memory support**, and **lane availability for PCIe**. In engineering, robotics, and ML workloads, selecting the right CPU socket ensures you get the right balance of compute power, I/O, and upgradeability.

Different sockets exist for different processor families (e.g. AMD vs Intel) and generations, and often a new CPU requires a new motherboard socket.

---

## 🧠 Overview

- Determines physical compatibility of a CPU with a motherboard  
- Directly influences memory channels, PCIe lane availability, and platform features  
- Varies between desktop, server, and embedded systems  
- Not backward-compatible across generations in many cases  
- Affects cooling requirements (different mounting standards)

---

## 🧪 Use Cases in Engineering

- **Robotics**: Socketed CPUs in custom control PCs or SBCs with embedded sockets  
- **Simulation**: Workstations with high-core CPUs on large sockets (e.g., TR4 for Threadripper)  
- **AI/ML**: Workstations with PCIe lanes tied to socket/chipset (e.g., x16 GPU lanes)  
- **Embedded**: Soldered-down CPUs with specific thermal and form factor constraints (e.g., BGA)

---

## 📊 Socket Comparison Table

| Socket        | Manufacturer | Example CPUs                    | Chipset Examples    | Use Case                        |
|---------------|--------------|----------------------------------|---------------------|----------------------------------|
| LGA1700       | Intel        | Intel Core i9-14900K         | Z690, Z790          | High-end desktops, ML dev        |
| AM5           | AMD          | Ryzen 9 9950X                | X670E, B650         | Desktop workstations, gaming     |
| sTR5          | AMD          | Threadripper 7960X           | WRX90               | HEDT, simulation-heavy workloads |
| SP5           | AMD          | EPYC Genoa                      | Server boards        | AI training, enterprise compute  |
| BGA (various) | Intel/ARM    | Mobile/Embedded CPUs             | N/A (soldered)       | Embedded robotics, SBCs          |

---

## 🔧 Key Features

- **Pin vs. Land Grid Array (LGA)**: Pins on socket (Intel), or on CPU (older AMD)
- **Contact Count**: More pins = more memory channels, PCIe lanes  
- **Thermal Interface & Mounting**: Cooling system compatibility  
- **Chipset Support**: Determines peripheral connectivity and BIOS features  
- **Longevity**: Some sockets are short-lived, others (like AM4) are long-supported

---

## ✅ Pros

- Modularity: CPUs can be upgraded independently of other components  
- Higher performance: Compared to soldered-down (BGA) chips  
- Ecosystem flexibility: Support for overclocking, PCIe lanes, memory configs

---

## ❌ Cons

- Frequent changes in socket type between generations  
- Compatibility tied tightly to BIOS/chipset updates  
- Physical size may limit use in small enclosures  
- Embedded systems often cannot use socketed CPUs

---

## 🔗 Related Concepts

- [[CPUs]]  
- [[PCIe]]  
- [[DDR5]]  
- [[CPU Cache]]

---

## 📚 Further Reading

- [CPU Socket Types – Wikipedia](https://en.wikipedia.org/wiki/CPU_socket)  
- [Intel Socket Info](https://www.intel.com/content/www/us/en/processors/processor-numbers.html)  
- [AMD Socket Info](https://www.amd.com/en/chipsets/sockets)

---
