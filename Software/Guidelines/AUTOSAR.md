# AUTOSAR (AUTomotive Open System ARchitecture)

## 📝 Summary

**AUTOSAR** is a worldwide development partnership of automotive manufacturers, suppliers, and tool developers that aims to establish an open and standardized software architecture for **automotive Electronic Control Units (ECUs)**. 

Its goals are to **standardize the software infrastructure**, enhance **reusability**, improve **modularity**, and **reduce development time** in automotive systems.

---

## 🏛️ Origins & Consortium

- Launched in 2003 by BMW, Bosch, Continental, Daimler, Siemens VDO, and Volkswagen.
- Now includes over **300 partners**, including OEMs, Tier 1 suppliers, and tool vendors.
- Managed by the **AUTOSAR Development Partnership**.

---

## 🧱 AUTOSAR Architectures

AUTOSAR defines two primary platforms:

### 1. **Classic Platform (CP)**
- Focuses on **deeply embedded, real-time systems**
- Designed for microcontrollers
- Used for traditional ECUs (e.g., powertrain, chassis, body control)

### 2. **Adaptive Platform (AP)**
- Designed for **high-performance computing** and **complex applications**
- Runs on multi-core processors with **POSIX**-compatible OS (e.g., Linux or QNX)
- Supports applications like **autonomous driving**, **over-the-air updates**, and **V2X**

---

## 🛠️ Key Features

| Feature               | Classic Platform        | Adaptive Platform           |
|-----------------------|-------------------------|-----------------------------|
| Target HW             | Microcontrollers        | High-end multi-core SoCs    |
| OS                    | AUTOSAR OS (OSEK-based) | POSIX (e.g., Linux, QNX)    |
| Language              | Mainly C                | C++, Python, others         |
| Updates               | Static linking          | Dynamic deployment possible |
| Use Cases             | ABS, airbag, powertrain | ADAS, infotainment, AI      |

---

## 📦 AUTOSAR Layers (Classic)

1. **Application Layer**  
   - Application-specific logic (e.g., cruise control)
2. **Runtime Environment (RTE)**  
   - Acts as middleware between application and BSW
   - Auto-generated
3. **Basic Software (BSW)**  
   - Services like diagnostics, memory, communication
   - Includes MCAL (Microcontroller Abstraction Layer)
4. **Microcontroller**  
   - Hardware-specific layer

---

## 🧩 Adaptive Platform Key Concepts

- **Service-oriented architecture (SOA)**
- **Execution Management** to control startup/shutdown
- **Update and Configuration Management (UCM)**
- **Communication Management** via SOME/IP, DDS
- Use of **ARA::** namespace for services (e.g., ARA::Com)

---

## ⚙️ Standards and Protocols Used

| Area                  | Standards/Protocols             |
|-----------------------|---------------------------------|
| Timing                | IEEE 802.1AS, gPTP              |
| Communication         | CAN, CAN FD, LIN, FlexRay, Ethernet, SOME/IP |
| OS                    | OSEK, POSIX                     |
| Safety                | ISO 26262                       |
| Serialization         | Google Protobuf, CDR (RTPS)     |
| Networking (Adaptive) | SOME/IP, DDS                    |

---

## 🔍 Comparison with Related Standards

| Standard         | Focus Area                  | Comparison to AUTOSAR                      |
|------------------|-----------------------------|--------------------------------------------|
| ISO 26262        | Functional safety           | Complements AUTOSAR safety compliance      |
| MISRA            | Coding guidelines           | AUTOSAR often requires MISRA compliance    |
| DDS              | Middleware communication    | Used in Adaptive Platform                  |
| ROS              | Research-focused robotics   | More dynamic and less safety-driven        |
| POSIX            | OS API standard             | Basis for Adaptive OS layer                |

---

## 🎯 Use Cases

- Powertrain control
- Body electronics
- Infotainment
- ADAS (Advanced Driver Assistance Systems)
- Autonomous driving platforms
- Over-the-air update systems

---

## ✅ Pros and ❌ Cons

### ✅ Pros
- Strong support for **modular development**
- Improves **code reuse** and **interoperability**
- Compliant with **safety standards**
- Extensive **toolchain support**
- Scalability for both small ECUs and complex compute nodes

### ❌ Cons
- Steep **learning curve**
- Significant **tooling and licensing costs**
- Complex **configuration and integration**
- Not well-suited for rapidly evolving prototyping environments

---

## 🔧 Tooling Ecosystem

| Tool Type           | Examples                                 |
|---------------------|------------------------------------------|
| Configuration       | Vector DaVinci Developer, EB tresos      |
| Code Generation     | RTE Generators, ARXML-based tools        |
| Validation/Analysis | CANoe, CANalyzer, Lauterbach             |
| Simulink Integration| TargetLink, Embedded Coder               |

---

## 🧠 See Also

- [[ISO 26262]]
- [[MISRA]]
- [[SOME/IP]]
- [[DDS]]
- [[gPTP]]
- [[Adaptive AUTOSAR]]
- [[RTE]] (Runtime Environment)
