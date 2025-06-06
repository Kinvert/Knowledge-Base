# RTE (Runtime Environment)

## 📝 Summary

The **RTE (Runtime Environment)** is the **middleware layer** in the **AUTOSAR Classic Platform** that connects **Application Software Components** to the **Basic Software (BSW)** modules and to **each other**. It acts as an abstraction layer, providing a standardized interface so that application developers can build modular and reusable components without needing to know the specifics of the hardware or services underneath.

---

## 🧠 Key Roles

- Abstracts communication between software components and BSW
- Enables portability and reusability of components
- Handles scheduling, signal routing, and access to services
- Generated automatically based on system configuration (ARXML)

---

## 🧱 Position in AUTOSAR Architecture

+------------------------+
| Application Layer |
| (SW Components/Ports) |
+----------▲-------------+
|
Runtime Environment (RTE)
|
+----------▼-------------+
| Basic Software |
+------------------------+
| Microcontroller |
+------------------------+

---

## 🔌 Interfaces Handled by RTE

- **Client-Server** (function call style)
- **Sender-Receiver** (data exchange via signals)
- Mode management
- Diagnostic events and services
- Communication abstraction (CAN, LIN, Ethernet, etc.)

---

## 🛠️ RTE Generation

- **RTE is auto-generated** using AUTOSAR tools (e.g., Vector DaVinci, EB tresos)
- Inputs:
  - SW-C descriptions (SWC ARXML)
  - System topology
  - Service needs (e.g., COM stack, OS services)
- The generated RTE includes:
  - Glue code to call APIs
  - Scheduler entries
  - Communication handlers

---

## ⚙️ Execution

- RTE acts like a **dispatcher** or **intermediary**:
  - For Sender-Receiver, it copies data between components
  - For Client-Server, it maps calls to target functions
- Makes use of the underlying OS tasks/interrupts for scheduling

---

## 🧪 Debugging and Troubleshooting

| Issue                     | Common Causes                         |
|--------------------------|----------------------------------------|
| Unmet RTE dependency     | Missing BSW configuration or service   |
| Missing RTE API in build | RTE not regenerated or outdated ARXML  |
| Runtime crash            | Incompatible or invalid port mapping   |
| Timing issues            | OS task priorities or cycle mismatches |

---

## 🔄 Classic vs Adaptive

| Feature                     | RTE (Classic)             | RTE (Adaptive)              |
|-----------------------------|---------------------------|-----------------------------|
| Code Generation             | Yes (compile-time)        | No (dynamic binding)        |
| Port Binding                | Static                    | Dynamic                     |
| Scheduling                  | OS-based tasks            | Execution Manager (EM)      |
| Middleware                  | Proprietary (RTE)         | SOME/IP, DDS, ARA::Com      |

---

## ✅ Pros and ❌ Cons

### ✅ Pros
- Highly abstracted, modular communication
- Automates glue code, reducing developer burden
- Enables formal testing and traceability
- Platform-independent application development

### ❌ Cons
- Tooling and configuration are complex
- Limited flexibility at runtime (classic)
- Performance overhead in high-throughput cases

---

## 🔗 Related Concepts

- [[AUTOSAR]]
- [[MCAL]]
- [[Basic Software (BSW)]]
- [[Client-Server Communication]]
- [[Sender-Receiver Communication]]
