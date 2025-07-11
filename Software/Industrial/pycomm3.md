# pycomm3

**pycomm3** is a Python library for communicating with [[Allen-Bradley]] [[PLC]]s (Programmable Logic Controllers), such as ControlLogix and CompactLogix, over Ethernet/IP using the [[CIP]] protocol. It is a modern, actively maintained replacement for older libraries like `pycomm` and supports both reading and writing tags.

---

## 📚 Overview

pycomm3 allows engineers and developers to interface directly with industrial controllers using a Pythonic API. It is especially useful for automation, testing, diagnostics, and integrating PLCs into larger software systems, including those for data logging, visualization, or robotic coordination.

It is often used in control systems development and industrial integration scenarios where Python is part of the software stack.

---

## 🧠 Core Concepts

- **EtherNet/IP**: An industrial networking protocol used by Allen-Bradley PLCs  
- **CIP (Common Industrial Protocol)**: Layer on top of EtherNet/IP for tag-based communication  
- **Tag Access**: pycomm3 allows reading/writing to named variables (tags) in the PLC  
- **Session Management**: Handles connection lifecycle with automatic reconnects  
- **Structured Data Support**: Can access user-defined structures and arrays in the PLC  
- **Tag Browsing**: Explore available tags on the connected controller  

---

## 🧰 Use Cases

- Industrial automation scripting and diagnostics  
- Logging and monitoring PLC variables  
- Creating custom human-machine interfaces (HMIs)  
- Robotics integration with existing factory PLCs  
- Educational tools for learning PLC networking  

---

## ✅ Pros

- Pythonic and easy-to-use API  
- Active development and modern Python 3 support  
- Supports structured and scalar tag types  
- Asynchronous communication support  
- Tag browsing and metadata reading  

---

## ❌ Cons

- Only supports Allen-Bradley Ethernet/IP PLCs  
- Requires knowledge of PLC tag names  
- Not a real-time communication tool  
- May require firewalls and ports to be properly configured  

---

## 📊 Comparison: pycomm3 vs Alternatives

| Library       | Language | Allen-Bradley Support | Tag Browsing | Async Support | Open Source |
|---------------|----------|------------------------|---------------|---------------|--------------|
| pycomm3       | Python   | Yes (Logix-based PLCs) | Yes           | Yes           | Yes          |
| cpppo         | Python   | Partial                | No            | No            | Yes          |
| pylogix       | Python   | Yes                    | Partial       | No            | Yes          |
| libplctag     | C/C++    | Yes (multi-vendor)     | No            | No            | Yes          |
| Kepware       | Commercial | Yes (via OPC UA)     | Yes           | Depends       | No           |

---

## 🤖 In a Robotics Context

| Application                | pycomm3 Use Case                             |
|----------------------------|----------------------------------------------|
| PLC-Robot Integration      | Control robot behavior based on PLC tags  
| Data Logging               | Periodic reading of process variables  
| Remote Diagnostics         | Python-based monitoring over EtherNet/IP  
| SCADA Integration          | Reading sensor states or actuator commands  
| Testing and Debugging      | Write test patterns directly from Python  

---

## 🔧 Common Operations

- Reading a tag: `client.read('MyTag')`  
- Writing a tag: `client.write('MyTag', 123)`  
- Browsing tags: `client.browse()`  
- Connecting to PLC: `with LogixDriver('192.168.1.10') as client:`  

---

## 🔧 Compatible Items

- [[EtherCAT]] – Operates at lower level; pycomm3 interfaces with higher-level controllers  
- [[ROS2 Web Bridge]] – Can wrap pycomm3 calls for web-based interaction  
- [[Real-Time Systems]] – pycomm3 used for supervisory, not hard real-time tasks  
- [[CI-CD Pipelines]] – Can automate tests against hardware-in-the-loop PLCs  

---

## 🔗 Related Concepts

- [[Ethernet/IP]] (Network layer used by Allen-Bradley PLCs)  
- [[OPC UA]] (Another interface layer for industrial systems)  
- [[PLC]] (Programmable Logic Controllers)  
- [[SCADA]] (Supervisory Control and Data Acquisition systems)  
- [[Industrial Protocols]] (Fieldbus and Ethernet-based communication layers)  

---

## 📚 Further Reading

- [pycomm3 GitHub Repository](https://github.com/ottowayi/pycomm3)  
- [pycomm3 ReadTheDocs](https://pycomm3.readthedocs.io/)  
- [Rockwell Automation Ethernet/IP Overview](https://rockwellautomation.custhelp.com/)  
- [Wikipedia – EtherNet/IP](https://en.wikipedia.org/wiki/EtherNet/IP)  
- [Getting Started with pycomm3](https://pycomm3.readthedocs.io/en/latest/tutorial.html)

---

## 🗂 Suggested Folder Location

Protocols > Robotics and Industrial > pycomm3  
