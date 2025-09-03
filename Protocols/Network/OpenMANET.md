# OpenMANET (Open Mobile Ad Hoc Network Framework)

The term **OpenMANET** typically refers to open-source frameworks, implementations, or enhancements for **Mobile Ad Hoc Networks (MANETs)**—self-configuring, infrastructure-free networks where nodes communicate dynamically. While there's no single “OpenMANET” standard, several open MANET toolsets and protocol extensions — such as INETMANET for OMNeT++, OSPF-MDR in Quagga/FRR, and others — embody the spirit of an open, extensible MANET platform. This note surveys those and compares their features and uses in robotics and embedded networking.

---

##  Overview

MANETs enable devices to form networks on the fly, ideal for applications like swarm robotics, disaster response, and mobile IoT systems. "OpenMANET" usually denotes open-source tools or extensions supporting MANET routing in simulators or real devices.

---

##  Core Concepts

- **Ad Hoc Routing**: Dynamic path discovery without fixed routers.
- **Self-Organizing Topology**: Nodes establish routes based on proximity and availability.
- **Open Source Frameworks**: Tools for developing, simulating, and deploying MANET protocols freely.
- **Protocol Flexibility**: Support for AODV, DSR, OLSR, GPSR, DSDV, DYMO, OSPF-MDR, and custom routing logic.

---

##  Comparison Chart

| Framework / Tool        | Domain                 | Key Features                                                                 | Typical Use Case                       |
|--------------------------|------------------------|------------------------------------------------------------------------------|----------------------------------------|
| **INETMANET (OMNeT++)**  | Simulation (INET stack) | Adds AODV, DSR, OLSR, DYMO, B.A.T.M.A.N. to OMNeT++ INET framework | Academic simulations, robotics network research |
| **INET (MANET showcase)** | Simulation (OMNeT++)   | Built-in MANET routing examples (DSDV, GPSR) with tuning via `omnetpp.ini` | Protocol evaluation and demo setups     |
| **OSPF-MDR (Quagga/FRR)**| Real-world routing      | Extends OSPFv3 for MANET using designated routers, implements RFCs 5614/5243/5838 | Real deployments in mobile ad hoc routing |
| **Custom C++ via OMNeT++** | Simulation             | Ability to write custom routing in C++ modules and integrate via NED/omnetpp.ini | Tailored protocol research              |

---

##  Use Cases

- **Robotics Swarming**: Simulate mesh comms in multi-robot systems.
- **Protocol Development**: Prototype and benchmark MANET protocols in controlled environments.
- **Field Deployments**: Run OSPF-MDR on Linux-based embedded systems for mobile ad hoc routing.
- **Education & Research**: Learn routing principles and evaluate routing behaviors under mobility.

---

##  Strengths

- **Open and Extensible**: Modify or add protocols freely.
- **Versatile Simulation Tools**: Well-supported frameworks with graphical interfaces (OMNeT++).
- **Real-world Applicability**: OSPF-MDR brings MANET routing capabilities to production Linux routers.
- **Custom Routing Support**: Developers can prototype entirely new routing logic.

---

##  Weaknesses

- **Simulation Overheads**: OMNeT++ and INET can require steep setup and maintenance.
- **Maturity Variance**: Some extensions (like INETMANET) may be outdated or partially maintained
- **Complex Builds**: Configuring real-time OSPF-MDR may require manual compilation and dependency handling.
- **Documentation Gaps**: Some projects lack comprehensive user guides.

---

##  Related Concepts

- [[MANET Routing Protocols]]
- [[Swarm Robotics]]
- [[Mesh Networking]]
- [[INET Framework]]
- [[OMNeT++]]
- [[OSPF]]
- [[Ad Hoc Networks]]

---

##  Compatible Items

- **Simulation Platforms**: OMNeT++ IDE, INET, INETMANET modules.
- **Real Deployments**: Quagga/FRR with OSPF-MDR on Linux or BSD systems.
- **Custom Development**: C++ modules integrated via NED files and OMNeT++ configuration.
- **Testing Tools**: Use cases integrated with robotics platforms (e.g., ROS) or network testing utilities.

---

##  External Resources

- INETMANET project (OMNeT++ extensions for MANET)
- INET’s MANET routing showcase documentation (DSDV, GPSR, etc.)
- OSPF-MDR implementation for MANET in Quagga (development by NRL)

---

##  Developer Tools

- **OMNeT++ + INET/INETMANET**: IDE for modeling and simulating network scenarios.
- **Custom Routing Modules**: Define behavior in C++ and configure via NED/omnetpp.ini
- **Quagga/FRR**: OSPF-MDR plugin for real-world MANET routing setups.
- **Mobility Models**: Random waypoint, group, or trace-based mobility in OMNeT++.

---

##  Key Highlights

- "OpenMANET" reflects the ecosystem of open-source MANET simulation and deployment tools.
- OMNeT++ (with INET and INETMANET) offers rich platforms for experimentation.
- OSPF-MDR is a standout for bringing MANET routing to Linux-based routers.
- Customism flexibility allows on-demand innovation in MANET protocols.

