# ACLs (Access Control Lists)

**Access Control Lists (ACLs)** are a security mechanism used to define which users, groups, or systems are permitted to access specific resources, and what operations they can perform. ACLs are widely applied in operating systems, networking equipment, file systems, and robotics middleware to enforce access policies and protect critical resources.

---

## ⚙️ Overview

ACLs provide a fine-grained method of **authorization**, complementing or extending broader models like **Role-Based Access Control (RBAC)**. Each resource has an associated list of rules that specify which identities (users, groups, or processes) have permission to perform actions such as *read, write, execute, or modify*.  

In **robotics and drone swarms**, ACLs are particularly useful to:
- Restrict which drones can send commands to others  
- Protect sensitive sensor data in multi-tenant systems  
- Enforce safety policies in distributed networks  

---

## 🧠 Core Concepts

- **Subjects**: Users, groups, services, or devices attempting access  
- **Objects**: Resources (files, network ports, APIs, topics, etc.) being protected  
- **Permissions**: Allowed actions (read, write, execute, delete, publish, subscribe)  
- **Rules/Entries**: Explicit access specifications tied to a subject-object pair  
- **Default Policies**: "Deny by default" vs. "Permit by default" approaches  

---

## 📊 Comparison Chart

| Feature / Approach           | ACLs | RBAC (Role-Based Access Control) | MAC (Mandatory Access Control) | DAC (Discretionary Access Control) | Capabilities |
|-------------------------------|------|----------------------------------|--------------------------------|-------------------------------------|--------------|
| Granularity                   | High | Medium                           | High                           | Medium                              | Very High    |
| Ease of Administration        | Moderate | High                         | Low                            | Medium                              | Low          |
| Flexibility                   | High | Medium                          | Low                            | High                                | High         |
| Common in Robotics?           | Yes  | Growing use in enterprise-style  | Limited                        | Sometimes                           | Rare         |
| Drone Swarm Use Case          | Node/topic access control | Operator roles | Safety-critical restrictions | Per-drone control policies | Fine-grained capability delegation |

---

## 🔧 Use Cases

- **Drone Swarms**  
  - Ensuring only authorized drones can publish control commands  
  - Restricting access to sensitive telemetry topics in [[ROS 2]]  
  - Securing multi-operator farm environments where different crews control subsets of drones  

- **General Robotics**  
  - Access control on camera feeds or LIDAR data streams  
  - Regulating API access to robot services  
  - Managing shared lab environments with multiple researchers  

- **Networking and Systems**  
  - Firewalls and routers implementing ACLs for traffic filtering  
  - File systems restricting read/write access  
  - Cloud services applying ACLs for object storage security  

---

## ✅ Strengths

- Very fine-grained access control  
- Flexible across file systems, networks, and middleware  
- Strong auditability (easy to log policy decisions)  
- Widely supported and standardized  

---

## ❌ Weaknesses

- Can be complex to manage at scale  
- Risk of misconfiguration leading to security gaps  
- Performance overhead with large ACL sets  
- Less abstracted than RBAC (role-based models scale more easily)  

---

## 🔗 Related Concepts

- [[RBAC]] (Role-Based Access Control)  
- [[ROS 2 Security]]  
- [[TLS]] (Transport Layer Security)  
- [[Encryption]]  
- [[Zero Trust Architecture]]  

---

## 🛠️ Compatible Items

- **Linux File Systems**: ext4, NTFS, ZFS support ACLs  
- **Networking Equipment**: Cisco, Juniper, MikroTik routers and switches  
- **Robotics Middleware**: [[DDS]] (Data Distribution Service) access control plugins, ROS 2 SROS2 policies  
- **Cloud Storage**: AWS S3, Google Cloud Storage, Azure Blob Storage ACLs  

---

## 📚 External Resources

- NIST Access Control Frameworks  
- ROS 2 SROS2 Security Guide: https://docs.ros.org/en/foxy/How-To-Guides/Using-SROS2.html  
- IETF RFC 4949 (Internet Security Glossary)  
- Cisco ACL configuration examples  

---

## 🏆 Summary

ACLs (Access Control Lists) are a **fundamental building block of system security**, providing fine-grained permissions for resources. In the context of **drone swarms on farms**, ACLs ensure that only trusted drones, operators, or processes can issue commands, access data, or interact with swarm coordination. They balance flexibility and control but require careful design to avoid complexity and misconfiguration.
