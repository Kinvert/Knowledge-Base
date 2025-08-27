# RTI Connext DDS (RTI DDS)

RTI Connext DDS (from Real-Time Innovations) is a high-performance implementation of the **Data Distribution Service (DDS)** standard. It’s widely used in robotics (ROS 2 middleware), defense, autonomous systems, and industrial automation. DDS provides a decentralized pub/sub communication model with strong QoS (Quality of Service) guarantees.

---

## Core Concepts
- **Domain** → A virtual communication space; participants must share the same domain ID.
- **Participant** → A process that joins a domain and communicates.
- **Topic** → A named data channel (data type + name).
- **Publisher/Subscriber** → Entities that send/receive data.
- **DataWriter/DataReader** → Send/receive typed data for a topic.
- **QoS (Quality of Service)** → Fine-grained controls (reliability, durability, deadlines, history, etc.).

---

## Common Tools
- **rtiddsgen** → Generates source code from IDL (Interface Definition Language).
- **rtiddsping / rtiddsspy** → Command-line tools for testing DDS communications.
- **rticonnextdds-config** → Config utility.
- **rtiroutingservice** → Bridges different DDS domains or external systems.
- **Admin Console** → GUI for monitoring DDS networks.

---

## Useful One-Liners Cheatsheet

`rtiddsgen -language C++11 -example x64Linux2.6gcc5.4.0 MyType.idl`  
Generate DDS type code for C++11 from IDL.

`rtiddsping -domain 0 -pub`  
Publish test data in DDS domain 0.

`rtiddsping -domain 0 -sub`  
Subscribe to test data in DDS domain 0.

`rtiddsspy -domain 0`  
Spy on all traffic in DDS domain 0.

`rticonnextdds-config -version`  
Check installed RTI DDS version.

`rticonnextdds-config -help`  
Show available configuration options.

`rtiroutingservice -cfgFile routing_config.xml`  
Run DDS Routing Service with the given config.

`rtirecord -domainId 0 -file out.db`  
Record DDS traffic from domain 0 to a file.

`rtireplay -domainId 0 -file out.db`  
Replay recorded DDS traffic into a domain.

`rtiddsprototyper -cfgFile prototype.xml`  
Quickly test and prototype DDS apps without writing code.

---

## DDS QoS Examples (commonly adjusted policies)
- **Reliability** → BEST_EFFORT vs RELIABLE.
- **Durability** → VOLATILE, TRANSIENT_LOCAL, TRANSIENT, PERSISTENT.
- **History** → KEEP_LAST (n samples) vs KEEP_ALL.
- **Deadline** → Maximum time between samples.
- **Liveliness** → Ensures publishers are alive and detectable.

---

## Comparisons

| Feature                  | RTI DDS (Connext)               | ROS 2 (rmw_fastrtps)   | [[ZeroMQ]]   | [[Kafka]]    | [[MQTT]]     |
|---------------------------|---------------------------------|------------------------|--------------|--------------|--------------|
| Standard                  | OMG DDS                        | Uses DDS (Fast-DDS)    | No           | No           | No           |
| Latency                  | µs range (very low)            | Low (via Fast-DDS)     | Low          | Higher       | Higher       |
| [[QoS]]                  | Extensive (Reliability, Durability, History, Deadline, Liveliness, Ownership, Partitions, etc.) | Limited (subset of DDS) | Minimal      | Some (acks, retention) | Minimal (QoS 0/1/2) |
| Discovery                | Built-in peer-to-peer           | DDS-based              | Manual       | Broker-based | Broker-based |
| Persistence              | Yes (via QoS + Persistence Service) | Partial               | External     | Yes          | Limited      |
| Scalability              | Very high (decentralized)       | Medium (DDS under ROS) | Medium       | Very high    | High         |
| Ecosystem                | Industrial, defense, robotics   | Robotics (ROS 2)       | General IPC  | Data pipelines | IoT/M2M      |

---

## References
- RTI Connext DDS official: https://www.rti.com/products/connext-dds
- ROS 2 and DDS relationship: https://docs.ros.org/en/foxy/Concepts/About-Different-Middleware-Vendors.html
- Tutorials: https://community.rti.com/tutorials
- Admin Console: https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/manuals/connext_dds_professional/tools/admin_console/index.htm
