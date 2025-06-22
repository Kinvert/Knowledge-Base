# 📡 DDS (Data Distribution Service)

**DDS (Data Distribution Service)** is a middleware protocol and API standard for data-centric publish-subscribe communication. It is widely used in real-time and embedded systems, including robotics, autonomous vehicles, aerospace, and industrial automation. DDS is the primary communication layer underlying [[ROS2]] (Robot Operating System 2).

---

## 🧠 Summary

- **Type**: Real-time data exchange middleware
- **Standardized by**: OMG (Object Management Group)
- **Key feature**: Data-centric publish-subscribe model
- **Typical use case**: Real-time distributed systems requiring scalable, low-latency, reliable communication

---

## 🎯 Main Features

- **Publish/subscribe model**: Decouples data producers and consumers
- **QoS (Quality of Service)**: Fine-grained control over data delivery (e.g. reliability, durability, latency, deadline)
- **Discovery**: Automatic discovery of publishers/subscribers on the network
- **Scalability**: Suitable for both small embedded systems and large distributed networks
- **Real-time support**: Designed for time-sensitive and safety-critical applications

---

## 🔬 Common Use Cases

- [[ROS2]] communication layer
- Autonomous vehicles (ADAS, driverless trucks)
- Industrial automation (Industry 4.0)
- Aerospace and defense systems
- Healthcare devices
- Distributed simulation systems

---

## 📊 Comparison Table

| Feature                   | DDS                          | [[MQTT]]                   | [[Websockets]]             |
|---------------------------|-----------------------------|----------------------------|---------------------------|
| Communication model        | Publish-subscribe            | Publish-subscribe           | Full-duplex stream         |
| Real-time support          | ✅ Strong                    | ⚠️ Limited                  | ❌ No native real-time     |
| QoS granularity            | ✅ Extensive (many policies)  | ⚠️ Basic                    | ❌ None                    |
| Automatic discovery        | ✅ Yes                        | ❌ No                        | ❌ No                      |
| Designed for embedded      | ✅ Yes                        | ✅ Yes                       | ⚠️ Somewhat (depends)      |

---

## ✅ Strengths

- Flexible QoS policies for various reliability and timing requirements
- No central broker needed → true distributed architecture
- Built-in discovery and configuration
- Widely supported by major vendors (RTI, eProsima, ADLINK, TwinOaks, etc.)
- Foundation of [[ROS2]] communication

---

## ❌ Weaknesses

- Can be complex to configure due to rich QoS options
- Larger footprint than lightweight protocols like [[MQTT]]
- Performance and features can vary across vendor implementations

---

## 🌐 External References

- [OMG DDS standard](https://www.omg.org/spec/DDS/)
- [eProsima Fast DDS](https://fast-dds.docs.eprosima.com/)
- [RTI Connext DDS](https://www.rti.com/products/dds)

---

## 🔗 Related Notes

- [[ROS2]]
- [[MQTT]]
- [[Websockets]]
- [[Publish-Subscribe Model]]
- [[RTI Connext]]
- [[eProsima Fast DDS]]

---
