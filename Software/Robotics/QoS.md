# 🟣 ROS 2 Quality of Service (QoS)

**Quality of Service (QoS)** in ROS 2 controls how data is transmitted between [[ROS2 Publishers]] and [[ROS2 Subscribers]]. Built on [[DDS]] (Data Distribution Service), QoS settings help ensure communication behaves as needed for different robotic applications.

QoS isn't exclusive to ROS2 this is just how ChatGPT wrote it and I'm low on requests for the day.

---

## 🧠 Summary

- QoS defines policies for message delivery guarantees, durability, history, and more.
- Allows tuning of communication to match application requirements (e.g., sensor streaming vs. control commands).
- Configured per publisher and subscriber, often requiring matching policies to connect.

---

## ⚙️ Key Policies

| QoS Policy            | Description                                           | Example Use |
|-----------------------|-------------------------------------------------------|-------------|
| **Reliability**        | Guarantee delivery (`Reliable`) or best effort.       | Critical control commands: Reliable |
| **Durability**         | Keep messages for late joiners (e.g., `TransientLocal`). | Config/static data: TransientLocal |
| **History**            | Store a certain number of past messages.              | Buffering sensor data |
| **Deadline**           | Expected message frequency, detect missed deadlines.  | Real-time systems |
| **Lifespan**           | Expire messages after a certain time.                 | Time-sensitive data |
| **Liveliness**         | Detect when a publisher stops sending.                | Safety-critical monitoring |

---

## 🚀 Example Use Cases

- **Reliable + TransientLocal:** Configuration data that must be received by all new subscribers.
- **BestEffort + Volatile:** High-frequency sensor data like LIDAR where losing some data is acceptable.
- **Deadline + Liveliness:** Systems needing guaranteed update rates, e.g., motor controllers.

---

## 📊 Comparison of Typical QoS Profiles

| Profile Name         | Reliability | Durability       | History         | Notes                                     |
|----------------------|-------------|-----------------|----------------|-------------------------------------------|
| Default               | Reliable    | Volatile         | Keep Last (1)   | Standard safe default                     |
| Sensor Data           | Best Effort | Volatile         | Keep Last (10)  | Efficient streaming, lossy if needed      |
| Parameters / Config   | Reliable    | Transient Local  | Keep Last (1)   | Ensure late subscribers get last value    |
| Real-time Control     | Reliable    | Volatile         | Keep Last (1)   | Low latency critical control commands     |

---

## 🏆 Strengths

- Highly flexible to match various robotics communication needs.
- Built on DDS, benefits from mature QoS system.
- Helps balance reliability vs. performance.

---

## ⚠️ Weaknesses

- Publisher/subscriber QoS mismatch prevents communication (can be subtle to debug).
- Complex combinations can be intimidating for beginners.
- Tuning for edge cases (e.g., lossy wireless) requires care.

---

## 🔗 Related Notes

- [[ROS2]]
- [[ROS2 Topics]]
- [[DDS]] (Data Distribution Service)
- [[ROS2 Node]]
- [[ROS2 Services]]
- [[ROS2 Actions]]

---

## 🌐 External References

- [ROS 2 QoS Policies](https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html)
- [DDS QoS Policies Overview](https://community.rti.com/static/documentation/connext-dds/5.3.1/doc/manuals/connext_dds/html_files/RTI_ConnextDDS_CoreLibraries_GettingStarted/Content/GetStarted/QoS_Policies_Intro.htm)

---
