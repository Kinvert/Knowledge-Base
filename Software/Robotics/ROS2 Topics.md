# 🟣 ROS 2 Topics

**ROS 2 Topics** provide the primary mechanism for nodes to communicate through a *publish-subscribe* model. Topics are named buses over which nodes exchange typed messages asynchronously.

---

## 🧠 Summary

- Topics enable **many-to-many** communication between nodes.
- Publishers send messages to a topic; subscribers receive messages from that topic.
- No direct knowledge between publishers and subscribers is required (decoupled architecture).
- Built on top of [[DDS]] (Data Distribution Service) for transport and discovery.

---

## ⚙️ Key Features

- **Asynchronous messaging:** Messages are sent without waiting for a response.
- **Flexible transport:** DDS manages delivery, reliability, and QoS (Quality of Service) settings.
- **Data typing:** Messages sent over a topic follow strict type definitions defined by `.msg` files.
- **Anonymous communication:** Publishers and subscribers need only agree on the topic name and type.
- **QoS policies:** Configure reliability, durability, history depth, and more.

---

## 🚀 Example Applications

- A node publishing sensor data (e.g., LIDAR, camera frames) on a topic like `/scan` or `/camera/image_raw`.
- A subscriber node listening for `/cmd_vel` to control robot velocity.
- Logging and monitoring of system status via diagnostic topics.

---

## 🏆 Strengths

- Loose coupling between components simplifies large, distributed systems.
- Scales well in complex, multi-node architectures.
- DDS QoS features provide robust control over delivery guarantees.

---

## ⚠️ Weaknesses

- No built-in request-response capability (use [[ROS2 Services]] or [[ROS2 Actions]] for that).
- Poor topic name or type management can lead to system confusion.
- DDS QoS policies add complexity for new users.

---

## 📊 Comparison with Other Communication Types in ROS 2

| Communication Type | Pattern              | When to Use                                   |
|--------------------|---------------------|-----------------------------------------------|
| Topic              | Publish-Subscribe    | Continuous, streaming data                   |
| [[ROS2 Services]]   | Request-Response     | One-time data exchange with reply             |
| [[ROS2 Actions]]    | Goal + Feedback/Result | Long-running tasks needing progress updates |

---

## 🔗 Related Notes

- [[ROS2]]
- [[ROS2 Node]]
- [[ROS2 Services]]
- [[ROS2 Actions]]
- [[DDS]] (Data Distribution Service)
- [[QoS]] (Quality of Service)

---

## 🌐 External References

- [ROS 2 Documentation - Topics](https://docs.ros.org/en/foxy/Concepts/Topics.html)
- [ROS 2 Tutorials - Writing Publishers and Subscribers](https://docs.ros.org/en/foxy/Tutorials/Topics/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

---
