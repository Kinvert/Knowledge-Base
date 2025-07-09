# ROS2 Subscribers

ROS2 Subscribers are part of the core publish-subscribe mechanism in the Robot Operating System 2 (ROS2). A subscriber listens to a specific topic and receives messages of a designated type, enabling real-time communication between different components in a robotic system.

---

## 📚 Overview

In ROS2, subscribers are associated with a topic name and a message type. When a corresponding publisher sends data on that topic, all active subscribers receive it. This decoupled model allows for flexible and scalable system architectures, with one-to-many and many-to-one communication patterns.

---

## 🧠 Core Concepts

- **Topic**: Named channel that subscribers listen to for specific data streams.
- **Message Type**: Data structure defined by ROS interfaces (e.g. `sensor_msgs/msg/LaserScan`).
- **Callback Function**: A user-defined function executed each time a new message arrives.
- **QoS (Quality of Service)**: Determines message delivery guarantees (e.g., reliability, history).
- **DDS Backend**: ROS2 relies on DDS (Data Distribution Service) for the transport layer.

---

## 🧰 Use Cases

- Receiving sensor data (camera, IMU, LiDAR, etc.)
- Listening to control commands or state updates
- Subscribing to processed results from other nodes
- Monitoring diagnostics or system events
- Reacting to user input or GUI events

---

## ✅ Pros

- Asynchronous and event-driven
- Lightweight and efficient for high-frequency data
- Decouples producers and consumers
- Easily scales with multiple publishers or subscribers

---

## ❌ Cons

- No built-in request-response or result reporting
- Data may be lost if QoS is poorly configured
- Not ideal for long-running tasks or task feedback

---

## 📊 Comparison Chart

| Mechanism   | Communication Type | Pattern         | Best For                    | Notes                               |
|-------------|--------------------|------------------|------------------------------|-------------------------------------|
| **Topics**  | Asynchronous        | Publish-Subscribe| Streaming data               | Subscribers receive each message    |
| **Services**| Synchronous         | Request-Response | Config changes, commands     | No subscriber; use client/server    |
| **Actions** | Asynchronous        | Goal-Result-Feedback | Long-running operations | Includes feedback and cancelability |

---

## 🔧 Compatible Items

- `rclcpp::Subscription` / `rclpy.subscription.Subscription`
- `ros2 topic echo`, `ros2 topic list`, `ros2 topic info`
- [[ROS2 Publishers]] (Send messages to topics)
- [[ROS2 Messages]] (Standard and custom message types)
- [[ROS2 QoS Policies]] (Configure reliability, depth, etc.)
- [[DDS]] (Underlying pub/sub transport)

---

## 🔗 Related Concepts

- [[ROS2 Publishers]] (Counterpart that sends data)
- [[ROS2 Topics]] (Named communication channels)
- [[ROS2 QoS Policies]] (Affect message reliability and delivery)
- [[ROS2 Node]] (Subscribers are created within nodes)
- [[Sensor Integration]] (Often requires subscribing to sensor data topics)

---

## 🛠 Developer Tools

- `ros2 topic echo /topic_name` to view messages
- `ros2 topic hz /topic_name` to check frequency
- `ros2 topic info /topic_name` to inspect topic details
- `rqt_graph` to visualize publisher-subscriber relationships
- Subscription callback functions in C++ or Python

---

## 📚 Further Reading

- [ROS2 Topics Overview](https://docs.ros.org/en/foxy/Tutorials/Topics/Understanding-ROS2-Topics.html)
- [ROS2 Subscriber API - C++ and Python](https://docs.ros.org/en/foxy/How-To-Guides/Using-ROS2-Topics.html)
- DDS specification for underlying transport guarantees

---
