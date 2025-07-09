# ROS2 Services

ROS2 Services are a communication mechanism in the Robot Operating System (ROS2) that enable synchronous, request-response interactions between nodes. Unlike topics, which are asynchronous and one-way, services are used when a node needs to ask another node to perform an action and wait for the result.

---

## 📚 Overview

A ROS2 Service defines a pair of messages: a **request** and a **response**. One node advertises a service (server), and another node calls it (client). This is ideal for tasks like parameter setting, device control, or querying state, where immediate feedback is expected.

---

## 🧠 Core Concepts

- **Service Definition**: `.srv` file format with two sections: request and response.
- **Client Node**: Sends request and waits for response.
- **Server Node**: Implements logic to handle requests and send back results.
- **Blocking Call**: The client blocks until it receives a response or times out.
- **IDL**: ROS2 services are defined using Interface Definition Language (IDL).

---

## 🧰 Use Cases

- Controlling hardware devices (e.g., open gripper, reset sensor)
- Querying or setting parameters
- Requesting one-time computations or data (e.g., transform lookup)
- Managing state transitions in state machines
- Initiating or stopping processes (e.g., start mapping)

---

## ✅ Pros

- Simple to implement and understand
- Ensures responses for requests (synchronous behavior)
- Great for command-response patterns

---

## ❌ Cons

- Blocking by default (not suitable for long operations)
- Poor scalability compared to topics or actions for frequent events
- Fragile in unreliable or distributed networks

---

## 📊 Comparison Chart

| Mechanism  | Communication Type | Pattern          | Good For                     | Notes                          |
|------------|---------------------|------------------|-------------------------------|--------------------------------|
| **[[ROS2 Topics]]** | Asynchronous        | Publish-Subscribe| Streaming sensor data         | One-to-many, fire-and-forget   |
| **[[ROS2 Services]]** | Synchronous       | Request-Response | Configs, short-lived commands | One-to-one, blocking           |
| **[[ROS2 Actions]]** | Asynchronous        | Goal-Feedback-Result | Long-running tasks        | Cancelable, progress feedback  |
| **[[ROS2 Parameters]]** | Queryable        | Get/Set Config   | Configuration values          | Persistent key-value store     |

---

## 🔧 Compatible Items

- `rclcpp::Client` / `rclcpp::Service` (C++)
- `rclpy.client.Client` / `rclpy.service.Service` (Python)
- `ros2 interface show <pkg/srv/ServiceName>`
- `ros2 service call /service_name <type> "{data}"`
- [[ROS2 Topics]] (Alternative for streaming data)
- [[ROS2 Actions]] (Better for long-running tasks)

---

## 🔗 Related Concepts

- [[ROS2 Topics]] (Asynchronous communication)
- [[ROS2 Actions]] (Long-running asynchronous tasks)
- [[ROS2 Parameters]] (Persistent configuration values)
- [[ROS2 Node]] (Run clients/servers)
- [[ROS2 Interface Definition]] (Defines custom services)

---

## 🛠 Developer Tools

- `ros2 interface show`, `ros2 service list`, `ros2 service call`
- rqt service plugins for GUI-based testing
- `colcon` and `CMakeLists.txt` for generating service code
- `srv` files defined under `srv/` directory in a ROS2 package

---

## 📚 Further Reading

- [ROS2 Documentation: Services](https://docs.ros.org/en/foxy/Concepts/Services.html)
- ROS2 Tutorials on services (C++ and Python)
- Real-time ROS2 best practices (for latency-sensitive services)

---
