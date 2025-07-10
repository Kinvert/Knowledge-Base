# ROS2 Web Bridge

**ROS2 Web Bridge** is a bridge that enables communication between a web browser and a ROS2 system by translating ROS2 messages and services into web-friendly formats such as JSON over WebSockets or REST. It allows browser-based UIs, dashboards, and remote robot control systems to interact with ROS2 nodes without needing native ROS2 support in the browser.

---

## 📚 Overview

The ROS2 Web Bridge exposes ROS2 topics, services, and parameters to the browser or other remote clients through WebSockets (via `rosbridge_suite`) or HTTP. This makes it ideal for creating interactive robot interfaces using JavaScript, React, Vue, or other web technologies.

The bridge is typically built on:
- `rosbridge_suite` for WebSocket interfaces
- `ros2-web-bridge` for native ROS2 support
- `roslibjs` or `rclnodejs` on the client side

---

## 🧠 Core Concepts

- **WebSocket Server**: Allows browser clients to connect and subscribe/publish to ROS2 topics
- **JSON Messages**: ROS2 messages are serialized into JSON for use over the web
- **roslibjs**: JavaScript client library to interact with ROS2 Web Bridge
- **Topics, Services, Parameters**: All exposed over WebSockets/HTTP
- **Cross-Origin Resource Sharing (CORS)**: May need configuration for remote UIs

---

## 🧰 Use Cases

- Web-based robot control interfaces (e.g. teleop UIs)
- Real-time visualization of sensor data in the browser
- Remote monitoring of robot state or diagnostic info
- Mobile-friendly robot dashboards (tablet/smartphone)
- Integrating robots with external web APIs or cloud tools

---

## ✅ Pros

- Enables platform-independent UIs
- No need for native ROS2 installations on clients
- Compatible with modern frontend frameworks
- Integrates well with lightweight microservices or serverless backends
- Can use standard web hosting for deployment

---

## ❌ Cons

- Limited performance for high-frequency or binary data (compared to native DDS or gRPC)
- Additional serialization/deserialization latency
- Exposes ROS2 interface over the network (security must be handled manually)
- Requires CORS and firewall configuration for remote clients

---

## 📊 Comparison Chart

| Feature               | ROS2 Web Bridge       | gRPC                      | Native ROS2 Node          | REST API (Flask/FastAPI) |
|-----------------------|-----------------------|----------------------------|----------------------------|---------------------------|
| Browser Compatible    | ✅ Yes                | ❌ No                      | ❌ No                      | ✅ Yes                    |
| Realtime Streams      | ✅ WebSocket           | ✅ Streaming               | ✅ Full DDS                | ❌ Polling                |
| Message Format        | JSON                  | Protobuf                   | ROS2 native                | JSON                     |
| Ease of Integration   | ✅ High (web UIs)      | ⚠️ Moderate                | ❌ Low (no browser support)| ✅ High                   |
| Security Features     | ❌ Manual              | ✅ TLS + Auth               | ✅ Secure DDS (with setup) | ⚠️ Basic                 |

---

## 🤖 In a Robotics Context

| Use Case                     | How Web Bridge Helps                           |
|------------------------------|-------------------------------------------------|
| Teleoperation                | Web UI sends twist commands to ROS2 topic      |
| Sensor Visualization         | Display camera, lidar, or state data in UI     |
| Remote Diagnostics           | Monitor robot health from browser              |
| Parameter Tuning             | Read/write ROS2 params remotely                |
| Cloud Integration            | Interface robot with serverless web APIs       |

---

## 🔧 Compatible Items

- [[ROS2 Publishers]], [[ROS2 Subscribers]], [[ROS2 Services]], [[ROS2 Parameters]]
- [[REST API]] (Alternative communication strategy)
- [[Microservices Architecture]]
- [[Docker Container]] (Web bridge often containerized)

---

## 🔗 Related Concepts

- [[REST API]] (HTTP alternative to WebSocket interface)
- [[Microservices Architecture]] (Modular web-connected ROS2 systems)
- [[WebSockets]] (Underlying tech used in bridge)
- [[Docker Container]] (Typical deployment format)
- [[roslibjs]] (JavaScript library for client interaction)

---

## 🛠 Developer Tools & Commands

- `ros2 launch rosbridge_server rosbridge_websocket_launch.xml` – Start WebSocket server  
- `npm install roslib` – Install client library for JavaScript  
- `docker pull osrf/rosbridge_server` – Get Docker image  
- `ros2 topic list` – See available topics  
- `ros2 service list` – See available services  

---

## 📚 Further Reading

- [rosbridge_suite on GitHub](https://github.com/RobotWebTools/rosbridge_suite)
- [ros2-web-bridge (experimental)](https://github.com/RobotWebTools/ros2-web-bridge)
- [roslibjs Docs](https://robotwebtools.org/js-docs/roslibjs/current/)
- [ROS2 Web Integration Example](https://github.com/RobotWebTools/ros2-web-bridge-examples)
- [Secure ROS Web Interfaces](https://robotwebtools.org/articles/2021/secure_ros_web/)

---
