# REST API

A **REST API** (Representational State Transfer Application Programming Interface) is a standardized method for enabling communication between different software systems over HTTP. RESTful APIs are widely used in robotics, web applications, cloud services, and IoT for exposing sensor data, controlling actuators, or integrating external services.

---

## 📚 Overview

REST APIs use stateless HTTP methods (GET, POST, PUT, DELETE, etc.) to access and manipulate resources identified by URLs. Each resource is represented as a distinct URI, and clients interact with these resources using standard operations. REST emphasizes simplicity, scalability, and the use of standard web protocols.

---

## 🧠 Core Concepts

- **Resources**: Identified by URIs, such as `/robot/pose`
- **HTTP Verbs**:
  - `GET`: Retrieve data
  - `POST`: Create new resource
  - `PUT`: Update existing resource
  - `DELETE`: Remove resource
- **Statelessness**: Each request is independent with no stored session state
- **Representations**: Commonly JSON, sometimes XML or others
- **Headers**: Carry metadata (e.g., `Content-Type: application/json`)
- **Status Codes**: Indicate success/failure (e.g., 200 OK, 404 Not Found)

---

## 🧰 Use Cases

- Robot status dashboards (e.g., `GET /status`)
- Remote command/control (e.g., `POST /move`)
- Logging and telemetry endpoints (e.g., `POST /sensor-data`)
- Integration with external cloud services (e.g., mapping, analytics)
- Triggering simulations or parameter updates from UI or CI/CD pipelines

---

## ✅ Pros

- Widely supported and understood
- Easy to use from web clients, mobile, and CLI tools
- Human-readable (JSON), easy to debug
- Stateless nature improves scalability
- Language agnostic—usable from Python, JavaScript, C++, etc.

---

## ❌ Cons

- No built-in bi-directional communication (unlike WebSockets or gRPC)
- Less efficient for large or frequent binary data (e.g., image streams)
- Verb-based protocol is more rigid than pub-sub or RPC systems
- Statelessness can be limiting in session-based robotics scenarios

---

## 📊 Comparison Chart

| Feature              | REST API               | [[gRPC]]                     | [[WebSockets]]               | [[GraphQL]]                  |
|----------------------|------------------------|---------------------------|---------------------------|--------------------------|
| Protocol             | HTTP/1.1               | HTTP/2                   | TCP                       | HTTP                     |
| Data Format          | JSON (commonly)        | Protocol Buffers         | Any (JSON, binary, etc.)  | JSON                     |
| Bi-directional?      | ❌ No                  | ✅ Yes (streaming)        | ✅ Yes                    | ❌ No                   |
| Schema Enforced?     | ❌ Optional            | ✅ Strongly typed         | ❌ Optional               | ✅ Strongly typed        |
| Tooling Ecosystem    | ✅ Large                | ⚠️ Moderate                | ⚠️ Moderate               | ✅ Growing                |
| Best Use Case        | Simple CRUD + Web Apps | Low-latency structured APIs | Real-time sensor/control | Complex data queries     |

---

## 🤖 In a Robotics Context

| Use Case                        | Example Endpoint                  |
|----------------------------------|-----------------------------------|
| Robot Pose Access                | `GET /robot/pose`                 |
| Send Navigation Goal             | `POST /navigation/goal`          |
| Access Camera Calibration        | `GET /camera/params`             |
| Trigger SLAM Reset               | `POST /slam/reset`               |
| Upload Logs or Debug Info        | `POST /logs`                     |
| UI control of robot behavior     | `POST /control/mode`             |

---

## 🔧 Compatible Items

- [[Docker Container]] (Host REST API services)
- [[Heroku]] (Deploy Django/Flask REST APIs)
- [[Kubernetes Service]] (Expose APIs externally)
- [[Microservices Architecture]] (REST between services)
- [[ROS2 Web Bridge]] (Convert ROS2 messages to REST endpoints)
- [[CI-CD Pipelines]] (Trigger deployments or data collection via REST)

---

## 🔗 Related Concepts

- [[gRPC]] (Binary-efficient alternative to REST)
- [[Microservices Architecture]] (Common communication pattern)
- [[WebSockets]] (For real-time bidirectional data)
- [[Django]] (Includes Django REST Framework)
- [[Flask]] (Lightweight Python framework for REST APIs)

---

## 🛠 Developer Tools & Commands

- `curl -X GET http://localhost:8000/status` – Simple REST call
- `http GET /sensor-data` – Using HTTPie for human-friendly CLI
- `Postman` – GUI for testing REST APIs
- `FastAPI`, `Flask`, `Django REST Framework` – Python tools for building APIs
- `Express.js` – Node.js framework for REST endpoints

---

## 📚 Further Reading

- [REST API Design Best Practices](https://restfulapi.net/)
- [HTTP Status Codes Reference](https://developer.mozilla.org/en-US/docs/Web/HTTP/Status)
- [FastAPI Docs](https://fastapi.tiangolo.com/)
- [Django REST Framework](https://www.django-rest-framework.org/)
- [Postman Tool](https://www.postman.com/)

---
