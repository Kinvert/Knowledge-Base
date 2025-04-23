---
title: HTTP (Hypertext Transfer Protocol)
tags: [protocols, networking, web, application-layer, osi]
aliases: [Hypertext Transfer Protocol, HTTP/1.1, HTTP/2, HTTP/3]
---

# 🌐 HTTP (Hypertext Transfer Protocol)

## 🧭 Overview

**HTTP (Hypertext Transfer Protocol)** is an application-layer protocol in the OSI model used for transmitting hypermedia documents, such as HTML. It is the foundation of data communication on the World Wide Web and enables the transfer of resources between clients (e.g., browsers) and servers.

HTTP is **stateless**, meaning each request is independent, though mechanisms like cookies, sessions, and headers can simulate stateful behavior.

---

## 🛠️ How HTTP Works

1. **Client-Server Model**:
   - A client (e.g., browser) sends an HTTP request to a server.
   - The server processes the request and sends back an HTTP response.

2. **Request-Response Cycle**:
   - **Request**: Includes a method (e.g., `GET`, `POST`), a URL, headers, and optionally a body.
   - **Response**: Includes a status code (e.g., `200 OK`, `404 Not Found`), headers, and optionally a body.

3. **Transport Layer**:
   - HTTP typically runs over **TCP** (port 80) or **TLS/SSL** (port 443 for HTTPS).
   - HTTP/3 uses **[[QUIC]]**, a transport protocol built on UDP.

4. **Statelessness**:
   - Each request is independent, meaning the server does not retain information about previous requests.

---

## 🧩 Key Features

- **Methods**:
  - `GET`: Retrieve data.
  - `POST`: Submit data.
  - `PUT`: Update data.
  - `DELETE`: Remove data.
  - `HEAD`, `OPTIONS`, `PATCH`, etc.

- **Headers**:
  - Metadata about the request/response (e.g., `Content-Type`, `Authorization`).

- **Status Codes**:
  - Informational (`1xx`), Success (`2xx`), Redirection (`3xx`), Client Errors (`4xx`), Server Errors (`5xx`).

- **HTTPS**:
  - Secure version of HTTP using **TLS/SSL** for encryption.

---

## 📦 Common Use Cases

- **Web Browsing**: Loading websites and web applications.
- **APIs**: RESTful APIs use HTTP for communication.
- **File Transfer**: Downloading files or media.
- **IoT**: Lightweight HTTP-based communication for devices.

---

## ✅ Pros and ❌ Cons

### ✅ Advantages
- **Simplicity**: Easy to implement and widely supported.
- **Flexibility**: Supports a variety of data formats (e.g., JSON, XML, HTML).
- **Scalability**: Works well with caching and load balancing.
- **Security**: HTTPS provides encryption and authentication.

### ❌ Disadvantages
- **Statelessness**: Requires additional mechanisms (e.g., cookies) for stateful interactions.
- **Latency**: HTTP/1.1 can be slow due to sequential request-response cycles.
- **Overhead**: Headers can add significant overhead for small payloads.
- **Limited Real-Time Support**: Not ideal for real-time applications (e.g., chat, gaming).

---

## 🆚 Comparisons with Similar Protocols

| Protocol      | Direction      | Transport | Complexity | Best Use Cases                     |
|---------------|----------------|-----------|------------|------------------------------------|
| **HTTP**      | Request/Response | TCP       | Simple     | Websites, APIs, file transfer     |
| **HTTPS**     | Request/Response | TCP+TLS   | Moderate   | Secure websites, sensitive data   |
| **WebSockets**| Full-Duplex    | TCP       | Moderate   | Real-time apps (chat, gaming)     |
| **SSE**       | Server → Client | TCP       | Simple     | Real-time notifications, feeds    |
| **MQTT**      | Pub/Sub        | TCP       | Lightweight | IoT, telemetry, constrained devices |
| **gRPC**      | Unary/Stream   | HTTP/2    | Complex    | Microservices, high-performance APIs |

---

## ⚖️ HTTP Versions

| Version   | Key Features                              | Use Cases                          |
|-----------|-------------------------------------------|------------------------------------|
| **HTTP/1.0** | Basic request-response model, no persistent connections | Early web applications            |
| **HTTP/1.1** | Persistent connections, chunked transfer encoding | Most modern websites              |
| **HTTP/2**   | Multiplexing, header compression, binary framing | Faster, more efficient web apps   |
| **HTTP/3**   | Built on QUIC (UDP), reduced latency, improved reliability | Real-time apps, modern web apps   |

---

## 🛠️ How to Use HTTP

### 1. **Making Requests**
- Use tools like `curl`, Postman, or browser developer tools to send HTTP requests.
- Example `curl` command:
  ```bash
  curl -X GET https://example.com/api/resource
  ```
### 2. Building APIs
Use frameworks like Flask, Express, or Django to create HTTP-based APIs.
Example in Python (Flask):
```python
from flask import Flask, jsonify

app = Flask(__name__)

@app.route('/api/resource', methods=['GET'])
def get_resource():
    return jsonify({"message": "Hello, HTTP!"})

if __name__ == '__main__':
    app.run()
```
### 3. Debugging
Use tools like Wireshark or browser developer tools to inspect HTTP traffic.
🔗 Related Topics
- [[HTTPS]]
- [[WebSockets]]
- [[REST]]
- [[gRPC]]
- [[SSE]] (Server-Sent Events)
- [[MQTT]]
