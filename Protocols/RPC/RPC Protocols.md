---
title: RPC Protocols
tags: [protocols, rpc, networking, distributed-systems, microservices]
aliases: [Remote Procedure Call Protocols, RPC Frameworks]
---

# 🔗 RPC Protocols

## 🧭 Overview

**RPC (Remote Procedure Call)** protocols enable communication between distributed systems by allowing a program to execute a procedure on a remote server as if it were a local function. RPC abstracts the complexities of network communication, making it easier to build distributed applications and microservices.

RPC protocols are widely used in modern software architectures, including client-server systems, microservices, and cloud-based applications. They vary in terms of performance, serialization formats, and transport mechanisms, making them suitable for different use cases.

---

## 🛠️ Key Features of RPC Protocols

1. **Abstraction**:
   - Hide the complexities of network communication, allowing developers to focus on application logic.

2. **Serialization**:
   - Use serialization formats (e.g., JSON, Protobuf) to encode and decode data for transmission.

3. **Transport Independence**:
   - Support various transport protocols (e.g., HTTP, TCP, UDP).

4. **Cross-Language Support**:
   - Many RPC frameworks support multiple programming languages, enabling interoperability.

5. **Performance Optimization**:
   - Protocols like gRPC and Thrift are optimized for low-latency, high-throughput communication.

---

## 📦 Common RPC Protocols

### [[gRPC]]
- **Purpose**: High-performance, open-source RPC framework developed by Google.
- **Key Features**:
  - Uses HTTP/2 for transport, enabling multiplexing and low latency.
  - Employs Protobuf (Protocol Buffers) for serialization.
  - Supports bidirectional streaming.
- **Use Cases**:
  - Microservices communication.
  - Real-time applications (e.g., chat, video streaming).
  - Cloud-native systems.

---

### [[JSON-RPC]]
- **Purpose**: Lightweight RPC protocol using JSON for serialization.
- **Key Features**:
  - Simple and human-readable.
  - Transport-agnostic (commonly used with HTTP or WebSockets).
  - Stateless communication.
- **Use Cases**:
  - Web applications.
  - Lightweight APIs.
  - Systems requiring human-readable data formats.

---

### [[XML-RPC]]
- **Purpose**: RPC protocol using XML for serialization.
- **Key Features**:
  - Simple and widely supported.
  - Transport-agnostic (commonly used with HTTP).
  - Human-readable but verbose compared to JSON.
- **Use Cases**:
  - Legacy systems.
  - Applications requiring XML-based communication.

---

### [[Thrift]]
- **Purpose**: Cross-language RPC framework developed by Facebook.
- **Key Features**:
  - Supports multiple serialization formats (e.g., binary, JSON).
  - Highly efficient and compact.
  - Supports multiple transport protocols (e.g., HTTP, TCP).
- **Use Cases**:
  - High-performance microservices.
  - Cross-language communication in distributed systems.

---

### [[SOAP]] (Simple Object Access Protocol)
- **Purpose**: XML-based protocol for structured messaging.
- **Key Features**:
  - Built-in error handling and security features.
  - Transport-agnostic (commonly used with HTTP and SMTP).
  - Verbose and heavyweight compared to modern alternatives.
- **Use Cases**:
  - Enterprise systems.
  - Applications requiring strict standards and security.

---

### [[CORBA]] (Common Object Request Broker Architecture)
- **Purpose**: Legacy RPC protocol for distributed object systems.
- **Key Features**:
  - Language-agnostic and platform-independent.
  - Supports complex data types and object references.
  - Considered heavyweight and outdated.
- **Use Cases**:
  - Legacy enterprise systems.
  - Applications requiring distributed object management.

---

### [[kRPC]]
- **Purpose**: Lightweight RPC framework for Kerbal Space Program and other applications.
- **Key Features**:
  - Simple and efficient.
  - Designed for real-time communication.
  - Supports multiple programming languages.
- **Use Cases**:
  - Game development.
  - Real-time telemetry and control.

---

## ✅ Pros and ❌ Cons of RPC Protocols

### ✅ Advantages
- **Simplified Communication**: Abstracts network communication, making distributed systems easier to develop.
- **Cross-Language Support**: Many RPC frameworks support multiple programming languages.
- **Performance**: Protocols like gRPC and Thrift are optimized for high performance.
- **Scalability**: Suitable for microservices and large-scale distributed systems.

### ❌ Disadvantages
- **Complexity**: Adds overhead compared to direct network communication.
- **Tight Coupling**: RPC can tightly couple clients and servers, making changes harder to manage.
- **Serialization Overhead**: Protocols like XML-RPC and SOAP can introduce significant overhead due to verbose serialization formats.

---

## 🆚 Comparisons of RPC Protocols

| **Protocol**   | **Serialization** | **Transport**       | **Performance** | **Use Cases**                     | **Strengths**                     | **Weaknesses**                   |
|-----------------|-------------------|---------------------|-----------------|------------------------------------|------------------------------------|-----------------------------------|
| **gRPC**       | Protobuf          | HTTP/2              | High            | Microservices, real-time apps      | Low latency, streaming support     | Requires Protobuf knowledge      |
| **JSON-RPC**   | JSON              | HTTP, WebSockets    | Moderate        | Web apps, lightweight APIs         | Simple, human-readable             | Stateless, less efficient         |
| **XML-RPC**    | XML               | HTTP                | Low             | Legacy systems                     | Widely supported                   | Verbose, slow                    |
| **Thrift**     | Binary, JSON      | HTTP, TCP           | High            | Cross-language microservices       | Compact, efficient                 | Steeper learning curve            |
| **SOAP**       | XML               | HTTP, SMTP          | Low             | Enterprise systems                 | Built-in security, error handling  | Verbose, heavyweight             |
| **CORBA**      | Binary            | Custom              | Moderate        | Legacy enterprise systems          | Language-agnostic, object support  | Outdated, complex                |
| **kRPC**       | Binary, JSON      | TCP, WebSockets     | High            | Game development, telemetry        | Lightweight, real-time             | Limited to specific use cases     |

---

## 🔗 Related Topics

- [[Protocols]]
- [[Networking Basics]]
- [[Microservices]]
- [[Serialization Formats]]
- [[Capn Proto]]

---

## 📚 Further Reading

- [gRPC Documentation](https://grpc.io/docs/)
- [JSON-RPC Specification](https://www.jsonrpc.org/specification)
- [Thrift Documentation](https://thrift.apache.org/)
- [SOAP Overview](https://www.w3.org/TR/soap/)
- [CORBA Specification](https://www.omg.org/spec/CORBA/)
- [kRPC Documentation](https://krpc.github.io/krpc/)

---

## 🧠 Summary

RPC protocols are essential for building distributed systems, enabling seamless communication between applications across networks. From high-performance frameworks like gRPC and Thrift to lightweight options like JSON-RPC, each protocol has its strengths and weaknesses. Choosing the right RPC protocol depends on factors like performance requirements, serialization preferences, and use cases.
