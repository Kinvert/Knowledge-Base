---
title: Server-Sent Events (SSE)
tags: [protocols, transport, real-time, http]
---

# Server-Sent Events (SSE)

**Server-Sent Events (SSE)** is a communication protocol that allows a server to push real-time updates to the browser over a single HTTP connection. Unlike WebSockets, SSE is **unidirectional**, meaning messages flow from server to client only.

---

## 🔍 Overview

- Built on **HTTP** (specifically `text/event-stream` content type)
- Maintains a **persistent connection** to push updates to the client
- Often used for real-time notifications, logs, or feeds
- Supported natively in most modern browsers via the `EventSource` API

---

## ✅ Key Features

- Uses standard HTTP — easy to integrate with existing infrastructure
- Efficient for **one-way** data updates (e.g., stock tickers, chat updates)
- Auto-reconnects and can include **last event ID** for resuming after disconnect
- **Text-based** and very lightweight compared to full-duplex options

---

## ⚖️ Compared To

| Protocol      | Direction      | Built On  | Complexity | Ideal Use |
|---------------|----------------|-----------|------------|-----------|
| SSE           | Server → Client | HTTP      | Simple     | Notifications, feeds |
| WebSockets    | Bi-directional | TCP       | Medium     | Chat, gaming, live apps |
| Long Polling  | Server → Client | HTTP      | Higher     | Legacy support, fallback |

---

## 🧠 Use Cases

- Real-time news or social media feeds
- Monitoring dashboards and logs
- Collaborative document editing (updates from other users)
- Lightweight IoT or telemetry data streaming

---

## 🔗 See Also

- [[WebSockets]]
- [[HTTP]]
- [[REST]] (for contrast)
