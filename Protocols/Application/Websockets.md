---
title: WebSockets
aliases: [WebSocket, WS]
tags: [protocol, networking, real-time, [[Protocols/Transport]]]
---

# 🌐 WebSockets

**WebSockets** is a communication protocol that provides full-duplex, bidirectional communication channels over a single TCP connection. It's commonly used for applications that require real-time communication, such as chat apps, live data feeds, collaborative tools, or games.

## Key Characteristics

- Built on top of TCP
- Enables persistent, low-latency connections
- Initiated via HTTP(S) handshake and then upgraded to WebSocket
- Supports full-duplex communication
- Reduces the overhead of repeatedly opening new connections

## 📦 Common Use Cases

- 📲 **Real-time chat applications**
- 📈 **Financial dashboards and tickers**
- 🎮 **Online multiplayer games**
- 📡 **Telemetry feeds in IoT or automotive**
- 🧠 **Driverless vehicle UIs (debugging and live sensor views)**
- 💬 **Customer support/live agents**
- 🗂️ **Collaborative editing (e.g. Google Docs-like apps)**
- 📊 **Live log viewers or dev dashboards**
- **Remote control and monitoring systems**
- **IoT data streaming**

## How It Works

1. **Client initiates a handshake** using HTTP(S)
2. **Server responds with a special header** to switch to the WebSocket protocol
3. Once the connection is established, both parties can **send messages at any time**
4. The connection persists until **explicitly closed** by either party

## ⚙️ Protocols and Standards

- **WebSocket Protocol**: [RFC 6455](https://datatracker.ietf.org/doc/html/rfc6455)
- **Secure WebSockets**: Uses **WSS** over **TLS 1.2 or 1.3**
- **TCP/IP Stack**: WebSockets are built on standard **TCP (port 80/443)**
- **IEEE 802.3**: Ethernet Layer (for LAN connectivity)
- **IEEE 802.11**: Wi-Fi Layer (for wireless connections)
- **TLS/SSL**: [RFC 5246] for secure sessions

## 🖥️ Hardware Requirements

WebSockets don't require specialized hardware, but **system capabilities influence performance**:

| Component           | Recommendation                                       |
|--------------------|------------------------------------------------------|
| **NIC (Network Interface Card)** | Gigabit Ethernet or Wi-Fi 6 preferred for high concurrency |
| **CPU**             | Multicore CPUs help with scaling via threading      |
| **Memory**          | Sufficient RAM for concurrent session buffers       |
| **Firewall/Router** | Must allow WebSocket upgrades and persistent TCP    |
| **SSL Accelerator** | Useful for offloading TLS for secure WebSockets     |

**Embedded use**: Some microcontrollers and embedded Linux boards (e.g., Raspberry Pi, Jetson Nano) can handle WebSocket servers for IoT.

## 🆚 WebSocket vs Other Protocols

| Feature            | WebSocket       | HTTP/2          | MQTT             | gRPC (with streaming) |
|--------------------|------------------|------------------|------------------|------------------------|
| Connection Type     | Full-duplex       | Request/response | Pub/sub (usually) | Unary or stream-based |
| Real-Time Capable   | Yes               | Limited          | Yes               | Yes                    |
| Built Over TCP      | Yes               | Yes              | Yes               | Yes                    |
| Built Over HTTP     | Starts via HTTP   | Yes              | No                | Yes                    |
| Lightweight         | Moderate          | Heavy            | Very light        | Moderate               |

| Feature              | WebSocket         | HTTP/2 Push      | SSE (Server-Sent Events) | MQTT                | gRPC (Streaming)         |
|----------------------|------------------|------------------|---------------------------|---------------------|---------------------------|
| **Direction**        | Bi-directional   | Server → Client  | Server → Client           | Bi-directional      | Bi-directional             |
| **Transport**        | TCP              | TCP              | TCP                        | TCP                 | HTTP/2 (TCP)              |
| **Overhead**         | Low              | Medium           | Low                        | Very Low            | Medium                    |
| **Binary Support**   | ✅ Yes           | ❌ No            | ❌ No                      | ✅ Yes              | ✅ Yes                    |
| **Browser Native**   | ✅ Yes           | ✅ Yes           | ✅ Yes                     | ❌ No               | ❌ No                     |
| **Connection Type**  | Persistent       | Persistent       | Persistent                 | Persistent          | Persistent                |
| **Reconnection Logic**| Manual           | Handled by HTTP2 | Built-in                   | Built-in            | Handled by gRPC           |
| **Best For**         | Real-time apps   | Push notifications| Dashboards, monitoring     | IoT/Telemetry       | Microservice comms        |

## ✅ Advantages

- 🌀 **Full-duplex Communication**: Send/receive without re-establishing connections.
- ⚡ **Low Latency**: Ideal for real-time scenarios.
- 🧩 **Lightweight Headers**: Minimal overhead compared to HTTP.
- 🔐 **WSS Support**: Secure communication via TLS.
- 🧠 **Ideal for UIs and event-driven architecture**

## ❌ Limitations

- May require additional setup for load balancing and scaling
- Not all environments or firewalls handle WebSocket connections gracefully
- May fall back to HTTP-based alternatives if unsupported

## Examples

```python
import asyncio
import websockets

async def hello():
    uri = "ws://localhost:8765"
    async with websockets.connect(uri) as websocket:
        await websocket.send("Hello, WebSocket!")
        response = await websocket.recv()
        print(f"Received: {response}")

asyncio.run(hello())
```

```javascript
// JavaScript WebSocket Client in Browser

const socket = new WebSocket("ws://localhost:8765");

socket.onopen = () => {
  console.log("Connected to server");
  socket.send("Hello from browser!");
};

socket.onmessage = (event) => {
  console.log("Received:", event.data);
};

socket.onclose = () => {
  console.log("Connection closed");
};

```

```go
// Go WebSocket Echo Server
// Requires: go get github.com/gorilla/websocket

package main

import (
	"fmt"
	"net/http"
	"github.com/gorilla/websocket"
)

var upgrader = websocket.Upgrader{}

func handler(w http.ResponseWriter, r *http.Request) {
	conn, _ := upgrader.Upgrade(w, r, nil)
	defer conn.Close()

	for {
		mt, msg, err := conn.ReadMessage()
		if err != nil {
			break
		}
		fmt.Printf("Received: %s\n", msg)
		conn.WriteMessage(mt, msg) // Echo back
	}
}

func main() {
	http.HandleFunc("/", handler)
	fmt.Println("WebSocket server listening on :8765")
	http.ListenAndServe(":8765", nil)
}

```

```javascript
// Node.js WebSocket Echo Server
// Requires: npm install ws

const WebSocket = require('ws');
const server = new WebSocket.Server({ port: 8765 });

server.on('connection', socket => {
  socket.on('message', message => {
    console.log('Received:', message);
    socket.send(message); // Echo back
  });
});

console.log("WebSocket server running on ws://localhost:8765");

```
## Related Topics

- [[MQTT]]
- [[HTTP]]
- [[gRPC]]
- [[ZeroMQ]]
- [[WebRTC]]
- [[SSE]]
- [[TPC]]
- [[Networking]]
- [[Protocols/Transport]]
- [[Protocols/Application]]

## See Also

- [MDN: WebSockets Overview](https://developer.mozilla.org/en-US/docs/Web/API/WebSockets_API)
- [RFC 6455 – The WebSocket Protocol](https://datatracker.ietf.org/doc/html/rfc6455)
