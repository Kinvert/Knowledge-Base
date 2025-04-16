---
title: WebSockets
aliases: [WebSocket, WS]
tags: [protocol, networking, real-time, [[Protocols/Transport]]]
---

# WebSockets

**WebSockets** is a communication protocol that provides full-duplex, bidirectional communication channels over a single TCP connection. It's commonly used for applications that require real-time communication, such as chat apps, live data feeds, collaborative tools, or games.

## Key Characteristics

- Built on top of TCP
- Enables persistent, low-latency connections
- Initiated via HTTP(S) handshake and then upgraded to WebSocket
- Supports full-duplex communication
- Reduces the overhead of repeatedly opening new connections

## Common Use Cases

- Real-time chat and messaging
- Multiplayer online games
- Live sports or stock ticker dashboards
- Remote control and monitoring systems
- IoT data streaming

## How It Works

1. **Client initiates a handshake** using HTTP(S)
2. **Server responds with a special header** to switch to the WebSocket protocol
3. Once the connection is established, both parties can **send messages at any time**
4. The connection persists until **explicitly closed** by either party

## Advantages

- Lower overhead compared to polling or HTTP long-polling
- Real-time communication without latency from repeated requests
- Efficient for high-frequency, two-way data exchange

## Limitations

- May require additional setup for load balancing and scaling
- Not all environments or firewalls handle WebSocket connections gracefully
- May fall back to HTTP-based alternatives if unsupported

## WebSocket vs Other Protocols

| Feature            | WebSocket       | HTTP/2          | MQTT             | gRPC (with streaming) |
|--------------------|------------------|------------------|------------------|------------------------|
| Connection Type     | Full-duplex       | Request/response | Pub/sub (usually) | Unary or stream-based |
| Real-Time Capable   | Yes               | Limited          | Yes               | Yes                    |
| Built Over TCP      | Yes               | Yes              | Yes               | Yes                    |
| Built Over HTTP     | Starts via HTTP   | Yes              | No                | Yes                    |
| Lightweight         | Moderate          | Heavy            | Very light        | Moderate               |
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
- [[Protocols/Transport]]
- [[Protocols/Application]]

## See Also

- [MDN: WebSockets Overview](https://developer.mozilla.org/en-US/docs/Web/API/WebSockets_API)
- [RFC 6455 – The WebSocket Protocol](https://datatracker.ietf.org/doc/html/rfc6455)
