---
title: kRPC
aliases: [krpc, Kerbal RPC]
tags: [RPC, KerbalSpaceProgram, Protobuf, Scripting]
---

# kRPC

**kRPC** is a Remote Procedure Call (RPC) server for [Kerbal Space Program (KSP)](https://kerbalspaceprogram.com), allowing external scripts and programs to control the game by calling functions remotely over a network connection.

It enables advanced automation, telemetry, and visualization for [[KSP]] through languages like Python, C++, Java, and more.

---

## 🚀 Overview

- **Full Name**: Kerbal Remote Procedure Call (kRPC)
- **Primary Use Case**: External control of spacecraft and access to game data in KSP
- **Transport Layer**: [[TCP]], [[WebSockets]], [[Serial]]
- **[[Serialization]]**: [Protocol Buffers](https://developers.google.com/protocol-buffers)
- **NOT** gRPC: While it uses Protobuf like gRPC, it has its own transport and protocol structure.

---

## 📦 Features

- Call KSP functions remotely in real-time
- Read and write KSP game state (telemetry, position, control, etc.)
- Multilanguage support via auto-generated client libraries
- Support for streams, telemetry subscriptions, and events
- Extendable with custom scripts on both client and server

---

## 🌐 Communication Protocols

| Component         | Technology Used |
|------------------|------------------|
| RPC Mechanism     | Custom over Protobuf |
| Transport         | TCP, WebSocket, Serial |
| Message Format    | Protobuf (v2) |
| Streaming         | Supported via subscriptions |

---
```python
import krpc

# Connect to the game
conn = krpc.connect(name='My Mission')

# Get the active vessel
vessel = conn.space_center.active_vessel

# Set throttle to full
vessel.control.throttle = 1.0

# Activate the next stage
vessel.control.activate_next_stage()
```

---

## 📖 See Also

- [[RPC]]
- [[gRPC]]
- [[Protocol Buffers]]
- [[KSP]]
- [[Python]]

---

## 🔗 External Resources

- [GitHub Repo](https://github.com/krpc/krpc)
- [kRPC Docs](https://krpc.github.io/krpc/index.html)
- [Protocol Buffers](https://protobuf.dev
- [KRPC.MechJeb](https://github.com/Genhis/KRPC.MechJeb)
- https://github.com/marcnesium-iv/SuicideBurnAutopilot
- 
