---
title: eCAL
aliases: [Enhanced Communication Abstraction Layer]
tags: [middleware, communication, robotics, distributed-systems, [[Protocols/Transport]], [[RPC]]]
---

# eCAL

**eCAL (Enhanced Communication Abstraction Layer)** is a high-performance publish-subscribe middleware framework developed by Continental. It's designed for real-time inter-process communication (IPC) and inter-host communication in distributed systems, especially in robotics and automotive applications.

## Key Features

- Publish/Subscribe and Client/Server communication
- Works across processes and hosts
- Supports multiple transport layers (shared memory, UDP, TCP)
- Cross-platform (Windows, Linux)
- Language bindings: C++, Python, C#, Java, Rust (community)
- Monitoring tools included

## Basic Architecture

- **Publishers** send messages on named topics.
- **Subscribers** listen for those messages on the same topic.
- **Services** allow for request/response patterns (RPC-style).

eCAL handles serialization (e.g., via protobuf), transport, and discovery automatically.

## Example Use Cases

- Robot component communication (e.g., sensors to controllers)
- Distributed systems in autonomous vehicles
- Real-time data acquisition across machines

## Example Code (Python)

> ✅ *Make sure `ecal5` is installed and your environment is set up properly.*

**Publisher:**

```python
from ecal5.core import eCALInit, eCALShutdown
from ecal5.pubsub import StringPublisher
import time

eCALInit()

pub = StringPublisher("chatter")

while True:
    pub.send("Hello from eCAL Python!")
    time.sleep(1)

eCALShutdown()
```

Subscriber:

```python
from ecal5.core import eCALInit, eCALShutdown
from ecal5.pubsub import StringSubscriber

def callback(topic_name, msg, time):
    print(f"[{topic_name}] {msg}")

eCALInit()

sub = StringSubscriber("chatter", callback)

try:
    while True:
        pass
except KeyboardInterrupt:
    pass

eCALShutdown()
```

## GUI Tools

- **eCAL Monitor** – See active publishers/subscribers and message statistics
    
- **eCAL Recorder** – Record message traffic for later replay
    
- **eCAL Play** – Replay recorded sessions
    

## Dependencies & Serialization

- Often uses **Google Protocol Buffers (protobuf)** for serialization
    
- Supports custom serialization formats
    

## Related Topics

- [[Protocols/RPC]]
    
- [[Protocols/Transport]]
    
- [[gRPC]]
    
- [[DDS (Data Distribution Service)]]
    
- [[ROS 2]]
    
- [[Protobuf]]
    
- [[ZeroMQ]]
    

## See Also

- [Official GitHub Repo](https://github.com/eclipse-ecal/ecal)
    
- [Documentation](https://ecal.io/)

- [Official eCAL Eclipse](https://eclipse-ecal.github.io/ecal/stable/index.html)
    
- [[kRPC]]
    
- [[ROS]]
