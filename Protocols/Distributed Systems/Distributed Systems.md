---
title: Distributed Systems
aliases:
  - Distributed Computing
  - Distributed Architecture
tags:
  - protocols
  - distributed-systems
  - systems
---

# Distributed Systems

**Distributed systems** coordinate computation and state across multiple machines, processes, or datacenters to provide scalability, fault tolerance, and availability beyond a single host.

They are essential for cloud services, robotics fleets, telemetry backends, and large-scale learning pipelines.

---

## Core design problems

- **Communication**: message reliability, ordering, and discovery.
- **Consistency**: what is visible at each node and when.
- **Fault tolerance**: handling node crashes and network partitions.
- **Coordination**: leader election, membership, and scheduling.
- **State management**: replication, recovery, and conflict resolution.

---

## Architectural patterns

- **Client-server**
  - Single service entrypoint with backing workers.
- **Peer-to-peer**
  - Nodes share work without strict hierarchy.
- **Sharded systems**
  - Split state by keyspace for throughput scale.
- **Event-driven (pub/sub)**
  - Loose coupling with eventual consistency options.
- **Dataflow pipelines**
  - Deterministic processing stages and replayable logs.

---

## Comparison table

| Architecture | Coupling | Scaling | Failure Isolation | Complexity | Typical Use |
|---|---|---|---|---|---|
| Monolith | Tight | Vertical/limited | Low | Low | Small internal tooling |
| Microservices | Loose | High | Medium | High | Robotics backend services |
| Actor/Reliable messaging | Message-centric | High | Medium-High | High | Real-time control coordination |
| Service-oriented APIs | API-first | Medium-High | Medium | Medium | Internal enterprise stacks |
| Peer-to-peer | Weak | Very High | Medium | Very High | Discovery and distributed sensing networks |

---

## Protocol and consistency helpers

- [[Distributed Systems Protocols]]
- [[Consensus Algorithms]]
- [[Vector Clocks]]
- [[Raft]]
- [[Paxos]]
- [[Distributed Databases]] (where available)

---

## When to use a distributed approach

Use distributed design when you need any of these:
- independent scale across data streams or robot fleets
- fault tolerance beyond single-box redundancy
- global collaboration across multiple locations
- high-throughput telemetry ingestion and replay

Avoid pure distribution when:
- latency is dominated by WAN links
- consistency requirements are strict and simple consistency is easy
- team size cannot support operational complexity

---

## Related notes

- [[Distributed Systems Protocols]]
- [[Consistency Models]]
- [[Database Replication]]
- [[CAP Theorem]]
- [[Cloud Infrastructure]]

