# MAVLink

MAVLink is a compact binary telemetry and command protocol for unmanned vehicles and companion software. It is a widely used data link language between flight controllers, ground control stations, and higher-level autonomy software in hobby and pro drone stacks.

---

## 🧠 Overview

- MAVLink is packet-oriented and message-based.
- It is designed to work over serial, UDP, TCP, radio links, or USB-like transports as long as both endpoints exchange the same wire format.
- It is not a transport protocol by itself; it is a message frame and schema layer that rides on top of a transport.
- Typical MAVLink dialects include `common.xml` and stack-specific XML files (for example `ardupilot.xml` in ArduPilot ecosystems).

---

## 🎯 What it is and what it is not

What it is:

- A vehicle telemetry/control protocol for exchanging structured messages between nodes.
- A common vocabulary for commands, statuses, sensors, mission items, and diagnostics.
- A bridge format for both GCS and onboard software.

What it is not:

- A guaranteed transport protocol (delivery is not guaranteed by default).
- A video transport protocol.
- A mission-management UI (that's handled by tools like `QGroundControl`, `ArduPilot Mission Planner`, etc.).

---

## 🧱 Packet structure (practical level)

MAVLink has two practical protocol formats:

### MAVLink 1

- 1-byte magic marker.
- 1-byte message ID (0-255).
- no message extension fields.
- max payload 263 bytes.
- CRC and CRC extra on wire checks.

### MAVLink 2

- 0xFD magic marker and explicit protocol extension fields.
- 24-bit message ID (up to 16,777,216 possible message IDs).
- optional message signing.
- payload truncation support (trims trailing zero bytes in transport).
- max signed packet size up to 280 bytes.

In general, this means MAVLink 2 keeps MAVLink 1 compatibility on wire-level decoding where possible while adding extension and security features.

---

## 🧮 Core fields and behavior

MAVLink packets carry:

- Packet header (len, ids, flags, sequence, magic).
- Payload bytes (message data).
- Checksum (CRC-16/MCRF4XX) and CRC extra to verify definition compatibility.
- Optional signing block for MAVLink 2 packets.

Important implementation traits:

- Payloads are little-endian.
- Sender and receiver must share a compatible message definition.
- Message definitions are usually generated from XML dialect files.

The message model gives fixed IDs to message types (`HEARTBEAT`, `GLOBAL_POSITION_INT`, etc.), but semantics depend on the active dialect loaded by both ends.

---

## 🔁 Compatibility and versioning

### MAVLink 1 vs 2 in one place

| Topic | MAVLink 1 | MAVLink 2 |
|---|---:|---:|
| Frame marker | `0xFE` | `0xFD` |
| Message ID width | 8-bit | 24-bit |
| Max payload | 255 bytes payload section (263 total message size) | 255-byte payload with signing/truncation (280 max size) |
| Signing | no | optional |
| Extension fields | no | yes |
| Backward compatibility | legacy baseline | designed to interoperate with MAVLink 1 senders |

### Versioning behavior you should actually plan for

- If an autopilot is in MAVLink 1 mode, it may ignore newly-added MAVLink 2 extension bytes but still parse base fields.
- If both ends negotiate MAVLink 2 (or one end emits MAVLink 2 packets), extra capabilities such as signing and extensions become available.
- You still need explicit handling for transport MTU and packet loss in telemetry loops.

---

## 🌐 Transports and links

MAVLink is transport-agnostic at the design level. In practice, common links are:

- Serial (USB/UART, radio modem UARTs).
- UDP/TCP forwarded channels (common in simulation and companion computer stacks).
- Tunnel-like bridges where MAVLink frames are carried over external transport.

Important practical detail:

- MAVLink packets are lightweight and stream-oriented, so you should treat them like control/telemetry streams, not file transfers.

---

## 🧩 Dialects and message schemas

`common.xml` provides vehicle-agnostic primitives (state, attitude, GPS, RC, battery, mission).
Stack-specific dialects add custom messages and vendor features.

Because message layout is not included verbatim in each payload, sender and receiver must agree on:

- Message IDs.
- Field ordering.
- Field type/size.
- Extension fields for newer versions.

That is why a compatible XML definition set is critical for mixed environments.

---

## 🧭 Ecosystem integration points

Typical integration layers in this repo:

- `[[MAVProxy]]` for CLI routing, stream aggregation, and command scripting.
- `[[QGroundControl]]` for GUI-first setup, missions, map/telemetry, and logs.
- `[[ArduPilot Mission Planner]]` for ArduPilot-specific mission and operational workflows.
- `PX4 Boards` and `ArduPilot Boards` as common flight-controller families carrying this protocol.
- `[[SITL]]` toolchains where MAVLink is used for synthetic telemetry and commanding.
- Automation APIs via `MAVSDK` (external API layer for MAVLink-backed links).
- ROS integrations via `MAVROS` (`roslaunch` nodes and ROS topic bridges).

---

## 🛑 What MAVLink can and cannot do

### Can do

- Send low-latency vehicle state and commands.
- Carry mission items, commands, and waypoint set updates.
- Support parameter read/write and heartbeat/status loops.
- Drive companion computers and autonomy services over radio/IP links.
- Scale to many vehicle and component IDs through `sysid` and `compid`.

### Cannot do (or not ideal for)

- Reliable transport guarantees by itself (must be handled in higher logic).
- Full security of transport security stacks without signing configuration.
- High-bitrate streaming (use dedicated media channels for video/audio).
- Guaranteed ordering and retransmission semantics for all message classes.
- Deep policy-driven service orchestration without additional middleware.

---

## 🧪 What it is compatible with

### Compatibility (what plugs in naturally)

| Component Type | Compatibility | Typical Use |
|---|---|---|
| `ArduPilot Boards` | ✅ Native | Telemetry, commands, mission upload/download |
| `PX4 Boards` | ✅ Native | Ground station + offboard/computer workflows |
| `QGroundControl` | ✅ Native | Mission editing, parameter tuning, firmware helpers |
| `MAVProxy` | ✅ Native | Stream routing, failover links, CLI automation |
| `MAVSDK` | ✅ Native | API-driven companion/offboard control |
| `MAVROS` | ✅ Native | ROS topic/service bridge for ROS-based systems |

### Incompatible by design / poor fit

| Domain | Why it is weak / not a fit |
|---|---|
| Reliable guaranteed control links | No built-in ACK/retransmission for all flows |
| High-complexity transactional APIs | Needs explicit orchestration and state reconciliation |
| Real-time control across highly variable IP paths | You still need heartbeat, loss handling, and command confirmation patterns |
| Multi-vehicle authenticated command fabric | Needs strict signing key policy and transport hardening |

---

## 🧰 Security model

- MAVLink 1 has no signing.
- MAVLink 2 can append a signature block (link ID, timestamp, signature) for tamper detection.
- Signing is optional and must be enabled/configured on both ends where threat model requires it.
- Even with signing, higher-level operational security still depends on physical/logical link hardening and link authentication.

For signing setups you usually need:

- Secret key provisioning.
- Time/state tracking per `(linkID, sysid, compid)` tuple.
- Rollback and key-reset behavior for maintenance operations.

---

## 🧠 Comparison chart

| Protocol / System | Scope | Message model | Transport fit | Typical Use | Why compare |
|---|---|---|---|---|---|
| `MAVLink` | Vehicle telemetry/control protocol | Fixed message IDs in dialects | Serial, UDP, TCP | UAV/UGV/UGCS integrations | Core competitor in drone control space |
| `[[CAN]]` (and variants) | Vehicle internal bus | Frame-based IDs, control + diagnostics | Physical bus / CAN-FD variants | Low-level actuation and sensors | Determinism vs higher-level autonomy traffic |
| `[[UART]]` | Byte stream hardware interface | Link layer only | Serial PHY | Physical connectivity | Transport vs protocol distinction |
| `[[UDP]]` | Network transport | Stateless datagrams | IP networks | Low-latency telemetry forwarding | Needed when MAVLink runs over IP |
| `[[TCP]]` | Network transport | Stream transport | Reliable connection | Tooling and remote shells with less jitter control | Different latency/reliability profile |
| `[[DDS]]` | Middleware publish/subscribe | Topic typed interfaces | UDP/IP + vendor transport stacks | Multi-process robotics systems | Different abstraction and QoS model |
| `[[ROS2]]` | Robotics middleware | Message interfaces (`topic`, `service`, actions`) | DDS-backed | Large robot software graphs | MAVLink often appears as bridge interface |

---

## 🧯 Gotchas and failure modes

- MAVLink message rate tuning matters on constrained links; overconfigured streams can flood low-bandwidth radios.
- If frame parsing drifts, validate serial settings and baud on UART endpoints first.
- If multi-link setups are used, explicit link selection and forwarding direction avoid telemetry echo loops.
- Signed packets require careful key handling; avoid auto-propagating key setup messages across insecure links.
- Legacy stacks in mixed MAVLink versions should always be tested in hardware-in-the-loop before field deployment.

---

## ✅ When MAVLink is the best fit

- You need compact, mature UAV/UGV telemetry and command transport.
- You are integrating with Pixhawk-family or other MAVLink-first stacks.
- You need an ecosystem with many tooling options (mission, logs, offboard APIs, ROS bridges).
- You can tolerate probabilistic transport behavior with explicit fail-safe logic.

## ❌ When to prefer something else

- If your system requires end-to-end guaranteed packet delivery by design.
- If you need low-latency video-in-band with telemetry (use dedicated media transport).
- If you are building a large non-vehicle message graph as the first-class abstraction; consider higher-level middleware directly.

---

## 🔗 Related notes

- `[[MAVProxy]]`
- `[[QGroundControl]]`
- `[[ArduPilot Mission Planner]]`
- `[[SITL]]`
- `[[ArduPilot Boards]]`
- `[[PX4 Boards]]`
- `[[Brain-in-the-Loop Flight Simulation for ArduPilot and Pixhawk]]`
- `[[Serial Protocols]]`
- `[[UART]]`
- `[[TCP]]`
- `[[UDP]]`
- `[[ROS2]]`

---

## 🗂 Sources (official + authoritative)

- [MAVLink guide (overview)](https://mavlink.io/en/about/overview.html)
- [MAVLink packet serialization (v1/v2 formats, CRC, checksums)](https://mavlink.io/en/guide/serialization.html)
- [MAVLink message signing](https://mavlink.io/en/guide/message_signing.html)
- [MAVLink API basics (ArduPilot)](https://ardupilot.org/dev/docs/mavlink-basics.html)
- [MAVProxy startup and link handling](https://ardupilot.org/mavproxy/docs/getting_started/starting.html)
- [MAVLink forwarding behavior (MAVProxy)](https://ardupilot.org/mavproxy/docs/getting_started/forwarding.html)
- [MAVSDK connection options (UDP/TCP/serial)](https://mavsdk.mavlink.io/main/en/cpp/guide/connections.html)
- [MAVLink over ROS gateway (MAVROS)](https://docs.ros.org/en/rolling/p/mavros/index.html)
- [QGroundControl ecosystem docs](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/index.html)
