# UDP (User Datagram Protocol)

## 📝 Summary

UDP (User Datagram Protocol) is a **transport layer** protocol in the **Internet Protocol Suite** (TCP/IP model), also corresponding to **Layer 4** of the **OSI model**. Unlike TCP, UDP is **connectionless** and does **not guarantee delivery**, making it lightweight, fast, and ideal for applications where speed matters more than reliability.

---

## 📚 OSI Model Placement

| Layer       | Name            | Role with UDP                     |
|-------------|------------------|----------------------------------|
| Layer 4     | Transport         | UDP resides here                 |
| Layer 3     | Network           | IP handles routing of datagrams |
| Layer 2     | Data Link         | Ethernet, Wi-Fi, etc.           |
| Layer 1     | Physical          | Cables, radios, etc.            |

---

## ⚙️ Characteristics

- **Connectionless**: No handshake or session setup
- **Unreliable**: No built-in delivery acknowledgment, retries, or order enforcement
- **Low overhead**: Minimal headers (8 bytes compared to TCP's 20+ bytes)
- **Fast and efficient** for real-time and stateless communication

---

## 🧠 Common Use Cases

- **Streaming media** (video/audio)
- **Online multiplayer games**
- **VoIP (Voice over IP)**
- **DNS queries**
- **Sensor data transmission** in embedded/real-time systems

---

## 🔁 UDP Packet Structure

- **Source Port** (16 bits)
- **Destination Port** (16 bits)
- **Length** (16 bits) — header + data
- **Checksum** (16 bits) — optional error checking
- **Data** — payload being sent

---

## 🆚 UDP vs TCP

| Feature              | UDP                             | TCP                             |
|----------------------|----------------------------------|----------------------------------|
| Connection setup     | None (connectionless)            | Requires handshake (connection-oriented) |
| Reliability          | No guarantees                    | Guaranteed delivery, order       |
| Header size          | 8 bytes                          | 20 bytes (minimum)              |
| Speed                | Faster                           | Slower                          |
| Use case             | Streaming, gaming, VoIP          | Web, file transfer, email       |

---

## 🔌 Hardware & Networking Considerations

- **Routers/switches** treat UDP similarly to TCP at Layer 3
- **Firewall/NAT behavior** may differ; often stricter for UDP
- **Less buffering** needed on client-side due to lower overhead
- **Jitter and packet loss** common due to lack of retransmission

---

## 📡 Related Protocols and Standards

- **UDP itself** is defined by:  
  `RFC 768 - User Datagram Protocol`

- Common protocols layered on top of UDP:
  - **DNS** (RFC 1035)
  - **DHCP** (RFC 2131)
  - **SNMP** (RFC 1157)
  - **TFTP** (RFC 1350)
  - **RTP** (Real-Time Protocol for media)

---

## ✅ Pros

- Very fast
- Low overhead
- Stateless — scalable for large applications
- Works well in real-time situations

## ❌ Cons

- No guaranteed delivery or order
- No congestion control
- Not ideal for transferring critical or large data

---

## 🔗 See Also

- [[TCP]]
- [[OSI Model]]
- [[DNS]]
- [[RTP]]
- [[TFTP]]
