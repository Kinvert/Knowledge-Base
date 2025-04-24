# TCP (Transmission Control Protocol)

## 📝 Summary

TCP is a **connection-oriented** protocol that operates at the **Transport Layer (Layer 4)** of the **OSI model**. It provides **reliable, ordered, and error-checked** delivery of data between applications across a network. TCP is one of the core protocols of the Internet Protocol Suite, often paired with IP (Internet Protocol) as **TCP/IP**.

---

## 📚 OSI Model Placement

| Layer       | Name            | Role with TCP                      |
|-------------|------------------|------------------------------------|
| Layer 4     | Transport         | TCP ensures reliable communication |
| Layer 3     | Network           | IP handles routing of packets      |
| Layer 2     | Data Link         | Ethernet, Wi-Fi, etc.              |
| Layer 1     | Physical          | Physical transmission medium       |

---

## ⚙️ Characteristics

- **Connection-oriented**: Requires a three-way handshake to establish a session
- **Reliable**: Guarantees delivery and order of packets
- **Flow control**: Manages data rate via sliding window protocol
- **Error checking**: Includes checksum validation and retransmission on error
- **Congestion control**: Adapts transmission rate to network conditions

---

## 🤝 TCP Handshake Process

1. **SYN** — Client sends synchronization request
2. **SYN-ACK** — Server acknowledges and responds
3. **ACK** — Client sends acknowledgment to complete handshake

---

## 🧠 Common Use Cases

- **Web browsing** (HTTP/HTTPS)
- **File transfers** (FTP, SCP)
- **Email protocols** (SMTP, IMAP, POP3)
- **Remote access** (SSH, Telnet)
- **Database connections**

---

## 🧱 TCP Packet Structure

- **Source Port** (16 bits)
- **Destination Port** (16 bits)
- **Sequence Number** (32 bits)
- **Acknowledgment Number** (32 bits)
- **Header Length & Flags** (control bits like SYN, ACK, FIN)
- **Window Size** (16 bits)
- **Checksum**
- **Urgent Pointer**
- **Options + Padding**
- **Payload Data**

---

## 🆚 TCP vs UDP

| Feature              | TCP                              | UDP                              |
|----------------------|-----------------------------------|----------------------------------|
| Connection setup     | Required (3-way handshake)        | None                             |
| Reliability          | Guaranteed delivery and order     | No guarantees                    |
| Overhead             | High (20+ byte header)            | Low (8 byte header)              |
| Speed                | Slower due to checks              | Faster                           |
| Use case             | Web, file transfer, database      | Streaming, gaming, DNS, VoIP     |

---

## 📡 Hardware & Networking Considerations

- **NAT traversal** is generally more reliable for TCP
- **Firewall rules** often favor TCP due to its statefulness
- **TCP Offload Engine (TOE)** supported by some NICs
- **MTU fragmentation** handling built in

---

## 📜 Related Protocols and Standards

- **Defined by**:  
  `RFC 793 - Transmission Control Protocol`  
  (plus many extensions like RFC 7323 for high-performance)

- Protocols that use TCP:
  - **HTTP/HTTPS**
  - **FTP**
  - **SMTP/IMAP/POP3**
  - **SSH**
  - **Telnet**

---

## ✅ Pros

- Reliable and ordered delivery
- Error detection and recovery
- Built-in congestion and flow control
- Widely supported and understood

## ❌ Cons

- Higher latency due to setup and checks
- Larger header overhead
- Less suited for real-time applications
- More complex protocol state machine

---

## 🔗 See Also

- [[UDP]]
- [[OSI]]
- [[IP]] (Internet Protocol)
- [[QUIC]]
- [[TLS]]
- [[HTTP]]
