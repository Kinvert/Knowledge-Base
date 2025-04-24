# WebRTC (Web Real-Time Communication)

## 📝 Summary

**WebRTC** is a **real-time communication framework** enabling peer-to-peer **audio, video, and data** sharing directly between browsers and mobile apps without the need for plugins or external software. It is supported natively in most modern browsers and is a core part of web-based video conferencing and peer-to-peer file transfer technologies.

Developed and maintained by Google, WebRTC allows secure, low-latency communications by using **standardized APIs** and protocols under the hood, including **SRTP**, **ICE**, **STUN**, and **TURN**.

---

## 📚 OSI Model Placement

| Layer       | Name            | Role with WebRTC                           |
|-------------|------------------|---------------------------------------------|
| Layer 7     | Application       | JavaScript APIs, [[SDP]] negotiation       |
| Layer 4     | Transport         | UDP, TCP (used in fallback via TURN)       |
| Layer 3     | Network           | IP addresses for endpoint communication    |
| Layer 2     | Data Link         | Ethernet, Wi-Fi (used for packet delivery) |
| Layer 1     | Physical          | Physical hardware like routers, NICs       |

---

## 🧩 Core Protocol Stack

- **ICE** (Interactive Connectivity Establishment) – connection negotiation
- **STUN** (Session Traversal Utilities for NAT) – IP discovery
- **TURN** (Traversal Using Relays around NAT) – relaying when direct connection fails
- **DTLS** – datagram-level encryption
- **SRTP** – secure audio/video transmission
- **SCTP** – for WebRTC data channels

---

## 🔧 Key Features

- **Peer-to-peer** communication
- **No plugins** required
- **Built-in NAT traversal**
- **End-to-end encryption**
- **Supports audio, video, and arbitrary data**
- Works behind firewalls via TURN if needed

---

## 💡 Common Use Cases

- Video conferencing platforms (Zoom-like apps)
- Web-based chat systems
- Multiplayer browser-based games
- Remote desktop and file-sharing tools
- IoT device communication
- AR/VR in-browser experiences

---

## 🔄 Comparison Table

| Feature         | WebRTC        | WebSockets    | TCP             | UDP            |
|-----------------|---------------|---------------|------------------|----------------|
| Real-time Media | ✅ Yes         | ❌ No          | ❌ No             | ❌ No           |
| Data Channels   | ✅ Yes         | ✅ Yes         | ✅ Yes            | ✅ Yes          |
| Latency         | 🟢 Very low    | 🟡 Moderate    | 🔴 High           | 🟢 Very low     |
| NAT Traversal   | ✅ Yes         | ❌ No (needs help) | ❌ No        | ❌ No           |
| Encryption      | ✅ DTLS/SRTP   | ✅ TLS         | ✅ TLS (via app)  | ❌ Optional     |
| Browser-native  | ✅ Built-in    | ✅ Built-in    | ❌ No             | ❌ No           |

---

## ⚙️ Hardware / Network Considerations

- Optimized for **peer-to-peer**, reducing server load
- **TURN servers** required as fallback – may cause latency
- Relies on **browser audio/video subsystems**
- Works well even on constrained networks
- Supports **adaptive bitrate streaming**

---

## 📜 Related Standards and Specs

- **IETF**: Various RFCs related to STUN, TURN, ICE, DTLS, SRTP
- **W3C**: JavaScript API standardization
- Notable protocols:
  - `RFC 5245` – ICE
  - `RFC 5766` – TURN
  - `RFC 5389` – STUN
  - `RFC 3711` – SRTP

---

## ✅ Pros

- No external dependencies for browsers
- Low-latency, secure peer-to-peer communication
- Supports rich media and data
- Encrypted by default
- Open-source and widely supported

## ❌ Cons

- Complex negotiation process (ICE/STUN/TURN)
- Can require TURN relay servers (adds latency and cost)
- Limited direct control of underlying transport layer
- Some corporate firewalls block UDP (WebRTC fallback to TCP or fail)

---

## 🧠 Strengths & Weaknesses

**Strengths:**
- Great for browser-native apps
- Modern web dev ecosystem support
- True peer-to-peer where possible
- Excellent performance on mobile and desktop

**Weaknesses:**
- Setup complexity (especially ICE/TURN handling)
- Debugging can be difficult
- Compatibility quirks across browsers
- Heavily reliant on NAT traversal success

---

## 🧭 See Also

- [[UDP]]
- [[TCP]]
- [[QUIC]]
- [[WebSockets]]
- [[Real-Time Communication]]
- [[DTLS]]
- [[STUN]]
- [[TURN]]
- [[SCTP]]
- [[ICE]]

## Examples
- Django
	- https://github.com/suitenumerique/meet
	- https://github.com/vsjakhar/Django-WebRtc
	- https://github.com/craigmadden/videochat
	- https://github.com/antsmc2/webrtc_meetings
- Javascript
	- https://github.com/webtorrent/webtorrent
	- https://github.com/webrtc/samples
- Go
	- https://github.com/pion/webrtc
- C++
	- https://github.com/paullouisageneau/libdatachannel
