# SDP (Session Description Protocol)

## 📝 Summary

**SDP (Session Description Protocol)** is a format and protocol used to describe multimedia communication sessions. It is not a transport protocol itself, but a **declarative format** used to convey media session parameters between endpoints — such as codecs, IP addresses, ports, and session timing.

SDP is a critical part of technologies like **[[WebRTC]]**, **[[SIP]] ([[VoIP]])**, and other multimedia signaling protocols. It was originally defined in the context of **[[RTP]] (Real-time Transport Protocol)** and **MBONE (Multicast Backbone)** use.

---

## 📚 OSI Model Placement

| Layer       | Name             | SDP's Role                        |
|-------------|------------------|-----------------------------------|
| Layer 7     | Application       | Describes session setup metadata |
| Layer 4     | Transport         | Often used with UDP/TCP          |
| Layer 3     | Network           | Describes IP addressing           |

SDP doesn't carry media or control connections — it's a **metadata format**, typically transported over other signaling protocols (e.g., SIP or WebRTC negotiation).

---

## 📦 What It Contains

An SDP message describes:

- **Session name and purpose**
- **Media types** (audio, video, data)
- **Transport protocols** (e.g., RTP, UDP)
- **Codec information**
- **Network information** (IP addresses and ports)
- **Session timing and repeat times**
- **Encryption and authentication parameters** (e.g., DTLS fingerprints)
- Optional **ICE candidates** for NAT traversal

---

## 🔄 Usage in Real Systems

| Technology     | Role of SDP                                     |
|----------------|--------------------------------------------------|
| WebRTC         | Describes ICE candidates, codecs, and media     |
| SIP / VoIP     | Defines RTP session parameters                  |
| RTSP           | Sometimes used to describe media session setup  |
| Telepresence   | Synchronizes streams across endpoints           |

---

## 🔧 Typical Flow in WebRTC

1. Peer A creates an **SDP offer** (with ICE, media, codecs, etc.)
2. Peer B receives and responds with an **SDP answer**
3. Media is exchanged using agreed-upon parameters
4. ICE connectivity checks begin

---

## ⚙️ Not a Transport Protocol

SDP doesn’t send or receive media — it’s usually transported over:

- SIP
- HTTP/HTTPS (via WebRTC API)
- XMPP (in Jingle)
- RTSP

---

## 📜 Relevant Standards

- **RFC 4566** – Session Description Protocol
- **RFC 3264** – Offer/Answer Model using SDP
- **RFC 5768** – ICE extensions
- **RFC 3261** – SIP (uses SDP in signaling)

---

## 💡 Use Cases

- WebRTC session negotiation
- SIP-based VoIP calls
- RTSP-based streaming services
- Any situation where media format/capabilities need to be negotiated

---

## 🔄 Comparison Table

| Feature                    | SDP         | SIP            | RTSP           | ICE/STUN/TURN    |
|----------------------------|-------------|----------------|----------------|------------------|
| Type                       | Metadata    | Signaling      | Control        | NAT traversal    |
| Media transport            | ❌ No        | ❌ No           | ❌ No           | ❌ No             |
| Media description          | ✅ Yes       | ✅ Via SDP      | ✅ Via SDP      | ❌ No             |
| Used in WebRTC             | ✅ Yes       | ❌ Not directly | ❌ Not common   | ✅ Yes            |
| Used in VoIP               | ✅ Yes       | ✅ Yes          | ❌ Rarely       | ✅ Yes            |

---

## ✅ Pros

- Human-readable and simple to parse
- Flexible — supports many codecs and formats
- Well-integrated with standards like SIP, WebRTC
- Clear division of media/control responsibilities

## ❌ Cons

- Not encrypted (unless encapsulated)
- Limited extensibility
- Poor support for dynamically changing media sessions
- Parsing complexity increases with features like ICE

---

## 🧠 Strengths & Weaknesses

**Strengths:**
- Widely supported across VoIP and WebRTC ecosystems
- Declarative and concise
- Compatible with many media formats and protocols

**Weaknesses:**
- Sensitive to formatting (whitespace, syntax)
- Lacks negotiation logic — needs to be combined with Offer/Answer model
- Can be inconsistent between browsers and platforms (in WebRTC)

---

## 🧭 See Also

- [[WebRTC]]
- [[ICE]]
- [[STUN]]
- [[TURN]]
- [[RTP]]
- [[SIP]]
- [[DTLS]]
- [[UDP]]
