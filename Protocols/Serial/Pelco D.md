---
title: Pelco D
tags: [protocols, serial, camera-control, security-cameras, ptz]
aliases: [Pelco-D, Pelco D Protocol]
---

# Pelco D

**Pelco D** is a simple binary control protocol for [[PTZ]] (Pan-Tilt-Zoom) security cameras. It is commonly sent over [[RS-232]] or [[RS-485]] links by DVRs, joystick controllers, and tools such as PTZ Controller software.

---

## ⚙️ Overview

Pelco D commands are 7-byte hexadecimal frames:

| Byte | Field | Purpose |
|------|-------|---------|
| 1 | Sync | Always `0xFF` |
| 2 | Address | Camera/receiver address, usually `0x01`-`0xFF` |
| 3 | Command 1 | Focus, iris, camera on/off, scan bits |
| 4 | Command 2 | Pan, tilt, zoom, focus direction bits |
| 5 | Data 1 | Usually pan speed |
| 6 | Data 2 | Usually tilt speed |
| 7 | Checksum | Sum of bytes 2-6 modulo 256 |

Example: `FF 01 00 04 3F 00 44` tells camera address `1` to pan left at high speed.

---

## 🧠 Core Concepts

- **Addressed devices**: Multiple cameras can share a bus when each has a unique address.
- **Bitmapped commands**: Movement and lens actions are packed into command bytes.
- **Speed fields**: Pan and tilt speed values are carried separately from direction bits.
- **No discovery layer**: The controller must know the serial port, baud rate, address, and camera support.
- **Vendor variation**: Many cameras support basic Pelco D while differing on presets or extended commands.

---

## 📊 Comparison Chart

| Protocol | Typical Transport | Main Use | Notes |
|----------|-------------------|----------|-------|
| Pelco D | RS-232/RS-485 | Legacy PTZ cameras | Compact 7-byte binary frames |
| Pelco P | RS-232/RS-485 | Pelco PTZ cameras | Related Pelco protocol with different framing |
| VISCA | RS-232/RS-485/IP | Sony-style PTZ cameras | Common in broadcast and conference cameras |
| ONVIF | IP/Ethernet | Network security cameras | Higher-level discovery and control |
| HTTP CGI APIs | IP/Ethernet | IP camera control | Vendor-specific endpoints |

---

## 🔧 Use Cases

- Driving surveillance PTZ domes from a joystick or DVR
- Testing camera pan, tilt, zoom, focus, iris, and preset commands
- Bridging legacy PTZ hardware into newer control systems
- Debugging serial camera-control wiring with a USB-to-RS-485 adapter

---

## ✅ Strengths

- Very small frame format
- Easy to generate from scripts or embedded firmware
- Widely supported by older and low-cost PTZ cameras
- Works over long RS-485 cable runs

---

## ❌ Weaknesses

- Minimal self-description or negotiation
- Serial settings and camera address must be configured manually
- Extended commands are not always portable across vendors
- Provides control only, not video transport

---

## 🔗 Related Concepts

- [[Serial Protocols]]
- [[UART]]
- [[RS-232]]
- [[RS-485]]
- [[PTZ Controller]]
- [[OBS PTZ Controls]]
- [[Security Cameras]]
- [[Industrial Cameras]]

---

## 🌐 External Resources

- [CommFront Pelco-D Protocol Tutorial](https://www.commfront.com/pages/pelco-d-protocol-tutorial)
- [PTZ Controller supported protocol list](https://www.ptzcontroller.com/2011/10/ptz-camera-protocol-supported-by-ptz-controller/)
