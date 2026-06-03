---
title: PTZ Controller
tags: [software, hardware-interface, cameras, ptz, security-cameras]
aliases: [PTZ Controller Software, Serial Port Tool PTZ Controller]
---

# PTZ Controller

**PTZ Controller** is Windows software for controlling [[PTZ]] (Pan-Tilt-Zoom) cameras from a PC using keyboard, mouse, joystick, gamepad, serial ports, or Ethernet. It is commonly referenced around [[Pelco D]] because Pelco D is its default camera-control protocol.

---

## 🧠 Summary

- Controls pan, tilt, zoom, focus, iris, presets, auto scan, and auxiliary functions.
- Supports serial camera links through COM ports and network camera control over Ethernet.
- Common protocols include [[Pelco D]], Pelco P, [[VISCA]], [[ONVIF]], Bosch, Panasonic, Samsung, LG, Canon, and related vendor protocols.
- Basic setup usually means selecting protocol, COM port or network target, baud rate, and camera address.
- Useful when replacing a physical PTZ keyboard or joystick during testing and integration.

---

## ⚙️ Key Features

| Feature | Description |
|---------|-------------|
| Protocol Selection | Chooses Pelco D/P, VISCA, ONVIF, and vendor protocols |
| Serial Setup | Selects COM port, baud rate, and PTZ address |
| Camera Motion | Controls pan, tilt, zoom, focus, iris, and scan |
| Presets | Stores and recalls camera positions |
| Input Devices | Supports keyboard, mouse, USB joystick, gamepad, and Xbox controllers |
| Remote Control | Can run as a server and accept HTTP control requests |

---

## 📊 Comparison Chart

| Tool | Platform | Protocol Focus | Best Fit |
|------|----------|----------------|----------|
| PTZ Controller | Windows | Pelco D/P, VISCA, ONVIF, vendor protocols | Legacy serial PTZ camera testing |
| [[OBS PTZ Controls]] | Windows, macOS, Linux | VISCA, Pelco, ONVIF, UVC | Live production inside OBS |
| [[cameractrls]] | Linux | [[V4L2]] camera controls | USB webcam and UVC PTZ adjustment |
| ONVIF Device Manager | Windows | ONVIF | IP camera discovery and testing |
| Hardware PTZ Keyboard | Embedded controller | Pelco, VISCA, ONVIF | Dedicated operator control surface |

---

## ✅ Strengths

- Broad protocol support for older and mixed PTZ fleets
- Practical for checking serial wiring, baud rate, and camera addresses
- Replaces dedicated joystick hardware for many test benches
- Exposes common PTZ operations in one interface

---

## ❌ Weaknesses

- Windows-focused tool
- Requires correct camera protocol and transport settings
- Protocol coverage does not guarantee every vendor extension works
- Not a video management system; it controls cameras rather than recording streams

---

## 🔗 Related Notes

- [[Pelco D]]
- [[RS-232]]
- [[RS-485]]
- [[UART]]
- [[Security Cameras]]
- [[Industrial Cameras]]

---

## 🌐 External Links

- [PTZ Controller User Manual](https://serialporttools.com/PTZController/tutorials/PTZController_UserManual.htm)
- [PTZ Controller Product Page](https://www.serialporttool.com/GK/product/ptz-controller-software/)
- [Control Pelco PTZ Camera with PTZ Controller](https://www.ptzcontroller.com/2013/10/control-pelco-ptz-camera-with-ptz-controller/)
