---
title: cameractrls
tags: [software, hardware-interface, cameras, linux, v4l2, ptz]
aliases: [Camera Controls for Linux, Cameractrls]
---

# cameractrls

**cameractrls** is a Linux camera-control utility with CLI, GTK GUI, and SDL viewer modes. It focuses on [[V4L2]] controls for USB webcams and camera devices, including exposure, focus, zoom, and vendor-specific PTZ-style controls on supported hardware.

---

## 🧠 Summary

- Works with Linux camera devices exposed through `/dev/video*`.
- Provides command-line and GUI access to camera controls.
- Supports presets and restoring settings when devices reconnect.
- Includes extensions for devices such as Logitech BRIO, Razer Kiyo Pro, Dell UltraSharp WB7022, and AnkerWork C310.
- Available through Linux packaging channels such as Flatpak.

---

## 📊 Comparison Chart

| Tool | Platform | Control Layer | Best Fit |
|------|----------|---------------|----------|
| cameractrls | Linux | [[V4L2]] / UVC controls | Webcam tuning and USB camera PTZ |
| [[v4l2-ctl]] | Linux | V4L2 CLI | Scripting and debugging camera controls |
| [[OBS PTZ Controls]] | Linux, macOS, Windows | VISCA, Pelco, ONVIF, UVC | PTZ control inside OBS |
| [[PTZ Controller]] | Windows | Pelco, VISCA, ONVIF, vendor protocols | Legacy PTZ camera control |
| ONVIF CLI tools | Linux | ONVIF SOAP/IP | Network-camera automation |

---

## ✅ Strengths

- Linux-native and open source
- Friendlier than raw `v4l2-ctl` for everyday adjustment
- Useful for webcams with pan, tilt, zoom, focus, and exposure controls
- Can restore saved camera settings automatically

---

## ❌ Weaknesses

- Not a full Pelco D or VISCA serial PTZ controller
- Control availability depends on the camera driver exposing V4L2 controls
- Mostly aimed at USB/UVC-style devices rather than surveillance DVR setups

---

## 🔗 Related Notes

- [[OBS PTZ Controls]]
- [[PTZ Controller]]
- [[V4L2]]
- [[v4l2-ctl]]
- [[USB Cameras]]
- [[Security Cameras]]

---

## 🌐 External Links

- [cameractrls GitHub Repository](https://github.com/soyersoyer/cameractrls)
- [cameractrls on Flathub](https://flathub.org/en/apps/hu.irl.cameractrls)
