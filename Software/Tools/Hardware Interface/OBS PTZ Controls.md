---
title: OBS PTZ Controls
tags: [software, hardware-interface, cameras, ptz, linux, obs]
aliases: [PTZ Controls, obs-ptz, OBS PTZ Plugin]
---

# OBS PTZ Controls

**OBS PTZ Controls** is an open-source plugin for controlling PTZ cameras from inside [[OBS Studio]]. It is one of the closer Linux-friendly alternatives to Windows-only PTZ Controller software, especially for streaming, recording, and live-production workflows.

---

## 🧠 Summary

- Adds a dockable PTZ control panel to OBS.
- Supports multiple cameras and can switch the active camera based on the active OBS scene.
- Supports pan, tilt, zoom, focus, presets, hotkeys, joystick control, and camera power controls.
- Protocol support includes [[VISCA]] over serial/IP, [[Pelco D]], Pelco P, experimental [[ONVIF]], and USB/UVC cameras.
- Runs on Windows, macOS, and Linux through OBS Studio.

---

## ⚙️ Key Features

| Feature | Description |
|---------|-------------|
| OBS Dock | Camera controls live inside the OBS interface |
| Scene Awareness | Can auto-select camera controls based on active scene |
| Protocol Support | VISCA, Pelco D/P, ONVIF, and UVC camera control |
| Presets | Saves and recalls PTZ positions |
| Automation | PTZ Actions can trigger movement when scenes change |
| Input Devices | Supports hotkeys and joystick-style control |

---

## 📊 Linux PTZ Software Comparison

| Tool | Linux Support | Protocols / API | Best Fit |
|------|---------------|-----------------|----------|
| OBS PTZ Controls | Yes | VISCA, Pelco D/P, ONVIF, UVC | PTZ control inside OBS |
| [[cameractrls]] | Yes | [[V4L2]] controls and vendor extensions | Linux webcam tuning and UVC PTZ |
| onvif_control | Yes | ONVIF SOAP CLI | Scripted IP-camera PTZ automation |
| Home Assistant ONVIF | Yes | ONVIF integration | Smart-home camera automation |
| Hardware PTZ Keyboard | OS-independent | Pelco, VISCA, ONVIF | Dedicated operator control |

---

## ✅ Strengths

- Native fit for Linux streaming and recording setups
- Open source and integrated directly into OBS
- Supports both serial-style PTZ protocols and IP camera control
- Useful for scene-driven camera automation

---

## ❌ Weaknesses

- Best suited to OBS workflows, not general surveillance management
- ONVIF support may be less complete than dedicated ONVIF tools
- Serial PTZ setups still require USB-to-RS-232/RS-485 adapters and correct baud/address settings
- Camera compatibility depends on protocol details and vendor behavior

---

## 🔗 Related Notes

- [[PTZ Controller]]
- [[Pelco D]]
- [[V4L2]]
- [[v4l2-ctl]]
- [[OBS Studio]]
- [[Security Cameras]]

---

## 🌐 External Links

- [obs-ptz GitHub Repository](https://github.com/glikely/obs-ptz)
- [OBS Forum PTZ Controls Resource](https://obsproject.com/forum/resources/ptz-controls.1284/)
- [cameractrls GitHub Repository](https://github.com/soyersoyer/cameractrls)
- [Cameractrls on Flathub](https://flathub.org/en/apps/hu.irl.cameractrls)
