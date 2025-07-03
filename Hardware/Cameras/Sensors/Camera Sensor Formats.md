# 📸 Camera Sensor Formats in Robotics and SBCs

This document provides an overview and comparison of popular **camera sensors and formats** frequently used in **robotics**, **embedded systems**, and **single-board computers** such as [[Raspberry Pi]], [[Jetson Family]], and others.

These sensors are often found in MIPI CSI-2 ribbon-based cameras and are critical for applications in Computer Vision, SLAM, Depth Estimation, and real-time robotics.

---

## 🎯 Overview of Common Camera Sensor Formats

| Sensor     | Type          | Shutter  | Resolution       | FPS     | Color | Notes                                                                 |
|------------|---------------|----------|------------------|---------|-------|-----------------------------------------------------------------------|
| **OV9281** | Monochrome    | Global   | 1280x800         | 60–120  | No    | Dual stereo variants exist. Ideal for SLAM, robotics, depth, etc.    |
| **OV5647** | RGB           | Rolling  | 2592x1944        | ~30     | Yes   | Legacy Pi Camera V1. Poor low-light performance.                      |
| **IMX219** | RGB           | Rolling  | 3280x2464        | 30–60   | Yes   | Pi Cam V2. Common, small, cheap.                                      |
| **IMX477** | RGB           | Rolling  | 4056x3040        | ~60     | Yes   | Pi HQ Camera, better optics, larger sensor. Good quality.             |
| **IMX708** | RGB           | Rolling  | 4608x2592        | 60–120  | Yes   | Pi Cam V3. Better HDR and low-light than predecessors.                |
| **IMX296** | Monochrome    | Global   | 1440x1080        | ~120    | No    | Precision, good in industrial vision, sometimes stereo.               |
| **IMX290** | Color/Mono    | Rolling  | 1920x1080        | ~120    | Yes   | Good in low-light, popular in security/AI vision.                     |
| **IMX378** | RGB           | Rolling  | 4032x3024        | ~30–60  | Yes   | Used in some phones, great quality for robotics if supported.         |
| **OV2311** | Monochrome    | Global   | 1600x1300        | ~60     | No    | High-res global shutter mono. Often stereo.                           |

---

## 📦 Which Boards Support Which Sensors?

| Sensor     | Raspberry Pi | Jetson Nano | Orin Nano / NX | CM4 / StereoPi | Arducam USB | Notes                                                 |
|------------|---------------|-------------|----------------|----------------|-------------|--------------------------------------------------------|
| **OV9281** | Yes (via Arducam) | Partial (via driver) | Yes (JetPack 5+) | Yes          | Yes         | Needs virtual channel support for stereo               |
| **OV5647** | Yes (V1)       | Yes         | Partial        | Yes            | Yes         | Deprecated, low quality                                |
| **IMX219** | Yes (V2)       | Yes         | Yes            | Yes            | Yes         | Popular, driver support is widespread                  |
| **IMX477** | Yes (HQ cam)   | Yes         | Yes            | Yes            | Yes         | High-quality but more expensive                        |
| **IMX708** | Pi 5 only      | No          | No             | No             | No          | Brand new; support just emerging                       |
| **IMX290** | No (natively)  | Yes         | Yes            | No             | Yes         | Needs driver support; great low-light                  |
| **OV2311** | No (natively)  | Partial     | Yes            | Yes            | Yes         | For stereo vision; good depth matching                 |

> 💡 Arducam, Waveshare, and others often provide camera boards with these sensors and custom drivers for SBCs.

---

## ✅ Use Case Matrix

| Application       | Suggested Sensor(s)            | Notes                                                  |
|-------------------|--------------------------------|--------------------------------------------------------|
| SLAM / VO     | OV9281, IMX296, OV2311         | Global shutter, stereo compatibility essential         |
| Low-light vision  | IMX290, IMX708                 | IMX290 especially for B&W or low-light tasks           |
| High-resolution imaging | IMX477, IMX708              | Raspberry Pi HQ and Cam 3 for static/fixed vision      |
| Stereo vision     | Dual OV9281, OV2311            | Must ensure camera sync and board driver compatibility |
| Color detection   | IMX219, IMX708, IMX378         | Rolling shutter acceptable in slow scenarios           |
| Depth estimation  | Stereo OV9281, TOF modules     | Depth from disparity or external TOF sensor fusion     |

---

## 🏆 Strengths and Weaknesses

| Sensor     | Strengths                                         | Weaknesses                                          |
|------------|---------------------------------------------------|-----------------------------------------------------|
| OV9281     | Low latency, fast global shutter, stereo support  | Mono only, niche driver support                    |
| OV5647     | Cheap and available                               | Rolling shutter, low quality                       |
| IMX219     | Balanced, cheap, decent resolution                | Rolling shutter artifacts                          |
| IMX477     | High-quality optics support                       | Price, bulkier                                     |
| IMX708     | Modern HDR sensor, higher FPS                     | Only works on Pi 5 and later                       |
| IMX290     | Great low-light, decent resolution                | Needs USB or advanced CSI camera driver support    |
| OV2311     | High-res mono, stereo global shutter              | No color, harder to source                         |

---

## 🔗 Related Notes

- [[Stereo Cameras]]
- [[Global Shutter]]
- [[Rolling Shutter]]
- [[MIPI CSI-2]]
- [[Raspberry Pi Camera]]
- [[Cameras]]

---
