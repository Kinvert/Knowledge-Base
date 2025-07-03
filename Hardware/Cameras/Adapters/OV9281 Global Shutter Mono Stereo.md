# 🔲 1MP*2 OV9281 Global Shutter Monochrome Stereoscopic Camera

**1MP*2 OV9281** is a compact stereo camera module that integrates **two OV9281 global shutter monochrome sensors**. This type of module is widely used for computer vision tasks requiring **precise timing**, such as **Visual Odometry (VO)**, **SLAM**, **gesture recognition**, and **high-speed motion capture**.

---

## 📷 Overview

- **Sensors**: Dual OmniVision **OV9281**, 1MP global shutter, monochrome
- **Resolution**: 1280×800 (per sensor)
- **Shutter**: **Global Shutter** – captures the whole frame at once
- **Interface**: **1x MIPI CSI-2 Ribbon Connector** (usually 22-pin FPC)
- **Data Transmission**: Uses **virtual channels** or **lane multiplexing** to send both image streams through a single CSI interface
- **Target Platforms**: Jetson Orin Nano, Xavier NX, Raspberry Pi 5, Raspberry Pi CM4 (w/ expansion board)

---

## 🔍 Why Global Shutter?

| Shutter Type   | Feature                                      | Notes                                                                 |
|----------------|----------------------------------------------|-----------------------------------------------------------------------|
| Rolling Shutter| Reads image row-by-row                       | Distortion in fast motion scenes (jello effect, skew)                 |
| Global Shutter | Captures entire frame simultaneously         | Great for robotics, drones, high-speed motion, feature tracking       |

Global shutter avoids motion artifacts by capturing all pixels at once, unlike rolling shutter which scans line-by-line. This is crucial in real-time 3D vision systems and when synchronizing stereo input for depth perception.

---

## 🧪 How Does It Work Over One CSI-2?

The OV9281 stereo module typically uses one of the following techniques:

- **MIPI CSI Virtual Channels**: CSI-2 supports up to 4 virtual channels over the same physical lanes. Each camera feeds a separate virtual channel.
- **Hardware Aggregation**: Some boards aggregate both sensor outputs via FPGA or CSI multiplexer onboard.
- **Firmware Sync**: Sensors can be triggered to ensure exact temporal alignment.

Your software (e.g. Arducam drivers, V4L2 kernel modules) must support **virtual channels** to differentiate left/right streams.

---

## ⚙️ Supported Boards

| Board               | CSI Support | Virtual Channel Support | Notes                                               |
|--------------------|-------------|--------------------------|-----------------------------------------------------|
| Jetson Orin Nano   | Yes (22-pin)| Yes                      | Ideal for this module with driver support (JetPack 5+) |
| Jetson Xavier NX   | Yes (15/22) | Yes                      | Custom carrier boards or Arducam adapters needed    |
| Raspberry Pi 5     | Yes (2x 22) | Partial                  | Limited software support, improving over time       |
| Raspberry Pi CM4   | Yes (custom)| Yes (via adapters)       | Requires breakout board (e.g., StereoPi)            |
| Jetson Nano 4GB    | Yes (15-pin)| No (typically)           | Needs adapters or mux; single cam only by default   |

---

## 🎯 Use Cases

- [[Monocular SLAM]] and [[Stereo Cameras]]
- [[Visual Odometry]]
- [[Gesture Recognition]]
- [[Depth Estimation]]
- Robotics & drones

---

## 🔁 Comparison to Common Alternatives

| Camera Module       | Sensor Type       | Shutter        | Stereo Support | Color | FPS      | Notes                                 |
|---------------------|-------------------|----------------|----------------|-------|----------|----------------------------------------|
| **1MP*2 OV9281**     | OV9281 (x2)       | Global         | Yes            | Mono  | 60–120fps| Fast stereo, accurate timing, mono only |
| **IMX219**           | Sony IMX219       | Rolling        | No (single)    | Color | 30–60fps | Common Pi Cam V2 sensor                |
| **IMX477 HQ**        | Sony IMX477       | Rolling        | No             | Color | 30–60fps | High res, rolling shutter              |
| **IMX708 (Pi Cam 3)**| Sony IMX708       | Rolling        | No             | Color | ~60fps   | Better HDR, for Raspberry Pi 5         |
| **OV5647**           | OmniVision        | Rolling        | No             | Color | ~30fps   | Obsolete, used in Pi Cam V1            |

---

## 🏆 Strengths

- ✅ Real-time accurate stereo data
- ✅ Global shutter eliminates motion artifacts
- ✅ Compact dual-sensor form factor
- ✅ Uses one CSI interface, saving pins and space

---

## ⚠️ Weaknesses

- ⚠️ Typically monochrome (not color)
- ⚠️ Software driver support can be tricky
- ⚠️ Bandwidth limits depending on CSI lane count
- ⚠️ Not all platforms support dual camera on single CSI

---

## 🔗 Related Topics

- [[Stereo Cameras]]
- [[Raspberry Pi Camera]]
- [[Global Shutter]]
- [[MIPI CSI-2 Protocol]]
- [[Arducam Multi Camera Adapter Module V2.2]]
- [[Jetson Family]]
- [[Depth Estimation]]

---

## 🌐 External Links

- [Arducam Dual OV9281 Camera](https://www.arducam.com/product/1mp-ov9281-global-shutter-monochrome-stereo-camera/)
- [Jetson CSI Camera Support Matrix](https://developer.nvidia.com/embedded/jetson-modules)

---

## 📂 Suggested Folder Location

`Hardware/Cameras/Modules/Stereo`
