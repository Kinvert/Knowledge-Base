# 🟣 Raspberry Pi Camera Modules

The **Raspberry Pi Camera Modules** are official camera boards designed to connect via the **CSI (Camera Serial Interface)** ribbon connector. They’re compact, affordable, and widely used for projects in photography, computer vision, robotics, surveillance, and more.

---

## 🧠 Summary

- Connects to Raspberry Pi boards using the CSI ribbon cable.
- Compatible with most Pi boards (Model A, B, Zero, Compute Module).
- Supported officially on **Raspberry Pi OS** and, with effort, on **Ubuntu**, **Debian**, and others.
- Several versions and variants exist, differing in sensor, resolution, features like autofocus, and low-light performance.

---

## ⚙️ Comparison of Raspberry Pi Camera Versions

| Model                       | Sensor            | Resolution | Max Video | Features                     | Year | Notes                                        |
|-----------------------------|------------------|------------|-----------|------------------------------|-------|----------------------------------------------|
| **Camera Module V1.3**       | OmniVision OV5647 | 5 MP       | 1080p30   | Fixed-focus                   | 2013  | Discontinued; first-gen camera module        |
| **Camera Module V2.1**       | Sony IMX219       | 8 MP       | 1080p30   | Fixed-focus                   | 2016  | Better low light, sharper than V1.3          |
| **Camera Module 3 (Standard)** | Sony IMX708     | 12 MP      | 1080p50   | Autofocus, HDR                | 2023  | Autofocus, HDR, improved low-light           |
| **Camera Module 3 (Wide)**   | Sony IMX708       | 12 MP      | 1080p50   | Autofocus, wide-angle lens    | 2023  | Wider FOV for robotics, drones               |
| **High Quality (HQ)**        | Sony IMX477       | 12.3 MP    | 1080p30   | C/CS lens mount, manual focus | 2020  | Pro projects, interchangeable lens support   |
| **Global Shutter Camera**    | Sony IMX296       | 1.6 MP     | 60fps     | Global shutter                | 2023  | For machine vision, fast motion capture      |
| **NoIR V1/V2/3**             | As above          | As above   | As above  | No IR filter                  | Various | Night vision, IR light use cases             |

---

## 📊 OS Compatibility Matrix

| OS / Version                  | V1.3 | V2.x | Camera 3 | HQ Camera | Global Shutter |
|---------------------------------|-------|-------|----------|-----------|----------------|
| **Raspberry Pi OS**            | ✅     | ✅     | ✅        | ✅         | ✅              |
| **Ubuntu 20.04 (Server/Desktop)** | ⚠️ (manual config) | ⚠️ | ⚠️ | ⚠️ | ⚠️ |
| **Ubuntu 22.04 / 24.04**       | ⚠️ | ⚠️ | ⚠️ | ⚠️ | ⚠️ |
| **Other Linux distros**        | ⚠️ | ⚠️ | ⚠️ | ⚠️ | ⚠️ |

> ⚠️ **Note:** Non-Pi OS distros often require manual driver setup, config changes, and ensuring the kernel includes camera support.

---

## 🚀 Example Applications

- Time-lapse photography
- Object detection / computer vision (e.g. [[OpenCV]])
- Streaming / surveillance
- SLAM and robotics vision
- Drone / vehicle cameras

---

## 📌 Raspberry Pi Camera Pinout

The Raspberry Pi Camera Module connects to the Pi via the **CSI (Camera Serial Interface)** port using a flat **ribbon cable**. This interface is **dedicated** to camera communication and provides a **high-bandwidth connection** between the camera and the GPU of the Pi. The CSI connector on the Pi is typically a 15-pin MIPI CSI-2 FFC connector.

Here’s a breakdown of the main signal lines and their functions:

| Pin Label   | Description                                                                 |
|-------------|-----------------------------------------------------------------------------|
| **GND**     | Ground – Provides electrical ground for the circuit                         |
| **CAM_GPIO**| General-purpose input/output line from the camera (may signal frame start)  |
| **SCL0**    | I2C Clock line – Used to communicate with the camera's internal registers   |
| **SDA0**    | I2C Data line – Also for camera control (e.g. exposure, gain, etc.)         |
| **CLK**     | Clock – Supplies the pixel clock from the Pi to the camera sensor           |
| **CN/CP**   | Clock Negative / Clock Positive – Differential clock pair for data transfer |
| **DN0/DP0** | Data Lane 0 (Negative/Positive) – First differential data lane              |
| **DN1/DP1** | Data Lane 1 (Negative/Positive) – Second differential data lane (used in higher bandwidth cameras) |

### 🔍 Explanation

- **[[MIPI CSI-2 Protocol]]**: The camera uses the MIPI CSI-2 protocol over differential pairs for **high-speed data transmission**. CSI-2 lanes include a **clock pair (CN/CP)** and one or more **data pairs (DNx/DPx)**. Basic cameras use one lane, more advanced ones like the HQ or Camera v3 may use two lanes for higher bandwidth.
  
- **I2C (SCL0/SDA0)**: Even though the actual image data goes through CSI, the **control signals** like camera mode, exposure, gain, resolution, and frame rate are configured using I2C.

- **CAM_GPIO**: Some camera modules expose GPIO lines for functions such as signaling frame start (VSYNC-style signals) or global reset. These aren't always used but may be exposed for advanced use cases.

- **CLK**: Provides a reference clock to the camera sensor itself. Some modules use external oscillators and don’t need this supplied by the Pi.

Note: **Not all camera modules use all pins**. Some newer cameras, especially those with embedded ISP (like the Raspberry Pi Camera Module 3), rely more on internal logic and may use simplified signaling, while older models may need more active external control.

---

## 🏆 Strengths

- Small, low-power, affordable
- Excellent integration with Pi ecosystem
- Wide choice: fixed-focus, autofocus, IR, global shutter, interchangeable lens

---

## ⚠️ Weaknesses

- Compatibility issues on non-Pi OS
- Older modules (e.g. V1.3) increasingly unsupported in newer kernels
- No integrated microphone

---

## 🛠️ Notes for Ubuntu Users

- You often need to enable CSI support in `/boot/firmware/config.txt`:
```
start_x=1
gpu_mem=128
```

- Tools like `libcamera` are the modern supported stack.
- Legacy V4L2 drivers (`bcm2835-v4l2`) may be needed for older models.
- Some boards (e.g. Pi 3 B) + Ubuntu combos can require patched kernels or overlays.

---

## 🔗 Related Notes

- [[Raspberry Pi]]
- [[OpenCV]]
- [[sensor_msgs]]
- [[Point Cloud Algorithms]]
- [[Foxglove]]
- [[ROS2 Topics]]

---

## 🌐 External References

- [Official Raspberry Pi Cameras](https://www.raspberrypi.com/documentation/accessories/camera.html)
- [libcamera Raspberry Pi](https://www.raspberrypi.com/documentation/computers/camera_software.html)

---
