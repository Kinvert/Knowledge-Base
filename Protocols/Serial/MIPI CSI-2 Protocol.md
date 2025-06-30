# 🔌 MIPI CSI-2 Protocol

**MIPI CSI-2 (Mobile Industry Processor Interface – Camera Serial Interface 2)** is a high-speed, point-to-point serial interface standard designed to connect cameras to host processors, such as SoCs used in mobile phones, embedded systems, and single-board computers like the Raspberry Pi or Jetson Nano.

---

## 🧠 Summary

- CSI-2 is a widely adopted interface for camera modules.
- Designed by the **MIPI Alliance** for **low power, high bandwidth** communication.
- Commonly used in smartphones, embedded vision systems, and robotics.
- It transmits **uncompressed image data** from camera sensors to processors using **differential signaling**.
- Works in conjunction with the **MIPI D-PHY** or **C-PHY** physical layers.

---

## 🔧 Architecture Overview

| Component     | Description                                                                 |
|---------------|-----------------------------------------------------------------------------|
| **Camera Sensor** | Source of raw image data                                                 |
| **Serializer**    | Converts image data to CSI-2 packet format                              |
| **PHY Layer**     | Electrical interface (D-PHY or C-PHY) using differential pairs          |
| **Receiver (Host)** | SoC or processor that receives and processes CSI-2 data               |
| **Control Interface** | I²C interface for control and configuration of the camera           |

---

## 🔌 Physical Layer (D-PHY)

MIPI CSI-2 is usually implemented over **MIPI D-PHY**, a low-power differential signaling interface.

- **Clock Lane (CP/CN)**: One pair of differential wires
- **Data Lanes (DPx/DNx)**: One or more differential pairs
  - Typically 1–4 lanes
  - Each lane can operate up to 2.5 Gbps (some newer specs go even higher)

---

## 📦 Data Transmission

- Uses **packet-based transmission**:
  - Short packets: metadata (e.g. frame start, end)
  - Long packets: image data (e.g. RAW10, YUV422, RGB888)
- Data is **serialized** and transmitted in frames.
- **Lane Skew Calibration** ensures that bits from different lanes align properly.

---

## 🖧 Protocol Stack

| Layer           | Role                                                                 |
|------------------|----------------------------------------------------------------------|
| **Application Layer** | Camera sensor, generating raw image data                         |
| **CSI-2 Protocol Layer** | Converts image lines into packets                             |
| **PHY Protocol Interface (PPI)** | Bridges CSI-2 layer and PHY layer                      |
| **D-PHY Layer**         | Physical transmission of bits over differential pairs           |

---

## ⚙️ Features

- **High Bandwidth**: Supports up to several Gbps per lane.
- **Low Power**: Optimized for battery-powered systems.
- **Lane Scalability**: Use 1–4 data lanes based on bandwidth needs.
- **Error Detection**: Includes ECC and CRC for packet integrity.
- **Multiple Virtual Channels**: Enables multiple streams on a single physical link.

---

## 🔗 Common Image Formats

| Format   | Description                          |
|----------|--------------------------------------|
| RAW8/10/12/14 | Bayer pattern sensor data        |
| YUV422  | Color format, often used in preview   |
| RGB888  | Full-color image                      |
| JPEG    | Compressed format (not common in CSI-2) |

---

## 🛠️ Hardware Requirements

- CSI-compatible camera module (e.g. Raspberry Pi Camera v2, HQ Camera)
- CSI interface on host (e.g. Raspberry Pi CSI port, Jetson CAM port)
- Proper flat ribbon cable (15-pin, 22-pin, etc.)
- Software stack that supports MIPI CSI-2 (e.g. V4L2, libcamera, GStreamer)

---

## 🧪 Troubleshooting Tips

| Issue                      | Possible Causes                                                |
|----------------------------|----------------------------------------------------------------|
| Camera not detected        | Loose ribbon cable, improper CSI port usage                    |
| `start_x` not enabled      | On Raspberry Pi, `start_x=1` must be set in `config.txt`       |
| Blank image                | I2C misconfiguration or unsupported resolution                 |
| Kernel errors              | Driver issues, bad firmware, or incompatible module            |
| Frame drops                | Insufficient bandwidth or thermal throttling                   |

---

## 🔄 Comparison to Other Protocols

| Feature       | MIPI CSI-2         | USB 3.0            | Parallel Interface       | Ethernet (GigE Vision)   |
|---------------|--------------------|---------------------|--------------------------|---------------------------|
| Latency       | Low                | Medium              | Low                      | High                      |
| Power Usage   | Low                | Medium-High         | High                     | Medium                    |
| Bandwidth     | High               | Very High           | Low                      | High                      |
| Cable Length  | Short (cm–20 cm)   | Long (meters)       | Very Short               | Very Long (100+ m)        |
| Complexity    | Medium             | Low                 | High                     | High                      |
| Hot-Pluggable | No                 | Yes                 | No                       | Yes                       |

---

## 📦 Example Devices That Use CSI-2

- Raspberry Pi Camera Modules
- NVIDIA Jetson camera modules
- Intel RealSense (some variants)
- Mobile phone image sensors
- Various robotics and embedded vision modules

---

## 🧭 Related Notes

- [[Raspberry Pi Camera]]
- [[sensor_msgs]]
- [[V4L2]]
- [[Jetson Family]]
- [[CSI Interface]]
- [[I2C]]
- [[USB Cameras]]

---

## 🌐 External Links

- [MIPI Alliance – CSI-2 Overview](https://www.mipi.org/specifications/csi-2)
- [Jetson Camera Bring-Up Guide](https://docs.nvidia.com/jetson/)
- [Raspberry Pi CSI Camera Setup](https://www.raspberrypi.com/documentation/accessories/camera.html)

---
