# 📸 Cameras

**Cameras** are essential components in robotics, embedded systems, computer vision, and surveillance. They convert light (or other forms of radiation like infrared) into electronic signals for processing or storage. This note provides a high-level overview of camera types, uses, protocols, and links to more specific entries.

---

## 📚 Categories of Cameras

| Type              | Description                                                                |
|-------------------|----------------------------------------------------------------------------|
| [[Visible Light Cameras]] | Standard cameras capturing the visible spectrum, often used with OpenCV or ROS |
| [[Infrared Cameras]]     | Capture thermal radiation (heat), often used in dark or smoky environments |
| [[Depth Cameras]]        | Measure distance to objects (e.g., stereo, ToF, LiDAR-based)          |
| [[High-Speed Cameras]]   | Capture motion at very high frame rates, used in analysis and robotics |
| [[Global Shutter Cameras]] | Reduce motion blur; essential for fast-moving scenes                 |
| [[Rolling Shutter Cameras]] | Common and cheaper; subject to distortion during rapid motion        |

---

## 🔌 Interfaces and Protocols

| Protocol / Interface | Description                                                        |
|-----------------------|--------------------------------------------------------------------|
| [[MIPI CSI-2 Protocol]] | High-speed serial interface for connecting cameras to SBCs         |
| [[USB Protocol]]      | Widely used for webcams, depth cams (e.g., Intel RealSense)        |
| [[Ethernet]]          | Used in industrial systems (e.g., GigE Vision)                     |
| [[UART]]              | Lightweight interface for thermal or embedded cameras              |
| HDMI / Analog         | For traditional or FPV camera setups                               |

---

## 🛠️ Integration in Projects

- **Embedded Systems**: Raspberry Pi, Jetson Nano, and other SBCs support cameras via MIPI CSI-2 or USB.
- **Robotics**: ROS2 nodes often use `sensor_msgs/Image` for camera streams.
- **FPV / Drones**: Cameras used with analog video transmitters or digital HD setups.
- **Thermal Monitoring**: Use thermal cameras like [[FLIR Lepton]] or [[Caddx Infra V2]].

---

## 🔗 Related Notes

- [[Raspberry Pi Camera]]
- [[Thermal Cameras]]
- [[MIPI CSI-2 Protocol]]
- [[sensor_msgs]]
- [[OpenCV]]
- [[Jetson Nano]]
- [[Depth Cameras]]
- [[ROS2 Topics]]

---

## 📦 Common Camera Modules

| Module                  | Type            | Interface         | Use Case                          |
|-------------------------|-----------------|-------------------|-----------------------------------|
| Raspberry Pi V2         | Visible light   | MIPI CSI-2        | General purpose SBC camera        |
| Raspberry Pi HQ         | Visible light   | MIPI CSI-2        | High resolution and lens swap     |
| FLIR Lepton 3.5         | Thermal         | SPI               | Thermal detection for hobbyists   |
| Caddx Vista / Nebula    | FPV/Low-latency | Digital / Analog  | FPV drones                        |
| Intel RealSense D435i   | Depth / RGB     | USB               | Robotics / 3D vision              |
| Arducam IMX219/IMX477   | Visible light   | MIPI CSI-2        | Custom camera modules for Pi      |

---
