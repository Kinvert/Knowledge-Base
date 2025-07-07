# 🎯 Arducam Camarray

**Arducam Camarray** is a powerful multi-camera solution from [[Arducam]], designed to allow multiple synchronized camera modules—especially **stereo** or **quad camera setups**—to connect to embedded systems like the [[Raspberry Pi]], [[Jetson Family]], and other platforms with limited CSI-2 lanes.

Camarray modules are ideal for applications involving **stereo vision**, **depth sensing**, **SLAM**, **AR/VR**, and **multi-view capture**. Unlike traditional multi-camera adapters which use I2C tricks or GPIO multiplexing, **Camarray uses MIPI CSI-2 virtual channels** and specialized hardware for **frame-level synchronization**.

---

## 🧠 Summary

- Multi-camera interface solution using MIPI CSI-2 virtual channel support.
- Allows 2–4 cameras to be interfaced over a **single CSI-2 ribbon cable**.
- Supports **global shutter** and **rolling shutter** sensors.
- Primarily designed for **stereo cameras**, but expandable to more.
- Enables **hardware synchronization** of cameras at the exposure/frame level.
- Compatible with select Raspberry Pi and Jetson boards with modified drivers.

---

## 🔌 Inputs and Outputs

### 🎥 Camera Inputs

Camarray boards support **up to 4 sensors** of the same type, connected via:
- Flat flex cable (FFC) from cameras to the Camarray board.
- Supported sensors include:
  - `OV9281` (Global Shutter, Monochrome)
  - `IMX219` (Rolling Shutter, Color)
  - `IMX477` (High-Res Rolling Shutter)
  - `OV2311`, `IMX296`, `OV7251` (others depending on variant)

> Cameras must generally match resolution, framerate, and format.

### 📤 Output to Host

- **Single MIPI CSI-2 ribbon** to host board (e.g., Pi/Jetson)
- CSI-2 lanes multiplexed using **Virtual Channel Indexing**
- Also supports:
  - **Trigger I/O** (for hardware sync and external strobe)
  - **GPIO** for custom camera timing/sync (on some models)

---

## 🧩 Compatible Boards

| Platform         | Compatible? | Notes                                                                 |
|------------------|-------------|-----------------------------------------------------------------------|
| Raspberry Pi 4   | ✅           | With custom overlays and drivers. Some limitations with VC support.  |
| Raspberry Pi 5   | ⚠️           | Driver support still evolving; dual CSI slots offer alternatives     |
| Jetson Nano      | ✅           | Supports VC-based multi-camera setups with Arducam drivers           |
| Jetson Orin Nano | ✅           | Newer Jetpack versions support VC with patches                       |
| Jetson Xavier NX | ✅           | Excellent support, high bandwidth and CSI lanes                      |
| Jetson AGX Orin  | ✅           | Advanced support for quad-sensor arrays                             |

---

## 🧠 Shutter Support

| Sensor     | Shutter Type | Stereo Sync? | Notes                                      |
|------------|---------------|--------------|--------------------------------------------|
| [[OV9281]]     | Global        | ✅            | Preferred for SLAM, avoids motion blur     |
| OV2311     | Global        | ✅            | High-res mono stereo vision                |
| IMX219     | Rolling       | ⚠️            | Possible but suffers from skew artifacts   |
| IMX477     | Rolling       | ⚠️            | Higher res but not ideal for fast motion   |
| IMX296     | Global        | ✅            | Industrial-grade sensor                    |

- Global shutter is highly preferred for motion applications like SLAM, drones, and robotics.

---

## 🔁 Synchronization

- Camarray offers **frame-level synchronization** using:
  - Shared trigger signals
  - Synchronized clock lines
  - MIPI virtual channel data separation
- **Hardware synchronization** is built-in for global shutter sensors.
- For rolling shutter, synchronization is more limited and may cause skew between frames.

---

## 🧪 Use Cases

- [[Stereo Vision]]
- Depth Estimation
- SLAM
- Multi-angle object detection and tracking
- Robotics / AR / MR perception
- Embedded stereo cameras for inspection

---

## 📈 Comparison to Similar Products

| Product                        | Max Cameras | Interface         | Sync Support | Shutter Type     | Notes                                  |
|-------------------------------|-------------|-------------------|--------------|------------------|----------------------------------------|
| Arducam Camarray              | 2–4         | MIPI CSI-2 (VC)   | ✅ Hardware   | Global / Rolling | High performance, small footprint      |
| [[Arducam Multi Camera Adapter Module V2.2]] | 4           | I2C Mux + GPIO    | ⚠️ Software    | Rolling only     | Good for low-end prototyping           |
| e-Con Systems Tara.X          | 2           | USB3              | ✅ Hardware   | Global           | More expensive, plug-and-play          |
| StereoPi V2                   | 2           | Dual CSI          | ⚠️ GPIO sync | Rolling / Global | Raspberry Pi form factor stereo cam    |
| Luxonis OAK-D                 | 2–3         | USB3 + MIPI       | ✅ Internal   | Global + Depth   | Includes onboard AI                    |

---

## 📦 Product Variants

- **Camarray HAT for Pi** – 2x or 4x camera support, MIPI VC, fits Pi GPIO
- **Camarray Dev Kits** – Bundled sensors + controller, ready for stereo vision
- **Camarray USB Sync Hub** – Adds USB interface for stereo setups without CSI

---

## 🔗 Related Notes

- [[Stereo Cameras]]
- [[Global Shutter]]
- [[OV9281 Global Shutter Mono Stereo]]
- [[Camera Sensor Formats]]
- [[Jetson Family]]
- [[Arducam]]

---

## 🌐 External Resources

- [Arducam Camarray Landing Page](https://www.arducam.com/camarray/)
- [Camarray Dev Kits](https://www.arducam.com/product-category/multi-camera-solutions/)
- [GitHub – Arducam Drivers and Overlays](https://github.com/ArduCAM)
- https://docs.arducam.com/Raspberry-Pi-Camera/Multi-Camera-CamArray/Multi-Camera-CamArray/#variable-baseline-version-camera-kit
- https://www.uctronics.com/arducam-1mp-2-stereoscopic-camera-bundle-kit-for-raspberry-pi-nvidia-jetson-nano-xavier-nx-two-ov9281-global-shutter-monochrome-camera-modules-and-camarray-stereo-camera-hat.html
- https://docs.arducam.com/Nvidia-Jetson-Camera/Multi-Camera-CamArray/quick-start/#imx519-stereo-camera-kit
- https://www.robotshop.com/products/arducam-stereoscopic-camera-kit-w-1mp-ov9281-2x-camarray-stereo-camera-hat

---
