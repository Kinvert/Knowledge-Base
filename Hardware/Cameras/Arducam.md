# 🔧 Arducam

**Arducam** is a leading hardware provider specializing in **camera modules**, **adapters**, and **camera accessories** for platforms like [[Raspberry Pi]], [[Jetson Family]], [[Arduino]], and others. They are especially well known in the **robotics**, **computer vision**, and **embedded systems** communities for making a wide variety of **plug-and-play**, **customizable**, and **high-performance** camera solutions.

This document breaks down Arducam’s product offerings by category, highlighting key modules, distinguishing features, and use cases—especially for **robotics**, **SLAM**, **stereo vision**, and **depth estimation**.

---

## 📦 Categories & Product Lines

---

## 🧠 1. Camera Modules

These are ready-to-use camera boards with sensors mounted and integrated, often supporting standard ribbon connectors (MIPI CSI-2) or USB.

### 🎥 MIPI CSI-2 Camera Modules

| Sensor     | Shutter | Mono/Color | Notes                                              |
|------------|---------|------------|----------------------------------------------------|
| OV9281     | Global  | Mono       | Fast FPS, stereo support, widely used in SLAM      |
| IMX219     | Rolling | Color      | Raspberry Pi Cam V2 equivalent                     |
| IMX477     | Rolling | Color      | High-quality, interchangeable lens, HQ Pi cam      |
| IMX708     | Rolling | Color      | Pi Cam V3 equivalent                               |
| IMX296     | Global  | Mono       | Used in industrial, high precision                 |
| OV2311     | Global  | Mono       | High-res mono stereo support                       |
| IMX290     | Rolling | Mono/Color | Excellent low-light, popular in AI vision setups   |
| IMX462     | Rolling | Color      | Superb night vision with IR                        |

- Many modules are **drop-in replacements** for Raspberry Pi and Jetson camera slots.
- Some include **lens mounts** (M12, C-mount) for **replaceable lenses**.

### 🔌 USB Camera Modules

| Product                     | Sensor     | Notes                                              |
|-----------------------------|------------|----------------------------------------------------|
| Arducam USB3 Global Shutter| OV9281     | Stereo-ready with board sync                       |
| Arducam USB2 Mini Series   | OV5647, IMX219 | Great for single-camera testing, plug-and-play |
| Arducam UVC Camera Series  | Various    | Standard USB Video Class compliant modules         |

- USB interface avoids MIPI driver headaches.
- Good for non-Linux OS or fast prototyping.

---

## 👓 2. Stereo Camera Modules

These modules have **dual image sensors** either side-by-side or tightly synced, ideal for stereo depth estimation and SLAM.

| Name / Series               | Sensors        | Notes                                                   |
|-----------------------------|----------------|----------------------------------------------------------|
| OV9281 Stereo Camera        | OV9281 x2      | Global shutter mono, MIPI CSI-2, stereo virtual channel |
| USB3 Stereo Global Shutter | OV9281 or OV2311 | High FPS, synced, works with Jetson/RPi via USB        |
| Arducam TOF Stereo Module   | Proprietary    | IR-based depth, integrated processing                   |

- Some support **hardware synchronization** (triggering both sensors in lockstep).
- Often rely on **virtual channel** MIPI CSI-2 extensions or custom drivers.

---

## 🔁 3. Multi-Camera Adapters

For platforms with only **one CSI-2 interface**, Arducam provides adapters to connect **multiple cameras**.

| Name                      | Cameras Supported | Method                      | Notes                                                |
|---------------------------|-------------------|-----------------------------|------------------------------------------------------|
| Multi Camera Adapter V2.2 | Up to 4x IMX219/477 | I2C multiplexing + GPIO sync| Compatible with Raspberry Pi                         |
| Camarray HAT              | 2x OV9281          | MIPI virtual channel        | For stereo and SLAM; requires compatible software    |
| Quad Sync HAT             | 4x IMX219/477      | MIPI + sync signal support  | For multi-cam vision and alignment                   |

- Helpful in robotics when multiple viewpoints are needed but board slots are limited.
- Sync capability varies by model.

---

## 🔬 4. Lens and Optics Options

Arducam provides camera modules with:

- **Fixed Focus**
- **Motorized Focus** (Autofocus)
- **Replaceable Lenses**: via
  - **M12 mounts**
  - **CS/C-mounts**
- **Fisheye Lenses** for wide-angle vision
- **IR Cut Filters** / **IR Pass Filters**

This enables use in a variety of lighting and application conditions: night vision, close-range inspection, outdoor mapping, etc.

---

## 📋 5. Dev Kits & Bundles

| Name                    | Includes                         | Notes                                       |
|-------------------------|----------------------------------|---------------------------------------------|
| Arducam Stereo Kit      | 2x OV9281 + HAT + ribbon cables  | Used for DIY stereo depth camera            |
| Arducam Camarray DevKit | Stereo/Quad camera + controller  | Complete kit for synchronized vision input  |
| Pi-Compatible Kits      | Sensor + Ribbon + Focus Control  | Plug-and-play for Pi / Jetson platforms     |

These kits simplify setup and integration for users in education, research, and robotics startups.

---

## 🧠 Common Use Cases

- SLAM (Simultaneous Localization and Mapping)
- [[Stereo Cameras]] and Depth Estimation
- Object Detection and Visual Odometry
- Real-time robotic navigation and computer vision
- Precision agriculture, drone imaging, warehouse automation

---

## 📎 Related Notes

- [[Stereo Cameras]]
- [[Camera Sensor Formats]]
- [[Global Shutter]]
- [[Raspberry Pi Camera]]
- [[Jetson Family]]
- [[Arducam Multi Camera Adapter Module V2.2]]
- [[OV9281 Global Shutter Mono Stereo]]

---

## 🌐 External Resources

- [https://www.arducam.com](https://www.arducam.com)
- [Arducam Store](https://www.arducam.com/product-category/)

---
