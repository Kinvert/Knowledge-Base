# 📸 Global Shutter

**Global Shutter** is a camera sensor technology where all pixels in the sensor array are exposed to light **simultaneously** and then read out, as opposed to **rolling shutter**, which reads out rows sequentially. This technology is essential in applications requiring accurate motion capture without distortion.

---

## 🧠 Summary

- All pixels are exposed at the **same instant**.
- Prevents **motion artifacts** like skew, wobble, or partial exposure (seen in rolling shutter).
- Crucial in high-speed imaging, robotics, machine vision, drones, and SLAM.

---

## ⚙️ How It Works

- The sensor accumulates light across all pixels at the same time.
- After exposure, the data is transferred to a storage register and read out.
- Can use a **mechanical** or **electronic** implementation.
- Requires **dedicated hardware** per pixel for simultaneous capture and buffering, which increases cost and power.

---

## 🚀 Use Cases

| Domain            | Examples                                              |
|------------------|--------------------------------------------------------|
| Robotics / SLAM  | Precise frame-to-frame motion estimation, e.g. [[ORB-SLAM]] |
| Drones            | Aerial mapping and tracking fast motion               |
| Machine Vision    | Industrial automation, barcode scanning               |
| AR / VR           | Low-latency head tracking                             |
| Autonomous Cars   | Object tracking, LiDAR-camera synchronization         |
| High-Speed Video  | Motion analysis in sports, research                   |

---

## 🆚 Global vs. Rolling Shutter

| Feature                     | Global Shutter                   | Rolling Shutter                     |
|----------------------------|----------------------------------|-------------------------------------|
| Exposure Method            | All pixels at once               | Row-by-row sequential               |
| Motion Distortion          | ❌ None                          | ✅ Skew, wobble, smear possible     |
| Cost & Complexity          | ⬆️ Higher                       | ⬇️ Lower                            |
| Power Consumption          | ⬆️ Typically higher              | ⬇️ Lower                            |
| Ideal for                  | Fast motion, SLAM, precision     | Still photography, budget sensors  |

---

## 🎯 Common Global Shutter Sensors

| Sensor Name   | Resolution | Interface     | Notes                                   |
|---------------|------------|---------------|-----------------------------------------|
| `OV9281`      | 1280×800   | MIPI CSI-2     | Monochrome, popular in stereo setups    |
| `OV2311`      | 1600×1300  | MIPI CSI-2     | High-res mono sensor                    |
| `IMX296`      | 1280×1024  | MIPI / Parallel| Industrial-grade, precise timing        |
| `AR0144`      | 1280×800   | MIPI           | Low power, good for embedded devices    |
| `PYTHON1300`  | 1280×1024  | Parallel       | Used in scientific cameras              |

---

## 📷 Stereo and Multi-Camera Systems

Global shutter is often used in systems like:
- [[Arducam Camarray]] with `OV9281`
- [[OV9281 Global Shutter Mono Stereo]]
- [[StereoPi]] (with appropriate sensors)
- [[e-Con Systems Tara.X]]
- [[Luxonis OAK-D]]

These systems require **precise synchronization** and **no distortion**, which global shutter provides.

---

## 🧪 Compatible Boards

| Platform         | Notes                                                    |
|------------------|----------------------------------------------------------|
| Raspberry Pi 4   | Needs compatible overlays/drivers for global shutter |
| Jetson Nano      | Supported with vendor drivers                         |
| Jetson Orin Nano | Higher performance, multiple CSI lanes                |
| [[StereoPi]]         | Stereo support when sensors are matched               |

---

## ✅ Pros

- ✅ Perfect motion capture without skew
- ✅ Ideal for robotics and SLAM
- ✅ Excellent for synchronization with other sensors (e.g., IMU, LiDAR)

---

## ⚠️ Cons

- ❌ More expensive than rolling shutter
- ❌ Slightly more complex driver/software support
- ❌ Limited color options for some modules

---

## 🔗 Related Notes

- [[Rolling Shutter]]
- [[Camera Sensor Formats]]
- [[OV9281 Global Shutter Mono Stereo]]
- [[Stereo Cameras]]
- [[Arducam Camarray]]

---

## 🌐 External References

- [Arducam Global Shutter Collection](https://www.arducam.com/product-category/camera-modules/global-shutter-cameras/)
- [Sony Global Shutter Sensor Lineup](https://www.sony-semicon.co.jp/e/products/IS/sensor1/index.html)
- [What Is a Global Shutter? - Basler](https://www.baslerweb.com/en/products/technology/global-shutter/)

---
