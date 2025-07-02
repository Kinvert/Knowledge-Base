# 🎥 Waveshare Binocular Camera Module (IMX219-83)

The **Waveshare Binocular Camera Module** is a dual‑lens stereo imaging system built around **two Sony IMX219 (8 MP)** sensors. It is designed for **high-resolution depth sensing** and stereo vision projects using **Raspberry Pi CM3/CM4/CM5**, **Jetson Nano/Xavier/Orin**, and similar platforms :contentReference[oaicite:0]{index=0}.

---

## 📌 Key Specifications

| Feature               | Details                                                                 |
|----------------------|-------------------------------------------------------------------------|
| **Sensors**           | 2 × Sony IMX219 (8 MP, 3280 × 2464) :contentReference[oaicite:1]{index=1}                 |
| **Focal Length**      | 2.6 mm (aperture F/2.4) :contentReference[oaicite:2]{index=2}                              |
| **Field of View**     | 83° diagonal (73° H / 50° V) :contentReference[oaicite:3]{index=3}                          |
| **Baseline**          | ~60 mm (stereo camera separation) :contentReference[oaicite:4]{index=4}                    |
| **IMU**               | Onboard ICM20948 (9‑axis accel/gyro/magnetometer) :contentReference[oaicite:5]{index=5}     |
| **Interface**         | Dual 15‑pin MIPI CSI‑2 to host board via supplied FFC cable             |
| **Power/I²C**         | Camera powered by CSI interface; IMU uses I²C (SDA/SCL) :contentReference[oaicite:6]{index=6} |
| **Dimensions**        | 24 × 85 mm :contentReference[oaicite:7]{index=7}                                            |

---

## ✅ Pros & ⚠️ Cons

### ✅ Pros
- High-resolution stereo at an affordable price (~$45–$60) :contentReference[oaicite:8]{index=8}
- Includes IMU for inertial-aided computation
- Compatible with various CSI-capable platforms

### ⚠️ Cons
- Incompatible with classic Raspberry Pi 4/Zero CSI ports—requires **Compute Module** or Jetson :contentReference[oaicite:9]{index=9}
- Requires external stereo calibration and depth processing pipeline
- Basic MIPI compatibility—adjust settings using `libcamera` tuning if color isn't accurate :contentReference[oaicite:10]{index=10}

---

## 🔄 vs Similar Stereo Solutions

| Feature                    | Waveshare IMX219‑83         | StereoPi v2                  | Intel RealSense D435       | Luxonis OAK‑D Lite         |
|----------------------------|-----------------------------|-----------------------------|-----------------------------|-----------------------------|
| **Resolution**             | 8 MP ×2                     | Depends on Pi cam           | 720p (depth) + RGB         | 1280×800 depth + AI        |
| **Interface**              | Dual CSI-2 (Compute Module) | CSI-2 via CM4                | USB 3.0                    | USB-C                      |
| **Depth Processing**       | User-implemented            | User-implemented            | Onboard RealSense SDK      | Onboard DepthAI pipelines  |
| **IMU**                    | Yes (ICM20948)              | Optional                    | No                         | No                         |
| **Price**                  | ~$50                        | ~$60+ (with CM4)            | ~$200–250                   | ~$150                      |
| **Ease of Use**            | Medium (calibration needed) | Medium (DIY)                | High (SDK pipelines)       | High (SDK + AI)            |
| **Host Requirements**      | Compute Module or Jetson    | CM4                        | Any USB-capable PC/SBC     | Any USB-C-capable host     |

---

## 🧰 Typical Use Cases

- DIY **dense depth mapping** on embedded platforms with depth or stereo SLAM
- **Robot stereo vision** combining depth with inertial sensing
- **Research projects**, where AI perception is offloaded to platforms like Jetson
- **Panoramic capture systems** (multi-camera stitching)

---

## 🔧 Setup Tips

1. Connect both CSI cables to the Compute Module or Jetson board.
2. Enable `libcamera` on Pi or Jetson CSI interface.
3. Configure tuning files for accurate color and image quality :contentReference[oaicite:11]{index=11}.
4. Calibrate stereo pair using OpenCV’s `stereoCalibrate()`.
5. Access IMU via `/dev/i2c` SDA/SCL lines.
6. Test streams with:
   ```bash
   libcamera-hello --camera 0
   libcamera-hello --camera 1

---

## 🔗 Related Notes

- [[Stereo Cameras]]
