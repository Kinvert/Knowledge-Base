# 🎥 Arducam Multi Camera Adapter Module V2.2

The **Arducam Multi Camera Adapter Module V2.2** is a breakout board designed to allow a Raspberry Pi (or other compatible boards) to connect **up to four camera modules** using the **CSI-2 interface**. This solution is particularly useful for embedded applications that need multiple image inputs, such as robotics, security systems, stereo vision, or panoramic capture, without needing multiple SBCs.

---

## 🧠 Summary

- Allows switching between multiple cameras via **I2C** commands.
- Only one camera can be active at a time due to the CSI-2 bandwidth limitation.
- Compatible with **Raspberry Pi Camera Module V1/V2** and **some Arducam cameras**.
- Based on the **TCA9548A I2C multiplexer** to handle camera communication.

---

## ⚙️ Key Features

- Supports up to 4 cameras per adapter.
- Uses GPIO and I2C to control camera selection.
- Stacks multiple boards (theoretically 2 or more for 8+ cameras).
- Compact form factor.
- Works with both **Raspberry Pi 4/3/Zero** and **Jetson Nano**.

---

## ⚠️ Limitations

- **Only one camera can be used at a time.** No simultaneous capture across channels.
- Image switching may introduce a few hundred milliseconds of delay.
- Not compatible with all Pi camera types (some HQ or global shutter models may not work reliably).
- Requires software changes to handle camera switching logic in Python/C++/Bash.

---

## 🆚 Comparison with Other Multi-Camera Methods

| Method                                    | Simultaneous Capture | Max Cameras | Cost   | Notes                                                |
|------------------------------------------|----------------------|-------------|--------|------------------------------------------------------|
| **Arducam Multi Camera Adapter V2.2**    | ❌ (one at a time)   | 4 per board | ~$25   | Cheapest option, needs switching logic               |
| **Arducam CSI-to-HDMI Quad Sync**        | ✅ (synchronized)    | 4           | $$$$   | Expensive, used for synchronized CSI capture         |
| **StereoPi**                              | ✅ (2 cameras)       | 2           | ~$60+  | Only 2 cameras, meant for stereo depth apps          |
| **USB Cameras (via USB hub)**            | ⚠️ (depends)         | 3–4+        | Varies | Depends on bandwidth and CPU, often less performant  |
| **Raspberry Pi Compute Module + Carrier**| ✅ (CM4 + CSI lanes) | 2–4         | $$$    | CM4 supports multiple CSI-2 lanes; better flexibility|
| **Jetson Xavier NX**                     | ✅                   | 3+          | $$$$   | Industrial use, supports multiple CSI/USB cameras    |
| **MIPI CSI Multiplexer Chips (custom)**  | ✅ (advanced use)    | 4–8         | $$$+   | Very complex, often used in custom boards            |

---

## 📌 Typical Use Cases

- Sequential image capture from multiple angles.
- DIY security/surveillance systems (non-real-time).
- Embedded robotics vision projects on a budget.
- Entry-level multi-camera time-sharing systems.

---

## ✅ Pros

- Affordable, easy to source.
- Plug-and-play with existing Raspberry Pi ecosystem.
- Great for prototypes and light applications.

---

## ❌ Cons

- No concurrent image capture (CSI-2 limitation).
- Software complexity increases with camera switching logic.
- Can be finicky with some camera types and Pi models.

---

## 🔗 Related Notes

- [[Raspberry Pi]]
- [[StereoPi]]
- [[Stereo Cameras]]
- [[MIPI CSI-2 Protocol]]

---

## 🌐 External Links

- [Official Product Page (Arducam)](https://www.arducam.com/product/multi-camera-adapter-module-v2-2/)
- [Setup Guide & Docs](https://github.com/ArduCAM/MultiCameraAdapter)

---
