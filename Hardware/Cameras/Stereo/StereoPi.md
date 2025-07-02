# 🔁 StereoPi

**StereoPi** is a Raspberry Pi-based open-source stereo camera system designed for 3D depth sensing, VR, robotics, and computer vision projects. It allows users to interface with two Raspberry Pi camera modules using the CSI interface for synchronized stereo imaging.

There are two main versions: **StereoPi v1** and **StereoPi v2**.

---

## 🧠 Summary

- DIY-friendly stereo camera setup built around Raspberry Pi Compute Modules.
- Provides access to raw stereo image data for custom [[Stereo Matching]] pipelines.
- Compatible with Raspberry Pi Camera modules (V1.3, V2, HQ).

---

## 🔢 Version Comparison: StereoPi v1 vs v2

| Feature                      | StereoPi v1                          | StereoPi v2                          |
|-----------------------------|--------------------------------------|--------------------------------------|
| Raspberry Pi CM Support     | CM3 / CM3+                           | CM4                                  |
| Camera Module Support       | V1.3 / V2                            | V1.3 / V2 / HQ                        |
| USB                         | USB 2.0                              | USB 3.0                              |
| Networking                  | 100 Mbps Ethernet                    | Gigabit Ethernet                     |
| Power Input                 | Micro-USB                            | USB-C                                |
| GPIO                        | Basic (40-pin GPIO)                  | Full access via breakout             |
| Form Factor                 | Standard 65x56mm                     | Slim, compact board or dev board kit |
| Expansion                   | Limited                              | PCIe (via CM4), M.2 (dev board only) |
| Price Range (board only)    | ~$50–60                              | ~$60–75                              |

> Note: StereoPi v2 Dev Kit includes optional M.2 slot, RTC battery, and extended breakout features.

---

## 📊 StereoPi vs Similar Hobbyist Stereo Cameras

| Feature             | StereoPi v2        | Intel RealSense D435     | Luxonis OAK-D Lite       | ZED Mini               |
|--------------------|--------------------|---------------------------|---------------------------|------------------------|
| DIY Flexibility     | ✅ High              | ❌ Fixed hardware           | ⚠️ Medium                 | ❌ Fixed hardware       |
| Camera Swappable    | ✅ Yes               | ❌ No                      | ❌ No                      | ❌ No                   |
| Depth Processing    | ❌ Manual (external)| ✅ Onboard                 | ✅ Onboard AI/Depth       | ✅ Onboard              |
| Price               | ~$60–75 (board)    | ~$200–250                 | ~$150                     | ~$350–450              |
| Interface           | CSI + USB3 (CM4)   | USB 3.0                   | USB-C                     | USB 3.0                |
| SDK/Software        | OpenCV, ROS, etc.  | RealSense SDK, ROS        | DepthAI, ROS              | ZED SDK, ROS           |
| Host Required       | ✅ Yes (CM4)         | ✅ Yes                     | ✅ Yes                     | ✅ Yes                  |
| Depth Quality       | ⚠️ Depends on algos | ✅ High                    | ✅ Medium                 | ✅ High                 |

---

## 🔧 Pros and Cons of StereoPi

| Pros                                                | Cons                                                  |
|-----------------------------------------------------|-------------------------------------------------------|
| Swappable Raspberry Pi cameras (V1.3, V2, HQ)        | Requires external depth computation                   |
| Full access to GPIO and Raspberry Pi ecosystem       | Steeper learning curve than plug-and-play systems     |
| Open-source and highly customizable                  | Performance depends on chosen Compute Module and OS   |
| Affordable and modular                               | No onboard IMU or AI acceleration (unless added)      |
| Works with [[OpenCV]], [[ROS]], and SLAM toolchains  | Depth accuracy dependent on user algorithm setup      |

---

## 🧰 Use Cases

- DIY stereo vision projects
- Robotics vision pipelines
- Educational computer vision platforms
- Custom VR/AR applications
- Mobile robots or drones with edge compute

---

## 🔗 Related Notes

- [[Stereo Cameras]]
- [[Depth Estimation]]
- [[Raspberry Pi]]
- [[Point Cloud Algorithms]]

---

## 🌐 External Resources

- [StereoPi Official](https://stereopi.com/)
- [StereoPi GitHub](https://github.com/realizator/stereopi-wiki)
- [StereoPi v2 Docs](https://wiki.stereopi.com/)

---
