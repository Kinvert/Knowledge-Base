# FSIN (Frame Sync)

**FSIN (Frame Sync)** is a hardware signal used to synchronize the capture of frames across multiple image sensors or devices. In robotics and computer vision systems, FSIN ensures that multiple cameras capture images simultaneously, which is essential for stereo vision, multi-camera arrays, and time-sensitive perception tasks.

---

## ⚙️ Overview

The FSIN pin (sometimes called **Frame Sync In**) is typically found on image sensors and camera modules. When an external trigger pulse is applied, the sensor aligns the start of its frame capture to the incoming signal. This allows for deterministic synchronization across multiple sensors, often paired with a corresponding **FSOUT (Frame Sync Out)** signal for daisy-chaining.

---

## 🧠 Core Concepts

- **Synchronization**: Ensures all cameras capture frames at the same time.  
- **Trigger Pulse**: External signal (often a square wave) that initiates frame capture.  
- **Master/Slave Configurations**: One device outputs FSOUT, others accept FSIN.  
- **Latency Reduction**: Critical in stereo depth estimation and SLAM.  
- **Deterministic Timing**: Avoids rolling shutter artifacts between unsynchronized cameras.  

---

## 📊 Comparison Chart

| Feature / Aspect         | FSIN (Frame Sync) | Software Sync | PTP (Precision Time Protocol) | Genlock | Free-Running |
|--------------------------|-------------------|---------------|-------------------------------|---------|--------------|
| Synchronization Accuracy | ✅ High           | ❌ Low        | ✅ Medium-High                | ✅ High | ❌ None      |
| Hardware Support Needed  | ✅ Yes            | ❌ No         | ✅ Yes                        | ✅ Yes  | ❌ No        |
| Latency                  | ✅ Minimal        | ❌ Variable   | ✅ Moderate                   | ✅ Low  | ❌ High      |
| Typical Use in Robotics  | ✅ Stereo Vision  | ❌ Rare       | ✅ Distributed Systems        | ✅ Video Systems | ✅ Simple Sensors |
| Scalability              | ✅ Moderate       | ✅ Easy       | ✅ High                       | ✅ Moderate | ✅ Easy    |

---

## 🔧 Use Cases

- Stereo vision in autonomous vehicles and robots  
- Multi-camera arrays (e.g., 360° vision systems)  
- Time-aligned video capture for SLAM and VIO (Visual-Inertial Odometry)  
- Structured light or ToF systems requiring synchronized frames  
- Robotics applications needing deterministic image acquisition  

---

## ✅ Strengths

- Extremely precise synchronization across devices  
- Low jitter, critical for 3D reconstruction  
- Hardware-level control independent of software stack  
- Works well in master-slave configurations  

---

## ❌ Weaknesses

- Requires dedicated hardware support (pins and drivers)  
- Limited scalability compared to network-based sync methods  
- Adds wiring complexity  
- Not universally supported by all sensors  

---

## 🛠️ Compatible Items

- [[OV9281]] (common global shutter sensor with FSIN support)  
- [[Arducam CamArray]]  
- [[FPGA]] (for timing distribution)  
- [[Jetson Family]] (via GPIO for triggering)  
- [[Microcontrollers]] (as signal generators)  

---

## 📚 Related Concepts

- [[Global Shutter]]  
- [[Rolling Shutter]]  
- [[PTP]] (Precision Time Protocol)  
- [[Clock Distribution]]  
- [[Time Synchronization]]  

---

## 🌐 External Resources

- Arducam FSIN documentation  
- Sony IMX series datasheets with FSIN support  
- NVIDIA Jetson developer guides on camera synchronization  

---

## 📝 Summary

FSIN provides deterministic, hardware-level synchronization for multi-camera setups. It is essential for stereo vision, SLAM, and high-precision robotic vision tasks where microsecond-level alignment of frames directly impacts system performance.
