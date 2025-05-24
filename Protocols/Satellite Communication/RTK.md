---
title: RTK (Real-Time Kinematic)
tags: [gps, gnss, positioning, navigation, rtk, satellite, high-precision, protocols]
aliases: [Real-Time Kinematic, RTK GPS, RTK GNSS]
---

# 📡 RTK (Real-Time Kinematic)

## 🧭 Overview

**RTK (Real-Time Kinematic)** is a high-precision satellite navigation technique that enhances the accuracy of GNSS (Global Navigation Satellite System) positioning to centimeter-level by using real-time correction data from a reference station. RTK is widely used in surveying, precision agriculture, robotics, autonomous vehicles, and any application where sub-meter or centimeter-level positioning is required.

RTK works by transmitting correction data from a fixed base station (with a known position) to one or more mobile receivers (rovers), allowing them to correct for errors in satellite signals such as atmospheric delays and satellite clock drift.

---

## 🛠️ Key Features

- **Centimeter-Level Accuracy**: Achieves 1–2 cm horizontal accuracy under ideal conditions.
- **Real-Time Corrections**: Provides instant position updates with corrections applied.
- **Multi-Constellation Support**: Works with GPS, GLONASS, Galileo, BeiDou, and other GNSS systems.
- **Base and Rover Architecture**: Requires a fixed base station and one or more rovers.
- **Low Latency**: Suitable for real-time applications like autonomous navigation and machine control.
- **Correction Data Formats**: Commonly uses RTCM (Radio Technical Commission for Maritime Services) or proprietary protocols.

---

## 📦 Common Use Cases

- **Land Surveying**: High-precision mapping and boundary determination.
- **Precision Agriculture**: Automated tractor guidance, planting, and spraying.
- **Construction**: Machine control, site layout, and grading.
- **Robotics and Drones**: Accurate localization for autonomous navigation.
- **Marine Navigation**: Harbor operations, dredging, and hydrographic surveys.
- **Asset Tracking**: High-accuracy tracking of vehicles and equipment.

---

## 🛠️ How RTK Works

1. **Base Station Setup**:  
   - A GNSS receiver is placed at a known, fixed location (the base station).
2. **Correction Data Generation**:  
   - The base station calculates the difference between its known position and the position calculated from satellite signals.
3. **Correction Data Transmission**:  
   - The base station transmits correction data to rovers via radio, cellular, or internet (NTRIP).
4. **Rover Positioning**:  
   - The rover receives both satellite signals and correction data, applying the corrections to achieve high-precision positioning.

---

## 🆚 RTK vs. Other GNSS Techniques

| Feature                | Standard GNSS      | D-GPS                | RTK                   | PPP (Precise Point Positioning) |
|------------------------|--------------------|----------------------|-----------------------|-------------------------------|
| **Accuracy**           | 2–10 m             | 0.5–2 m              | 1–2 cm                | 5–10 cm                       |
| **Correction Source**  | None               | Ground stations      | Local base station    | Global corrections             |
| **Latency**            | Real-time          | Real-time            | Real-time             | Minutes (convergence)          |
| **Infrastructure**     | None               | Regional             | Local (base/rover)    | None (global)                  |
| **Best Use Cases**     | Navigation         | Surveying, mapping   | Surveying, robotics   | Geodesy, remote areas          |

---

## ✅ Pros and ❌ Cons

### ✅ Advantages
- **High Precision**: Centimeter-level accuracy for demanding applications.
- **Real-Time**: Immediate corrections for dynamic systems.
- **Multi-Constellation**: Robust performance in challenging environments.
- **Widely Supported**: Many GNSS modules and receivers support RTK.

### ❌ Disadvantages
- **Requires Base Station**: Needs a local reference station or access to a correction network.
- **Line-of-Sight Needed**: Radio corrections may require clear line-of-sight.
- **Limited Range**: Accuracy decreases with distance from the base station (typically <20 km).
- **Setup Complexity**: More complex than standard GNSS or D-GPS.

---

## 🔗 Related Topics

- [[GPS]]
- [[D-GPS]]
- [[LC29H]]
- [[GNSS]]
- [[Precision Agriculture]]
- [[Surveying]]
- [[NTRIP]]
- [[RTCM]]
- [[IMU]]
- [[Sensor Fusion]]

---

## 📚 Further Reading

- [RTK Explained (u-blox)](https://www.u-blox.com/en/blogs/technology/what-rtk)
- [RTKLIB Open Source RTK Software](https://rtklib.com/)
- [Quectel LC29H RTK Module](https://www.quectel.com/product/gnss/lc29h/)
- [RTK vs D-GPS (Topcon)](https://www.topconpositioning.com/insights/rtk-vs-dgps)
- [NTRIP Protocol Overview](https://igs.bkg.bund.de/ntrip/about)

---

## 🧠 Summary

RTK is a powerful GNSS enhancement technique that delivers centimeter-level positioning accuracy in real time. Its use of local correction data makes it ideal for surveying, precision agriculture, robotics, and any application where high-precision navigation is critical.
