---
title: LDROBOT STL-06P
aliases:
  - STL-06P
  - LDROBOT STL-06P
tags:
  - lidar
  - robotics
  - sensors
  - mini-pupper
---

# LDROBOT STL-06P

## Overview

The **STL-06P** is a compact 2D lidar positioned around Mini Pupper-class robots and robotics education projects.

It is notable for:

- very small mass,
- compact body size,
- high internal ranging rate for a compact model,
- and explicit Mini Pupper community adoption.

---

## Verified specs from datasheet

From the LDROBOT datasheet:

- **Scanning:** default 10 Hz, scan range 6–13 Hz, and **5000 Hz ranging frequency**.
- **Range:** 0.03 m to 12 m class.
- **Accuracy (white target):** around ±10 mm at 0.03–0.5 m, degrading with distance.
- **Field tolerance:** ambient light up to about 60 kLux stated.
- **Interface:** UART 230400, plus external PWM speed control pin.
- **Electrical:** 5V supply nominal, ~290 mA working current.
- **Physics dimensions:** roughly 38.6 × 38.6 × 33.5 mm and 45 g body mass.

---

## Mini Pupper relation

The Mini Pupper docs state STL-06P as the default Lidar module for ROS/SLAM workflows and explicitly call out custom cable wiring for easy mounting.

This is a practical point if you want to skip heavy mechanical redesign:
- proven software expectation in a legged robot ecosystem,
- known behavior as a "small footprint ring for obstacle and SLAM tasks."

---

## Integration notes for your 2DOF bot

- The module is small enough to mount on a narrow mast or side mast while leaving room for IMU and battery.
- Serial bandwidth is straightforward: 230400 8-N-1.
- PWM control gives you explicit spin-speed adaptation if you want deterministic ray timing.
- It does not natively solve high-level localization by itself; you still need orientation and odometry context.

---

## Pros / cons in your stack

### Pros
- Excellent mechanical footprint.
- High ranging frequency for a compact package.
- Good community references in ROS-based mobile robots.

### Cons
- 3.5D/reflectivity channels are limited compared with some SL-DTOF class products.
- Still 2D only; your vertical profile must come from mounting or repeated transforms.

---

## Sources

- LDROBOT datasheet (v1.3): https://www.ldrobot.com/images/2023/03/02/LDROBOT_STL-06P_Datasheet_EN_v1.3_txOyicBl.pdf
- Mini Pupper docs (Lidar module note): https://minipupperdocs.readthedocs.io/en/latest/guide/Features.html
- Mini Pupper STL-06P listing: https://www.robotshop.com/products/mangdang-mini-pupper-stl-06p-lidar-module
- Sensorlidar commercial listing with key feature statement: https://www.sensorlidar.com/products/ldrobot-stl-06p-lidar-outdoor-light-resistance-60klx-5000hz-12m-360-scanning-support-ros1-and-ros2-replaces-ld06-lidar

