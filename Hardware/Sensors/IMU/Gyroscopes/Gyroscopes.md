# 🟣 Gyroscopes (Hobbyist-Friendly)

**Gyroscopes** measure angular velocity (the rate of rotation) around one or more axes. In hobbyist robotics, drones, and embedded systems, small MEMS (Micro-Electro-Mechanical Systems) gyroscopes are widely used as standalone devices or as part of an **IMU (Inertial Measurement Unit)**.

This note focuses on gyroscopes and IMUs that are affordable, accessible, and easy to integrate with microcontrollers (e.g., Arduino, ESP32, Raspberry Pi).

---

## 🧠 Summary

- **MEMS gyroscopes** are compact, low-power, and inexpensive.
- Typically used for orientation estimation, motion tracking, stabilization (e.g. in drones, robots, cameras).
- Often combined with accelerometers and magnetometers in IMUs for better drift correction.

---

## ⚙️ Commonly Used Hobbyist Gyroscopes / IMUs

| Sensor         | Axes | Interface | Integrated Accel? | Integrated Mag? | Notes                                          |
|----------------|------|-----------|------------------|----------------|------------------------------------------------|
| **MPU-6050**   | 3    | I2C       | Yes               | No             | Very popular; low cost; basic 6-DOF IMU.       |
| **BNO055**     | 3    | I2C/UART   | Yes               | Yes            | Includes onboard sensor fusion + Euler angles. |
| **ITG-3200**   | 3    | I2C       | No                | No             | Older 3-axis gyro, good for basic experiments. |
| **L3G4200D**   | 3    | I2C/SPI    | No                | No             | SPI option; low drift; good for robotics.      |
| **MPU-9250**   | 3    | I2C       | Yes               | Yes            | Successor to MPU-6050; adds magnetometer.      |
| **LSM6DS3**    | 3    | I2C/SPI    | Yes               | No             | Modern; low power; small package.              |
| **LSM9DS1**    | 3    | I2C/SPI    | Yes               | Yes            | 9-DOF with mag; good support in libraries.     |

---

## 🚀 Use Cases

| Application                   | Typical Sensor Choice                  |
|---------------------------------|----------------------------------------|
| **Drone flight stabilization**  | MPU-6050, MPU-9250                    |
| **Handheld IMU tracking**       | BNO055 (onboard fusion simplifies code)|
| **Self-balancing robots**       | MPU-6050, L3G4200D                    |
| **Camera gimbals**              | L3G4200D, MPU-6050                    |
| **Educational / learning**      | ITG-3200 (very basic, easy to start)   |
| **Magnetometer + gyro needed**  | MPU-9250, LSM9DS1                     |

---

## 🏆 Strengths

- 🟢 **Affordable** – Most hobbyist gyros/IMUs cost between $3 - $25.
- 🟢 **Compact** – Easily fits on small PCBs and sensor boards.
- 🟢 **Good library support** – Available drivers for Arduino, MicroPython, ROS, etc.
- 🟢 **Widely tested** – Community projects and tutorials abound.

---

## ⚠️ Weaknesses

- 🔴 **Drift** – All MEMS gyros experience drift over time; fusion with accelerometers/magnetometers or external corrections (e.g. GPS) needed for long-term stability.
- 🔴 **Noise** – Lower-cost units may have higher noise and require filtering (e.g. Kalman, complementary filter).
- 🔴 **Varying quality** – Cheap clones or poor PCB layout can reduce accuracy.

---

## 📊 Detailed Comparison Table

| Feature             | MPU-6050         | BNO055         | ITG-3200        | L3G4200D        | MPU-9250        | LSM6DS3        | LSM9DS1        |
|---------------------|-----------------|----------------|-----------------|----------------|----------------|----------------|----------------|
| Axes                | 3 gyro + 3 accel | 3 gyro + 3 accel + 3 mag | 3 gyro | 3 gyro | 3 gyro + 3 accel + 3 mag | 3 gyro + 3 accel | 3 gyro + 3 accel + 3 mag |
| Magnetometer         | No               | Yes             | No               | No              | Yes             | No              | Yes             |
| Interface            | I2C              | I2C/UART        | I2C              | I2C/SPI         | I2C             | I2C/SPI         | I2C/SPI         |
| Fusion onboard?      | No               | Yes (with orientation output) | No | No | No | No | No |
| Power consumption    | Low               | Moderate        | Low              | Low             | Low             | Very low        | Low             |
| Notes                | Low cost; needs external fusion | Simplifies orientation math | Basic; minimal setup | SPI useful for high-rate apps | Better than MPU-6050 | Modern, low power | All-in-one 9-DOF |
| Price (est.)         | $3-5             | $15-25          | $5-10            | $5-10           | $7-15           | $5-10           | $7-15           |

---

## 🔗 Related Notes

- [[Kalman Filter]]
- [[Sensor Fusion]]
- [[IMU]]
- [[I2C]]
- [[SPI]]
- [[ROS2]]
- [[MPU-6050 3-Axis Gyroscope]]
- [[BNO055]]
- [[ITG-3200 Tri Axis Gyroscope]]
- [[L3G4200D 3-Axis Gyroscope]]

---

## 🌐 External References

- [MPU-6050 Datasheet](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/)
- [BNO055 Datasheet](https://www.bosch-sensortec.com/products/smart-sensors/bno055/)
- [LSM9DS1 Datasheet](https://www.st.com/resource/en/datasheet/lsm9ds1.pdf)

---
