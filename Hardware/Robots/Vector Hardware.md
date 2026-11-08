# Vector Hardware

Vector Hardware describes the head computer, body controller and sensors inside [[Anki Vector]]. The separation matters: application telemetry, Linux images, body-controller firmware and cube firmware are different development targets.

Hardware details below follow the community reference; they are not a manufacturer service specification or a guarantee that a particular replacement part is compatible.

---

## ⚙️ Main Components

| Component | Documented hardware or role | Development consequence |
|---|---|---|
| Head SoC | Qualcomm APQ8009, quad-core Cortex-A7 | Onboard applications target ARM Linux, not the workstation CPU |
| Memory package | Kingston eMCP with 4 GB eMMC and 4 Gb DDR3 | 4 Gb RAM means **512 MB**, not 4 GB |
| Head IMU | Bosch BMI160 accelerometer and gyroscope | Measures motion in the moving head |
| Body MCU | STM32F030C8 | Handles body I/O and communication with the head |
| Motor feedback | Encoders on the motors | Feedback is not automatically exported as raw counts in the SDK |
| Cliff sensing | Four downward-facing proximity sensors | Individual readings differ from the public cliff-detected status bit |
| Forward ranging | Optical distance/proximity sensor | Interpret validity, geometry and lift obstruction alongside distance |
| Wireless | WCN3610 Wi-Fi/BLE | Robot networking and cube communication are distinct paths |

The headboard also hosts the camera, display and power-management circuitry. The bodyboard connects motors, encoders, cliff/proximity sensing and power-related hardware.

---

## 🧠 Head Computer Versus Body Controller

- **Head/Linux:** perception, localization, behavior, SDK-facing state and application services.
- **Body MCU:** low-level sensor/motor I/O; not the same executable target as `vic-engine` or `vic-robot`.
- **Cube:** a separate BLE accessory, with its own controller and firmware.
- **Charger:** part of the operating setup, not an extra Linux compute target.

The IMU is on the headboard. The inspected `hal_imu.cpp` explicitly describes an **SPI** interface; it is incorrect to route all sensor measurements through the body MCU in an architecture diagram.

See [[Vector Firmware Architecture]] for process and message boundaries.

---

## 🧭 The Moving-Head IMU

The public acceleration and gyro vectors use the **head frame**:

- X points forward through the face.
- Y points out the left side of the head.
- Z points out the top of the head.

Moving the head changes that sensor frame relative to the chassis. A head-frame reading is not interchangeable with a body-frame reading merely because both contain three numbers.

The internal robot-frame gyro history follows a different processing path. [[Vector IMU Telemetry]] traces acquisition, scaling, bias correction, frame transforms and export.

For accelerometer interpretation, account for gravity and sensor placement; a stationary sensor does not imply a zero vector. Software cannot infer a reliable grip-force or torque stream merely from the presence of motor encoders and an IMU.

---

## 📷 Vector 1.0 Versus Vector 2.0

| Item | Vector 1.0 | Vector 2.0 | Why to identify the generation |
|---|---|---|---|
| Camera | Community reference lists GC1066 | Community reference lists BF2253L | Sensor configuration and image processing differ |
| Display | 184 × 96 | 160 × 80 | Pixel dimensions and data handling differ |
| Display transfer | RGB565 over SPI | Byte-swapped RGB565 over SPI | Do not reuse a display routine blindly |
| Battery access | Original enclosure arrangement | Removable battery compartment | Physical maintenance differs |
| Firmware | Must support the actual hardware | Must support the actual hardware | A model label alone does not establish image compatibility |

These are hardware distinctions, not a guarantee that arbitrary camera modes are available through the SDK. The camera's advertised sensor resolution is not necessarily the delivered stream resolution or frame rate.

Do not copy calibration or factory identity material between robots, even if they are the same generation.

---

## 🔧 Choosing the Correct Change Layer

| Requirement | Start here | Not automatically required |
|---|---|---|
| Read existing acceleration/gyro | [[Vector SDK]] | Unlocking or flashing |
| Export gyro history already at the engine | Engine application/export path | New IMU driver or body-MCU firmware |
| Preserve higher-rate six-axis samples | Acquisition/collection and internal messages | Bootloader changes |
| Change Linux platform libraries | Matching OS/application build environment | Altering per-device factory data |
| Change body-controller behavior | Separate MCU source/toolchain/recovery investigation | Assuming a Linux application deploy handles it |
| Change cube behavior | Separate cube firmware investigation | Reflashing the robot's head OS |

---

## ⚠️ Resource and Safety Limits

- RAM-backed staging and telemetry buffers compete with perception and ordinary behavior within a small memory budget.
- eMMC storage is persistent; copying to an onboard directory does not make a change temporary.
- High CPU load, logging and camera streaming can affect latency and temperature.
- Sensors participate in pickup, fall, motion and cliff reactions. Observing their outputs is safer than changing shared filtering to simplify a logger.
- Keep device ownership explicit. A second process opening the hardware is not necessarily a passive observer.
- A higher thread polling rate does not prove lossless end-to-end telemetry at that rate.

Use supervised floor-level experiments, not an exposed tabletop, when later testing motion. Firmware changes do not create a guaranteed physical safety envelope.

---

## 🔗 Related Notes

- [[Vector Firmware Architecture]] — processes and internal messages.
- [[Vector IMU Telemetry]] — measurement semantics and missing public history.
- [[Vector Partitions and Recovery]] — factory identity, calibration and storage.
- [[Vector Gobot]] — alternative hardware-access stack.
- [[IMU]] and [[Encoder]] — general sensor concepts.
- [[SPI]] and [[BLE]] — hardware communication paths.

---

## 📚 External Resources

- [Community hardware reference](https://os-vector.github.io/vector-docs/5.-Hardware/index.html) — board components and generation differences; verify exact parts before repair.
- [Pinned IMU HAL](https://github.com/os-vector/wire-os-victor/blob/505eb84cfaa6ec01c075730f4be855e93713bb59/robot/hal/src/hal_imu.cpp) — SPI acquisition/scaling path.
- [Pinned head-frame definitions](https://github.com/os-vector/wire-os-victor/blob/505eb84cfaa6ec01c075730f4be855e93713bb59/engine/robot.h#L424-L436) — units and axis directions.
- [Pinned internal state producer](https://github.com/os-vector/wire-os-victor/blob/505eb84cfaa6ec01c075730f4be855e93713bb59/robot/supervisor/src/messages.cpp) — sensor data available inside the robot stack.
