---
title: T-Motor H7 Mini
aliases:
  - T-Motor H7 Mini
  - T-Motor H7 Mini Flight Controller
  - H7 Mini
tags:
  - drones
  - flight-controller
  - stm32-h7
  - fpv
  - ardupilot
---

# T-Motor H7 Mini

The **T-Motor H7 Mini** is a compact H7-flight-controller stack intended for FPV quad builds (especially 3.5"-5" classes). It is a performance-oriented alternative to older F4/F7 hobby stacks, with modern sensors and higher MCU headroom.

---

## 🧭 Core specs (first-order profile)

- **MCU**: STM32H743 @ 480 MHz
- **IMU options**: ICM42688P or BMI270 (gyro + accel)
- **Baro**: DPS310
- **OSD**: AT7456E
- **Onboard storage**: 128 Mbits / 16 MB (depending on doc naming)
- **UARTs**: 6 usable UART entries (`SERIAL0/1/3/5/6/7/9` in ArduPilot naming for board map)
- **PWM outputs**: up to 5 motor channels grouped for output mode constraints
- **BEC output**: 5V/2A and 10V/1.5A
- **Power input**: 2S-6S (with broader 7.4-26.4V wiring notes on some vendor PDFs)
- **Typical size**: 20x20 mounting pattern, about 27x30.8 mm listed by vendor

---

## 📎 Why this note exists

You asked for a practical comparison target around H7-class flight controllers for RL/RC workflows in this vault style. This note is written to be comparable with the two likely alternatives below:

- [[Holybro Kakute H7 Mini]]
- [[Matek H743-Wing]]

---

## 🔌 What it exposes (pin/firmware behavior)

From ArduPilot docs for this target:

- RC input is mapped to UART6 (R6/TX6), and all serial RC protocols are supported there.
- UART5 is typically used for GPS (DMA path on that variant), UART1 is used for ESC telemetry.
- UART6 may need half-duplex / pin swap / inversion for protocols like FPort or CRSF-style return-link paths.
- Battery sensing supports up to 130A current and 6S voltage monitoring in supported ArduPilot configs.
- It does **not** ship with a compass; external mag is added over I2C.

---

## 🆚 Comparison table (H7 options for compact builds)

| Board | Target vibe | Core spec strengths | Autonomy path | Main trade-off |
| --- | --- | --- | --- | --- |
| **T-Motor H7 Mini** | Lightweight racing/cine builds, compact FPV | H743 + two gyro options, dual BEC, compact 20x20 format, arming-ready sensor stack | ArduPilot-supported target; also practical for Betaflight/firmware swaps when supported by vendor | 5-parameter port group limitations can constrain custom wiring unless planned in advance |
| [[Holybro Kakute H7 Mini]] | Lightweight racer with HD/analog stacks | Same H743 class, extra integrated pit-switch workflow, dedicated 4-in-1 ESC connectors, HD port support in many variants | Well documented in PX4 + Holybro firmware matrix; easier path when you want PX4 tooling | Bigger ecosystem complexity for version-specific bootloader + variant drift |
| [[Matek H743-Wing]] | More feature-rich stack builds (higher IO count) | 7 UARTs, CAN, microSD family support, 13 PWM channels | Strong ArduPilot orientation with mature firmware lineage | Physically larger and variant-specific pinout differences (WING/SLIM/MINI/WLITE all differ) |
| [[SpeedyBee F405]] | F405 baseline reference class | Mature tuning ecosystem and broad firmware familiarity | Good for beginner control loops and simpler loops | Lower H7 headroom for heavy autonomy workloads |
| [[BETAFPV Air65]] | Integrated compact AIO 3"/mini-size build | Minimal BOM + integrated stack simplicity | Easy build for beginner flight-tuning loops | Not directly the same control-compute class as H7-class autonomous boards |

---

## ✅ Pros

- Strong update path into ArduPilot-style targets for higher-end experimentation.
- Better compute headroom than F405-class FCs for modern loops and filtering.
- Dual BEC and 10V rail make mixed vtx/receiver stacks easier than many AIO-only alternatives.

## ❌ Cons

- Firmware target names can be vendor-sensitive; board revisions often affect wiring and bootloader behavior.
- If you need a lot of high-channel PWM, output grouping and mode restrictions matter.
- [[SpeedyBee F405]]-style simplicity is lower; setup is a bit more deliberate.

---

## 🛠️ Recommended build path (for autonomy + `[[PufferLib]]` workflows)

1. **Pick one stack software path up front**
   - ArduPilot path for real-world autonomy and parameter exposure.
   - Keep firmware source and build target pinned per board revision.
2. **Before first flight, validate I/O map**
   - Confirm RX, ESC telemetry pin mapping on UART6/SERIAL6 and `SERIAL6` half-duplex mode.
   - Decide whether you need CRSF/FPort-like bidirectional RC before final wiring.
3. **Set sensing parameters manually when needed** (ArduPilot examples):

```text
BATT_MONITOR = 4
BATT_VOLT_PIN = 11
BATT_CURR_PIN = 13
BATT_VOLT_MULT = 11.0
BATT_AMP_PERVLT = 50.0
```

4. **For `[[PufferLib]]` transfer**, treat this as a real `H7 + ArduPilot` deployment target:
   - Sim train in software stack
   - Tune failsafes and estimator robustness in SITL first
   - Validate companion-link latency and protocol forwarding in bench/static tests

---

## 🧪 Practical checklist before first tuning session

- Confirm exact gyro variant (ICM42688P vs BMI270) and PCB revision.
- Confirm firmware chain supports your intended stack (`with_bl` DFU flash path, then `.apj` updates).
- Verify OSD output and voltage telemetry on companion/ground station if using logging loops.
- Verify DShot mode consistency: in grouped output mode, if one output in a group is DShot, the group follows.

---

## 📚 External resources

- ArduPilot: https://ardupilot.org/copter/docs/common-tmotor-h7-mini.html
- T-Hobby product detail (compact spec + interfaces): https://www.t-hobby.com/products/t-motor-dual-gyroscope-bec-h7-mini-fight-controller
- Holybro/Kakute comparison baseline (PX4 + firmware): https://docs.px4.io/main/en/flight_controller/kakuteh7mini.html
