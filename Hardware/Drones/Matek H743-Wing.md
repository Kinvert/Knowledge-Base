---
title: Matek H743-Wing
aliases:
  - Matek H743 Wing
  - Mateksys H743-Wing
  - Matek H743-Wing Mini
tags:
  - drones
  - flight-controller
  - stm32-h7
  - ardupilot
  - fixed-wing
---

# Matek H743-Wing

The **Matek H743-Wing** family (`WING`, `SLIM`, `MINI`, `WLITE`) is a higher-capability ArduPilot line with variant-specific pinouts and a larger I/O envelope than many racing-style H7 mini boards.

---

## 🧭 Core profile

- **MCU:** STM32H743VIT6, 480 MHz
- **IMU:** MPU6000 + ICM20602 in documented variants
- **Barometer:** DPS310
- **Flash/features:** microSD logging and large output topology
- **Power input:** 9V–36V class power ranges
- **BECs:** 5V (peripheral), 9V/12V (video), high-current 5/6/7.2V rail for servos
- **UARTs:** 7 serial ports on the board
- **PWM outputs:** 13 channels
- **CAN:** present
- **RC input:** PWM/PPM/SBUS support with UART routing controls
- **Important:** all variants share firmware but not identical pinout/resource mapping

---

## 🧩 Why this is not a drop-in `[[T-Motor H7 Mini]]` comparison

The `[[T-Motor H7 Mini]]` is compact and straightforward for FPV quad style wiring. `Matek H743-Wing` is generally chosen when you need:

- servo-heavy topologies,
- larger vehicle classes,
- extra serial ports,
- and more mixed peripherals/camera/relay behavior than 20x20 quad boards typically expose.

---

## 🔧 Default mapping behavior (high-level)

- **Default UART routing** is software-adjustable; canonical default order includes:
  - `SERIAL0` as console/USB,
  - `SERIAL1` telemetry,
  - `SERIAL2` GPS,
  - `SERIAL3` GPS2,
  - `SERIAL5/6/7` user ports and optional RC/bi-directional modes.
- **RC input defaults:** `R6` often starts as timer input, with UART remap for full CRSF/ELRS/SRXL2 pathways requiring extra settings.
- **DShot:** bi-directional DShot behavior is affected by whether bidirectional firmware mode is active.
- **Buzzer/Relay:** can be repurposed for camera and power switching in larger builds.

---

## ⚙️ Practical implications for firmware

- **Initial flash path:** DFU with bootloader payload, then normal ArduPilot update flow.
- **Upgrade flow:** typical ArduPilot `.apj` update path once bootloader exists.
- **RC/telemetry workflows:** check `BRD_ALT_CONFIG`, `SERIAL7_PROTOCOL`, and `SERIAL7_OPTIONS` when using CRSF/FPort/ELRS bi-directional links.
- **Current sensing:** board-level current-sensor availability depends on sub-variant (`-WING V2/V3`, `-WLITE` noted for integrated current monitoring).
- **Mounting strategy:** often easier to treat these as a separate integration path rather than a “small racer replacement.”

---

## ✅ Pros / ❌ Cons

### Pros
- significantly richer I/O and UART profile than ultra-compact FPV H7 boards
- fixed-wing and servo layouts fit more naturally
- high-capacity power rails for VTX/video and servo domains
- microSD + CAN support in one compact family

### Cons
- variant-specific resource differences are real; wrong variant mapping can waste days of debugging
- overkill for small quad or whoop platforms
- more ports means more explicit planning burden on first bring-up

---

## 🧪 Compare-by-objective table

| Objective | Best fit |
|---|---|
| Max peripheral count | [[Matek H743-Wing]] |
| Smallest FC stack | [[T-Motor H7 Mini]] / [[Holybro Kakute H7 Mini]] |
| Simpler F4-class ramp | [[SpeedyBee F405]] |
| Tiny whoop style | [[BETAFPV Air65]] |
| H7-class FPV baseline | [[T-Motor H7 Mini]] |

---

## 🔍 Comparison chart with 5+ alternatives

| Board | MCU class | UART profile | PWM channels | Best use |
|---|---|---|---:|---|
| **Matek H743-Wing** | H7 class | 7+ | 13 | wing / servo-rich builds |
| [[T-Motor H7 Mini]] | H7 class | 6 (documented class) | 5 (1-4 group + M5) | small quad/compact builds |
| [[Holybro Kakute H7 Mini]] | H7 class | ~6 | 9 | compact FPV + autopilot |
| [[SpeedyBee F405]] | F4 class | limited vs F7/H7 class | fewer | beginner/manual FPV tuning |
| [[BETAFPV Air65]] | tiny AIO class | n/a | n/a | lowest footprint, limited payload/autonomy |

---

## 🔗 Related notes

- [[T-Motor H7 Mini]]
- [[Holybro Kakute H7 Mini]]
- [[SpeedyBee F405]]
- [[PufferLib]]

---

## 📚 External resources

- ArduPilot: `https://ardupilot.org/copter/docs/common-matekh743-wing.html`
- Mateksys product family overview (variant-level details)
