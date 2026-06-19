---
title: Holybro Kakute H7 Mini
aliases:
  - Holybro Kakute H7 Mini
  - Holybro Kakute H7mini
  - Kakute H7 mini
tags:
  - drones
  - flight-controller
  - stm32-h7
  - autopilot
  - px4
  - ardupilot
---

# Holybro Kakute H7 Mini

The **Holybro Kakute H7 Mini** sits in the same class as the `[[T-Motor H7 Mini]]`: compact H7 flight controllers for lightweight builds that need modern IMU/baro/OSD features and decent headroom.

---

## 🧭 Core spec profile

- **MCU:** STM32H743, 480 MHz
- **IMU:** `BMI270` (v1.5 / modern batches), earlier revision variants include MPU6000
- **Barometer:** BMP280
- **OSD:** AT7456E
- **Flash:** 16 MB (older) / 128 MB (newer batches, when present)
- **Power input:** 7–26 V (2S–6S)
- **BEC:** 5V 2A output
- **UARTs:** 6 serials for GPS, telemetry, OSD, RC
- **PWM outputs:** 9 outputs (8 motors + WS2812-like pad)
- **Interfaces:** USB-C, external I2C, analog RSSI, buzzer output, switchable VTX power
- **Known caveat:** product-availability note appears to vary by revision; newer batch support changes over time.

---

## 🧩 How it compares in practice

| Board | Notable trait | Serial model | Pin/IO strategy | Typical fit |
|---|---|---|---|---|
| **Holybro Kakute H7 Mini** | Balanced FPV-H7 with better accessory ecosystem than many F4 class | Supports ArduPilot and PX4 workflows in vendor docs | `SERIAL6` default is RX for RC in many docs; `R6` is used heavily for control path | Lightweight FPV + autopilot-capable builds |
| [[T-Motor H7 Mini]] | Similar H7 class + stronger current sense range in T-Motor docs | Strong ArduPilot docs with explicit UART map and battery parameters | `SERIAL6` configured for all-serial RC plus half-duplex options for telemetry-capable protocols | T-Motor variant-first stack migration from Betaflight-like workflows |
| [[Matek H743-Wing]] | Higher connector/feature density for larger builds | `MatekH743` firmware family | Variant-specific pinouts; more PWM/UART/CAN availability | Wing/airframe builds and servo-heavy layouts |
| [[SpeedyBee F405]] | Lower-compute class but easier ecosystem familiarity | Non-H7 MCU class | Simpler UART and PWM expectations | Beginner tuning and cost-first builds |
| [[BETAFPV Air65]] | Tiny AIO form factor with ELRS-first stack | Mostly Betaflight-centric | All-in-one integration | Tiny whoops where full autonomy stack is unrealistic |

---

## ⚡ Firmware and integration notes

- **Bootloader / flashing path:** board commonly starts from non-ArduPilot setup; check firmware family docs before flashing
- **PX4:** supported on modern doc branches when bootloader/tooling matches current build target
- **ArduPilot:** multiple firmware folders depending on generation, often `KakuteH7Mini` + NAND/legacy split in older pages
- **RC input behavior:** commonly maps to UART6 and commonly supports most modern serial RC protocols except PPM unless remapped
- **OSD path:** built-in `MAX7456` path is straightforward when using OSD_TYPE 1 defaults
- **Compass behavior:** no built-in magnetic compass on many batches; external I2C compasses are common

---

## 🧱 What to check before flight stack setup

1. Confirm board generation (`v1.1`, `v1.3`, `v1.5`) before cloning parameter templates.
2. Pin this as a hard requirement for your vehicle:
   - RC/telemetry receiver protocol (`SERIAL6_PROTOCOL` + options)
3. Confirm current sensor wiring expectations if you want current-integrated metrics.
4. Keep `SERIAL6`/`SERIAL1` usage explicit if you also run camera OSD or DJI-related side links.

For `PufferLib` workflows this is usually a better paper prototype/autopilot stack than a pure Betaflight AIO board, but still requires clean telemetry and port planning before live hardware experiments.

---

## ✅ Pros / ❌ Cons

### Pros
- solid 480 MHz H7 platform for compact builds
- clean integration with FPV-oriented ports and accessories
- known ecosystem docs for both ArduPilot and PX4 pathways
- stronger default 2S–6S compatibility than many micro whoop FCs

### Cons
- revision drift changes flash + pinout behavior between older/newer hardware
- not a “plug and play” board for every stack unless flash and serial map are set first
- no guaranteed internal compass on all revisions
- 9V camera power and VTX behavior require explicit configuration in some firmware setups

---

## 🧪 Comparison table by practical objective

| Objective | Holybro Kakute H7 Mini | Why pick it |
|---|---|---|
| Lightweight FPV autopilot + autonomy experiments | Strong | compact, well-documented |
| Maximum port symmetry for advanced peripherals | Moderate | better than micro AIO, but less than Matek Wing |
| Autonomy-first stack reliability on first iteration | Medium | good once bootloader and UART map are set |
| Lowest maintenance in quick prototypes | Moderate | still needs version-aware setup |
| High-availability parts | Weak to medium | version availability can be revision sensitive |

---

## Related notes

- [[T-Motor H7 Mini]]
- [[Matek H743-Wing]]
- [[SpeedyBee F405]]
- [[PufferLib]]

---

## 📚 External resources

- ArduPilot: `https://ardupilot.org/sub/docs/common-holybro-kakuteh7mini.html`
- PX4: `https://docs.px4.io/main/en/flight_controller/kakuteh7mini.html`
- Holybro: product listing and availability (version-specific)
