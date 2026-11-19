# Matek H743-Wing

`Matek H743-Wing/SLIM/MINI/WLITE` is an ArduPilot-oriented STM32H743 family of high-density flight controllers, where the same firmware target (`MATEKH743`) is shared across multiple variants, while pinout and on-board resource options differ by SKU and revision.

The line is commonly selected when you want:

- high compute headroom for ArduPilot fixed-wing/VTOL features,
- dense output count for mixed motor/servo builds,
- and multiple serial/bus options for sensors, telemetry, and companion I/O.

---

## 🧠 Core architecture

- **MCU:** STM32H743VIT6 or STM32H743VIH6 (480 MHz, typically 2 MB Flash).
- **Default firmware family:** `MatekH743` on firmware.ardupilot.org.
- **Built-in IMUs:** `MPU6000` + `ICM20602` is shown in the shared ArduPilot page baseline.
- **Built-in barometer:** `DPS310`.
- **Built-in OSD:** yes (typically AT7456E class).
- **Logging:** microSD support.
- **Power rails:** multiple BEC outputs (5V/servos/video style rails depending on variant/model).
- **Common interfaces:** multiple UARTs, 2 I²C ports, CAN, 6 ADC inputs.
- **Critical limitation:** no onboard GPS and no built-in compass in the family baseline; external modules are required for autonomous operation.

---

## 🔀 Variant-level resource differences

The ArduPilot docs explicitly call out that variant-specific pinouts/resources differ even though firmware is shared.

| Variant | IMU(s) | Baro | Current sensing | PWM | Notes |
|---|---|---|---|---|---|
| `H743-WING` (older variants) | `MPU6000` + `ICM20602` | `DPS310` | integrated in V2/V3, default scaling tuned for 132A board family | 13 PWM | 2 camera inputs, dual batt monitor support |
| `H743-WLITE` | `ICM42688-P` + `DPS310` (family docs) | `DPS310` | integrated + high range | ~12 PWM | typically has 220A class current path on WLITE manuals |
| `H743-SLIM` V3/V4 | dual IMU variants incl. dual ICM42688P (`V4`) | `DPS310` or `DPS368` | usually no built-in current sensor | 13 PWM | often selected for tighter airframe constraints |
| `H743-MINI` V3 | `ICM42688-P` + `ICM42605` | `DPS310` | no guaranteed internal current stage as standard | 11 PWM | compact 20×20 footprint target |

Practical impact:

- `WLITE` and newer `WING` revisions can save wiring on power monitoring.
- `SLIM`/`MINI` are compact alternatives with lower total connector bulk.
- If you are comparing by capability, **the base `WING`/`WLITE` are usually the safest for first-time high-load fixed-wing builds**, because of richer power-monitor and I/O options.

---

## 🔌 I/O layout and defaults

## Default serial mapping (family baseline from ArduPilot page)

| ArduPilot serial | Physical UART | Typical use |
|---|---|---|
| `SERIAL0` | USB | console / MAVLink |
| `SERIAL1` | UART7 | TELEMETRY1 (CTS/RTS available) |
| `SERIAL2` | USART1 | TELEMETRY2 |
| `SERIAL3` | USART2 | GPS1 |
| `SERIAL4` | USART3 | GPS2 |
| `SERIAL5` | UART8 | USER |
| `SERIAL6` | UART4 | USER |
| `SERIAL7` | UART6 (RX only by default) | RC6 pad, or full UART RX when `BRD_ALT_CONFIG=1` |

### RC input behavior

- `Rx6` / `SERIAL7` is timer-mapped by default and supports most ArduPilot RC protocols.
- `CRSF`, `ELRS`, and `SRXL2` generally require true UART behavior and therefore `BRD_ALT_CONFIG=1` (with protocol changes in `SERIAL7_PROTOCOL` / `SERIAL7_OPTIONS`).
- `PPM` is not supported in this line.

### PWM outputs

`MATEKH743` mappings are PWM/DShot-capable on all outputs, with grouped protocol constraints:

- output groups that must share the same output mode:
  - `1/2`
  - `3/4/5/6`
  - `7/8/9/10`
  - `11/12`
  - `13 (LED)`

That means you cannot mix PWM and DShot within one group.

### CAN and I²C

- 1 CAN bus port exposed in the shared baseline.
- 2 I²C ports (`I2C1`, `I2C2`) for GPS/compass, airspeed, and expansion devices.
- In-flight companion and DroneCAN peripherals are commonly routed through this CAN path.

### ADC channels

- 6 ADC channels are exposed in the shared page for:
  - battery voltage, current,
  - second battery voltage/current,
  - RSSI, and analog airspeed.
- `BATT2` paths are present in default config even if you do not enable them immediately.

---

## ⚡ Power and battery-monitor behavior

Family defaults (ArduPilot shared page):

- `BATT_MONITOR = 4`
- `BATT_VOLT_PIN = 10`
- `BATT_CURR_PIN = 11`
- `BATT_VOLT_MULT = 10.5`
- `BATT_AMP_PERVLT = 40`
- `BATT2_VOLT_PIN = 18`
- `BATT2_CURR_PIN = 7`
- `BATT2_VOLT_MULT = 11.0`

Important overrides:

- `WLITE` and `WING V2/V3` commonly need `BATT_AMP_PERVLT` adjusted to the higher onboard-current path.
- If you change `BATT2_*` defaults, validate scale values against the exact divider and current sensor board.

ArduPilot notes call out current sensor noise sensitivity on these integrated channels; keep large electrolytic bypass capacitance close to ESC and power entry in mind.

---

## 🧩 Peripheral compatibility and wiring strategy

This board is strong for mixed fixed-wing payload stacks, but it rewards deliberate port planning. The biggest wins are clean sensor bus organization and strict protocol consistency.

### External navigation sensors (required)

- **`GPS + Compass + maybe baro` is not onboard**: connect external modules and let ArduPilot auto-detect.
- Common paths:
  - **UART GPS/compass** on one of `UART2/3/4/...`.
  - **I²C compass** on one of the I²C connectors if a clean external mag path is desired.
  - **DroneCAN GPS** via CAN when using AP_Periph-class nodes.

### Peripheral compatibility matrix (what works well on this family)

| Peripheral type | Preferred path on MATEKH743 | Typical example module | What to set |
|---|---|---|---|
| GPS only (UART) | USART2/USART3/USART4/USART1/USART8 depending on your free port | u-blox NEO-M8N, M10, etc. | set `SERIALx_PROTOCOL` to GPS type, leave `COMPASS_TYPE` external if using dedicated mag |
| GPS + compass + baro combo | CAN/DroneCAN (best for low UART load) or I²C compass on external bus | `Matek M8Q` (CAN/MSP), `Matek M10Q` | DroneCAN path: `CAN_D1_PROTOCOL=1`, `CAN_P1_DRIVER=1`, `GPS1_TYPE=9`; then set corresponding compass and baro auto-detect flags |
| Compass-only for better heading | I²C on `I2C1`/`I2C2` | QMC5883/QMC5883L-based mag, IST8310, etc. | keep onboard interference-prone compasses disabled if needed |
| Analog airspeed | `ARSPD_PIN` analog channel (Airspeed pad) | MPXV7002-type analog modules, pressure transducer modules | `ARSPD_TYPE=2`, configure `ARSPD_PIN` per pad and `ARSPD_USE` as needed |
| Digital airspeed | I²C or DroneCAN (preferred) | `ASPD-DLVR` (`ARSPD_TYPE=9`) | set `ARSPD_BUS` (I²C bus) when on I²C; or `ARSPD_TYPE=9` for CAN DLVR nodes |
| Telemetry radio | any unused UART, often USART1/USART7 first priority | SiK, SiK-v2, RFD900, LoRa modems | set that `SERIALx_PROTOCOL` (MAVLink) and `SERIALx_BAUD` to radio spec |
| RC receiver | RX6 (by default timer-based), or full UART with `BRD_ALT_CONFIG=1` for CRSF/ELRS/SRXL2 | FrSky SBus/FPort, ELRS, Crossfire, SBUS | if using CRSF/ELRS/SRXL2: `BRD_ALT_CONFIG=1`, `SERIAL7_PROTOCOL=23`, `SERIAL7_OPTIONS` per protocol |
| Camera switching / VTX power | Onboard relay pins 81/82 + GPIO-controlled relay options | analog FPV camera + VTX chain | `RELAY1_PIN=81`, `RELAY2_PIN=82`, then bind a trusted RC channel to `RCx_OPTION=34` |

For many fixed-wing builds, this is usually the cleanest layout:

- One UART dedicated to telemetry, one UART to GPS, one serial-capable pin for RC if using CRSF/FPort/SRXL2, and a second UART/I²C split for airspeed + compasses.
- Keep noisy 5V power rails short; route ground returns with the same harness as signal where possible.
- Add a common star point for 5V/6V/9V/12V supplies if you use servo camera VTX split rails.

### GPS / compass module compatibility table

| Module | Interface | When to prefer | Setup notes |
|---|---|---|---|
| `Matek M8N` / standard u-blox GPS/compass modules | UART + I²C | classic layout, no CAN | set `SERIALx_PROTOCOL` to GPS type, keep external mag enabled |
| `Matek M8Q` (CAN/MSP) | CAN or UART-MSP | clean wiring, low UART pressure builds | CAN mode requires `CAN_D1_PROTOCOL=1`, `CAN_P1_DRIVER=1`, `GPS1_TYPE=9` |
| `Here3+`-style AP_Periph modules | CAN | advanced multi-sensor builds | use Node-based CAN settings and per-device parameters |

### Airspeed and atmospheric sensing

- Airspeed path options on this family:
  - Analog analog input (`ARSPD_TYPE = 2`) with compatible analog sensor.
  - `DLVR-10` class digital airspeed via I²C or CAN (preferred for temperature stability).
- For `DLVR`:
  - set `ARSPD_TYPE` accordingly (typically `9` for I²C DLVR-10 variants in relevant firmware/board mappings),
  - when using CAN-based variants, `GPS1/GPS2` and `ARSPD_TYPE=9` are in the same DroneCAN node ecosystem.

### Companion computer + serial/telemetry planning

- You can allocate serial ports for:
  - telemetry radio,
  - USB/UART bridge,
  - OSD/HD link overlays,
  - companion traffic.
- For full-duplex links (`CRSF`, `SRXL2` on dedicated RC wiring), use the Rx6 UART-alt config explicitly; otherwise you lose telemetry channels on those protocols.

### Camera, VTX, and GPIO switching

- The shared family docs describe relay-controlled switching for:
  - Vsw / power on/off supply rail,
  - Camera input switching.
- Typical relay setup pattern:
  - `RELAY2_PIN = 82` for camera switch,
  - `RCx_OPTION = 34` (Relay2 control),
  - assign an RC switch that you trust (for safe in-flight control).

### Peripheral check routine (recommended before flight)

1. Confirm serial map against actual hardware revision.
2. Bind GPS/compass and confirm fix + compass status before arming.
3. Validate battery monitor scale with a bench measurement (voltage divider + current source).
4. Validate RC protocol settings for all telemetry/RC paths.
5. Validate DShot mode and group constraints before assigning motor outputs.
6. Verify any DroneCAN nodes are assigned valid `CAN_D1_*` and `GPSx_TYPE` values.

---

## 🔁 Firmware and flashing

- Target folders are in the ArduPilot firmware server under `MatekH743`.
- `MatekH743` is the normal target.
- `MatekH743-bdshot` enables dedicated bi-directional DShot behavior.
- `MatekH743-bdshot-heli` exists for heli-oriented build paths.

Initial firmware is often done through DFU using a `with_bl` image, then standard `.apj` updates through Mission Planner / QGC-style ground software.

Notes on bdshot variants:

- BDshot variants usually rework Rx6 mapping behavior.
- Some output combinations that are valid in normal target are intentionally restricted under bdshot.
- You typically lose full compatibility for non-bidirectional servo behaviors on specific channels; check output grouping before changing this target.

---

## ✅ Strengths vs limitations

### Strengths

- Very dense serial and PWM layout for fixed-wing complexity.
- Mature ArduPilot documentation and dedicated target support.
- Onboard logging and OSD support on common variants.
- Strong companion/comms path when configured per protocol.

### Limitations / caveats

- Not beginner-plug-and-play for RC protocols with bidirectional telemetry.
- No onboard GPS/compass means external dependency is mandatory for nav.
- DShot/PWM group constraints can break partial motor remaps if left unchecked.
- Current monitor scale values depend on hardware revision and must be verified.

---

## 📈 Comparison chart (5+ boards)

### Fixed-wing autopilot board comparison

| Board | MCU / Flash | IMUs | UART count | PWM channels | Canbus | OSD | Built-in GPS/compass | Power monitor default |
|---|---|---|---|---|---|---|---|---|
| `Matek H743-Wing` | STM32H743 (2 MB) | MPU6000 + ICM20602 | 7 (plus USB) | 13 | 1 | AT7456E-based | none | yes (WING variants, with range differences) |
| `Matek H743-SLIM V4` | STM32H743 | dual ICM42688 (family) | 7 | 13 | 1 | AT7456E | none | often external only |
| `Matek H743-MINI V3` | STM32H743 | ICM42688-P + ICM42605 | 7 | 11 | 1 | AT7456E | none | often external |
| `Matek H743-WLITE` | STM32H743VIH6 | ICM42688-P + DPS310 | 7 | 12 | 1 | AT7456E | none | built-in current path on WLITE variants |
| `ORBITH743` | STM32H743VIH6 | dual ICM42688 | 8 | 13 | 1 | MAX7456 + DisplayPort options | no | 2-channel battery monitor |
| `CORVON743V2` | STM32H743VIH6 | ICM-42688 + BMI088 | 8 + USB | 12+LED (10 motor +2 servo) | 1 | no analog OSD chip, DisplayPort path | built-in IST8310 (often disabled in flight) | dual analog battery monitor |
| `KakuteH743-WING` | STM32H743 | ICM42688 | 6 | 14 | 1 | AT7456E | no | built-in, 120A class |
| `MicoAir743v2` | STM32H743 | dual IMU (BMI088/BMI270) | 8 | 11 | often none / board-specific | AT7456E + DisplayPort | yes (QMC5883L onboard) | internal voltage + external current |

### Why this matters for selection

- If you want maximum fixed-wing output count and ArduPilot continuity, pick **Matek WING/WLITE**.
- If you need built-in dual battery telemetry out of the box and DisplayPort-heavy camera stack, **CORVON743V2** is a practical peer.
- If you want bigger PWM density than a typical 8-in/10-in multicopter board and still want `MATEKH743` firmware workflow, **ORBITH743** is a direct neighbor.

---

## 🧪 Quick compatibility matrix (practical setup decisions)

| Requirement | M8Q CAN + AP_Periph | Analog airspeed stack | HDVTX DisplayPort | BDshot motors | Multi-companion serial links |
|---|---|---|---|---|---|
| Out-of-box effort | moderate | moderate | moderate | high (firmware choice + group map) | high |
| Recommended on this line | good for low-UART builds | good (or DLVR if available) | possible via UART assignment | useful for efficiency telemetry | yes, with careful `SERIALx` planning |
| Typical complexity | `CAN_D1_*` + GPS type config | ARSPD type + voltage range check | pin routing + protocol choice | output map re-verify + `SERIAL7` behavior | ensure one stable ground + noise management |

---

## 📚 Related notes

- [[CAN]]
- [[I2C]]
- [[UART]]
- [[PWM]]
- [[MAVLink]]
- [[ArduPilot]]
- [[ArduPlane]]
- [[ArduPilot Mission Planner]]
- [[ArduPilot Boards]]
- [[PX4 Boards]]

---

## 🔗 External resources

- ArduPilot shared target page: `https://ardupilot.org/copter/docs/common-matekh743-wing.html`
- ArduPilot firmware index (MatekH743 and bdshot variants): `https://firmware.ardupilot.org/Copter/stable/`
- Matek product pages (variant pages and manuals):
  - `https://www.mateksys.com/?portfolio=h743-slim-v4`
  - `https://www.mateksys.com/?portfolio=h743-wlite`
  - `https://www.mateksys.com/?portfolio=h743-mini`
- `Matek M8Q DroneCAN/MSP` (for CAN GPS/compass/baro integration): `https://ardupilot.org/sub/docs/common-matek-m8q.html`
- `Matek DLVR-10` (airspeed integration): `https://ardupilot.org/copter/docs/common-matek-uavcan-dlvr.html`
- `CAN (DroneCAN) setup`: `https://ardupilot.org/copter/docs/common-uavcan-setup-advanced.html`
- Comparable board references:
  - `https://ardupilot.org/copter/docs/common-orbith743.html`
  - `https://ardupilot.org/copter/docs/common-corvon743v2.html`
  - `https://ardupilot.org/copter/docs/common-kakuteh7wing.html`
  - `https://ardupilot.org/copter/docs/common-MicoAir743v2.html`
