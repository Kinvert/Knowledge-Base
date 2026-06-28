# ArduPilot Mission Planner

`ArduPilot Mission Planner` is the project’s primary Windows ground control station (GCS) for initial configuration, mission planning, and field ops on `Plane`, `Copter`, and `Rover`.

It is often the first GCS people use to validate `ArduPilot` changes, fly basic test scripts, load firmware, and build autonomous missions without writing custom tooling.

---

## 🧭 What it is (and what it is not)

`Mission Planner` is a full-featured mission and telemetry app, but not the only ArduPilot GCS and not a replacement for platform-independent tooling.

Core roles:

- Firmware install and stack updates on supported boards
- `Mission` and `Survey` creation
- Runtime telemetry, parameter management, and mode actions
- Log review and basic flight-data analysis
- Flight simulation entrypoint when using ArduPilot SITL

Limits:

- It is `Windows` oriented first.
- The docs call out Linux use via MONO as secondary and sometimes unstable.
- `Pixhawk` is a hardware family; Mission Planner is one tooling option for stacks that support it.

---

## 🧱 Installation and platform expectations

### Windows path

- Download Mission Planner from `firmware.ardupilot.org` or the linked installer URL from the docs.
- Install from the `.msi` package and keep `Windows` drivers updated for board enumeration.
- Always run the latest release branch to keep signing keys, firmware links, and defaults current.
- The docs explicitly say it is designed for Windows first.

### Linux use

- Use MONO and run the zip-distribution executable.
- Typical flow: install MONO, unzip Mission Planner, run `mono MissionPlanner.exe`.
- The docs warn that this setup can have occasional issues and crashes compared with native tools.

### Version and update behavior

- The installer notifies for update checks; users are advised to use current versions.

---

## 🔌 Connect and onboarding the autopilot

Mission Planner supports several transport methods.

- USB cable (default for bench setup)
- Telemetry radios
- TCP/UDP endpoints
- Bluetooth or other serial-over-PC transports

Common defaults and gotchas:

- USB connect is commonly `115200` baud.
- Telemetry radio default in docs is commonly `57600` baud.
- On startup, choose port and rate before pressing `CONNECT`.
- If connection fails, check boot phase timing and Windows driver enumeration.
- The connection dropdown also supports network endpoints (`udp://`, `tcp://` forms).

For `F7/H7` boards with `CAN`, the docs include composite USB behavior notes: Mission Planner may expose multiple COM ports for MAVLink vs `SLCAN`, and wrong port/protocol selection can block connection.

---

## 🧩 Firmware workflow

The key distinction is whether ArduPilot is already installed.

### Boards with existing ArduPilot firmware

- Connect via USB directly and choose the board COM/baud.
- Use `SETUP -> Install Firmware`, select `vehicle/frame`, and confirm.
- Board-id detection can request a re-plug cycle.
- Save parameters to file before upgrade where possible.
- Parameter defaults can reset when changing vehicle type.

### Boards without existing ArduPilot firmware

- For many boards this requires a separate first-time path (external bootloader/factory path), not the same normal firmware upload flow.
- The docs point to first-time firmware pages for these stacks.

### Advanced/custom firmware updates

- `Load custom firmware` works with `.apj` artifacts.
- Custom firmware server path is documented for feature-specific builds.
- SD-card based boot/update is board-specific and covered in the same docs.

Notes:

- `Pixhawk`/`F7/H7` workflows should be handled carefully with protocol selection and driver state.
- `Load Custom Firmware` is not the normal path for most users.

---

## 🧪 Mandatory and optional setup (what you must do before flight)

`SETUP -> Mandatory Hardware` contains the core pre-flight requirements.

- Accelerometer calibration
- Compass setup (vehicle-dependent)
- Radio calibration
- Servo output mapping
- ESC calibration path (Copter path differs from Plane/Rover behavior)
- Flight mode mapping
- Failsafe pages

`SETUP -> Optional Hardware` is where non-mandatory but highly practical items are configured.

- Telemetry radio tuning, UAVCAN setup, rangefinders
- Integrations for optical flow, OSD, and joystick testing
- Optional `DroneCAN` and `Motor Test` operations for specific stacks

`CONFIG -> Full/Standard Params` is the core place for deeper tuning and param restore workflows.

---

## 🧭 Mission planning capabilities

Mission Planner’s `PLAN` workflow supports direct, map-driven mission authoring and execution prep.

- Manual waypoint entry from map UI
- Right-click mission manipulation
- Upload/download to/from autopilot
- Mission file import/export (`kml`, `shp`, and waypoint formats)
- `GeoFence` creation and upload/read workflows
- `Rally` points creation and management
- Survey/grid generation from map polygons

Useful mission-adjacent details:

- Mission Planner’s plan module is not just “path drawing” — it also tracks return altitude contexts, plan metadata, and includes helper features for repeated mission design.
- The main mission concepts apply to other GCS as mission-item operations at protocol level remain MAVLink-based.

---

## 📊 Live data, modes, and controls

`Flight DATA` is the operational center for real-time checks.

Common controls:

- HUD and map overlays
- Quick mode changes (including arm/disarm in test contexts)
- Mode switching and action buttons
- Status, parameters, and messages
- Mission restart and control mode transitions
- Telemetry log download start/inspect cycle

This screen is useful for both bench checks and field validation, because it centralizes command, telemetry, and log access in one place.

---

## 🧾 Logging and diagnostics

There are two log families in practice.

- `tlogs` (telemetry logs): generated while connected to Mission Planner; useful for message-level replay.
- `DataFlash` (autopilot logs): typically stored onboard and downloaded after a flight.

Important implications:

- Telemetry logs are automatically written from MAVLink stream timing and include what comes through the configured telemetry rate.
- DataFlash files can be replayed for deeper diagnosis.
- Replay-oriented workflows benefit from explicit logging flags and rate control to reduce storage pressure and improve determinism.

---

## 🧪 SITL and test loops

Mission Planner includes a built-in simulation tab that uses the same SITL models as ArduPilot’s Linux `sim_vehicle.py` flow.

- It is useful for parameter iteration and mission sanity checks before hardware test.
- The X-Plane-specific flow is useful for fixed-wing and helicopter model coverage in docs-era workflows.
- For broader experimentation, ArduPilot docs point to using SIM from command-line + preferred GCS as an alternative.

Use this flow if your goal is:

- Fast tuning and mission behavior checks
- Verifying failsafe and mission transitions
- Reproducing regressions without hardware risk

Avoid using it as your only validation source before real radio/power-chain checks.

---

## ⚖️ Compared with other ArduPilot operators (short matrix)

| Tool | Platform focus | Mission planning UX | Logging depth | Why use it |
|---|---|---|---|---|
| Mission Planner | Windows-first (Mono workaround on Linux) | Full map-based mission/fence/rally workflow | Strong on GCS-timing and quick review | Mainstream ArduPilot-first operator tool |
| QGroundControl | Cross-platform in typical use | Good cross-stack mission workflow | Good for general telemetry and mission ops | Useful when platform diversity matters |
| MAVProxy | Cross-platform | Minimal native point-and-click; command and script-oriented | Strong for repeatable script-driven command flows | Better for headless or CI-like workflows |
| APM Planner 2 | Legacy companion | Mission planning available | Niche legacy support | Useful for older pipelines and legacy docs references |
| MAVSDK / `pymavlink` scripts | API/host-driven | API-driven mission commands | Fully custom | Better for deterministic external autonomy orchestration |

---

## ✅ When Mission Planner is the right choice

- You need fast mission/procedure setup with GUI workflows.
- You are on Windows and want immediate board-to-board calibration and tuning loops.
- You need mission + fence + rally pathing quickly before porting to heavier automation.
- You need broad visibility during first integration pass (HUD, params, modes, logs).

## ⚠️ When to avoid it

- You need fully stable headless Linux automation.
- You need a pure script-first production GCS.
- You need strict CI reproducibility from the ground station layer.
- You need to avoid Windows-only dependencies.

---

## 🔗 Related notes

- [[ArduPilot]]
- [[SITL]]
- [[Brain-in-the-Loop Flight Sim]]
- [[ArduPilot vs Pixhawk]]
- [[Pixhawk]]
- [[MAVLink]]

---

## 📚 Sources (official + repo docs)

- ArduPilot Mission Planner overview: `https://ardupilot.org/planner/docs/mission-planner-overview.html`
- Installing Mission Planner: `https://ardupilot.org/planner/docs/mission-planner-installation.html`
- Connect Mission Planner to AutoPilot: `https://ardupilot.org/planner/docs/common-connect-mission-planner-autopilot.html`
- Loading firmware onto boards with existing ArduPilot: `https://ardupilot.org/planner/docs/common-loading-firmware-onto-pixhawk.html`
- Loading firmware onto chibios-only boards (first-time-only path): `https://ardupilot.org/planner/docs/common-loading-firmware-onto-chibios-only-boards.html`
- Initial setup and mandatory configuration: `https://ardupilot.org/planner/docs/mission-planner-initial-setup.html`
- Configuration and tuning: `https://ardupilot.org/planner/docs/mission-planner-configuration-and-tuning.html`
- Flight DATA screen: `https://ardupilot.org/planner/docs/mission-planner-flight-data.html`
- Flight PLAN screen: `https://ardupilot.org/planner/docs/mission-planner-flight-plan.html`
- Telemetry log notes: `https://ardupilot.org/planner/docs/mission-planner-telemetry-logs.html`
- Log download and replay: `https://ardupilot.org/planner/docs/common-downloading-and-analyzing-data-logs-in-mission-planner.html`
- Mission Planner simulation mode: `https://ardupilot.org/planner/docs/mission-planner-simulation.html`
- X-Plane + Mission Planner SITL entry: `https://ardupilot.org/dev/docs/sitl-with-xplane.html`
- ArduPilot HITL list for broader test architecture: `https://ardupilot.org/dev/docs/hitl-simulators.html`
- Mission Planner feature index: `https://ardupilot.org/planner/docs/mission-planner-features.html`
