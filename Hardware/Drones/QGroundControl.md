# QGroundControl

`QGroundControl` (`QGC`) is a cross-platform ground control station used across `ArduPilot` and `PX4` stacks for connect-setup-fly workflows, mission planning, parameter tuning, and log review.

It is commonly used when you need one GUI that covers both firmware families with a single app flow.

---

## 🧭 What it is and what it is not

- `QGroundControl` is both a flight controller setup tool and a mission planning/monitoring tool.
- It is **not** only for ArduPilot; official docs position it as a `PX4` and `ArduPilot` supported GUI with broader OS coverage.
- If your pipeline is command-line heavy (CI or fully scriptable operator logic), `[[MAVProxy]]` is often a better core loop than QGC.

---

## 💻 Install and platform expectations

- Linux and desktop-first workflow (`Windows`, `macOS`, Linux AppImage).
- Windows:
  - Download from QGC docs (daily or stable link path).
  - Installer creates shortcuts for normal and graphics-mode launches.
- `Ubuntu` desktop workflow requires serial permissions and GStreamer tooling for video pipelines.
- Older Android/iOS controllers are not always supported by latest builds; QGC docs call out `5.0` as the last legacy branch for older Android devices.

Example Linux user setup steps from docs:

- `sudo usermod -aG dialout "$(id -un)"`
- Re-login after updating group membership so serial permissions are active.
- Install required `gstreamer` and related runtime packages for video map overlays/mapsync.
- Prefer Ubuntu 22.04/24.04 style flow for AppImage operation on modern desktop.

For command references, use official docs URLs in the sources section.

---

## ⚙️ Core workflows

### 1) Firmware load and update

QGC desktop can flash `PX4` and `ArduPilot` to Pixhawk-family boards from one workflow.

Desktop-only limitation:
- Firmware load is not available on tablet/phone builds.

Typical desktop flow:

1. Open the Gear icon (Vehicle Setup) and choose `Firmware`.
2. Connect board via direct USB (not through a powered hub).
3. Select stack (`PX4` / `ArduPilot`) and vehicle type (for ArduPilot path).
4. Start and let progress bar complete; board reboots on success.

### 2) Vehicle configuration

Vehicle Configuration opens required calibration and setup pages:

- Airframe / frame
- `Radio`
- `Sensors` (compass/gyro/accel)
- `Flight Modes`
- `Failsafes`, `Safety`
- `Parameters`
- `Servo Outputs` (ArduPilot view), etc.

QGC shows red indicators on unconfigured blocks before flight.

### 3) Mission planning

Plan View handles waypoint and mission item authoring with:

- `Takeoff`, `Waypoint`, `RTL`/Return patterns, `ROI`, and mission sequencing layers.
- `GeoFence` and `Rally` points when firmware supports them.
- Save/Upload workflow with map-based editing and on-screen mission stats.
- KML export and mission file round-trips depending on selected firmware.

### 4) Connect and fly

QGC auto-connect behavior is standard for common serial and wireless links.

Typical link methods:

- USB
- Telemetry radio
- Wi-Fi / Ethernet (including companion-computer forwarded endpoints)

If a vehicle fails to auto-appear, troubleshooting path is to check connection settings (or create a manual `Comm Link`) and review log hints from app settings.

---

## 🧪 In the dev/sim loop

- QGC is often a secondary station for ArduPilot workflows while `[[MAVProxy]]` or scripts own deterministic command flow.
- In `ArduPilot` simulation pipelines, QGC is supported as a client when the sim output stream is available on a local link and firmware build is matched to target stack.
- For mission-centric experimentation, QGC is especially useful to validate:
  - mission timing
  - fence/rally geometry
  - preflight configuration states
  - recovery behavior while watching telemetry + HUD state

---

## ⚖️ Comparison (brief)

| Tool | Main strength | Weakness vs. neighbors |
|---|---|---|
| `QGroundControl` | Cross-stack GUI for setup + mission + log flow | Fewer scripting hooks than CLI-first stacks |
| `[[ArduPilot Mission Planner]]` | Mature Windows-first ArduPilot workflow | Windows bias / less native cross-stack ergonomics |
| `[[MAVProxy]]` | Fully scriptable CLI, lightweight bridging and forwarding | No primary GUI mission editor |
| `MAVSDK` | API-driven external control/orchestration | Not a direct replacement for full GCS UI |
| `[[ArduPilot Mission Planner]]` | Fast ArduPilot parameter and flight operations UI | ArduPilot-centric and Windows-first bias |
| `[[QGroundControl]]` (mobile/tablet variants) | Portable field use and review | Firmware upload is desktop-only |

---

## ✅ When to use QGC

- You want a single app for multiple stacks (`PX4` + `ArduPilot`) and mission workflows.
- You want GUI-first setup/checklist behavior.
- You need mapping and mission editing together with telemetry/live status.

## ⚠️ When it is less ideal

- You need heavy CLI automation or reproducible command-first tuning loops.
- You need a pure Linux-server/daemon model for headless repeated integration loops.
- You rely on old mobile OS targets that require pre-5.0 branch behavior.

---

## 🔗 Related notes

- [[ArduPilot]]
- [[ArduPilot Mission Planner]]
- [[MAVProxy]]
- [[MAVLink]]
- [[PX4]]
- [[SITL]]
- [[Brain-in-the-Loop Flight Sim]]

---

## 🗂 Sources (official)

- https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/download_and_install.html
- https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/firmware.html
- https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/setup_view.html
- https://docs.qgroundcontrol.com/master/en/qgc-user-guide/plan_view/plan_view.html
- https://docs.qgroundcontrol.com/master/en/qgc-user-guide/getting_started/quick_start.html
- https://docs.qgroundcontrol.com/v5.0/en/qgc-user-guide/getting_started/quick_start.html
- https://docs.qgroundcontrol.com/Stable_V5.0/en/qgc-user-guide/getting_started/quick_start.html
- https://www.ardupilot.org/planner/docs/mission-planner-simulation.html
