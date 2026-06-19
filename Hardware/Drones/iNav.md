# iNav

`iNav` is an open-source flight firmware focused on GPS-assisted and mission-capable flight workflows for lightweight UAVs, with a strong position toward fixed-wing and hybrid platforms.

This note is for cross-linking decisions inside the vault, not an official certification guide.

---

## 🧠 What iNav actually is

- `iNav` is a firmware stack that adds navigation primitives (position hold, altitude hold, return to home, and waypoint modes) on top of ACRO / attitude-control behavior.
- Configuration and runtime control are handled primarily through the `INAV Configurator` (`Windows`, `macOS`, `Linux` supported).
- The same project is also linked with `MSP`-style communication patterns; this is used for configurator control and external tooling.
- As of `INAV 9.0.1` and related documentation, the project describes itself as navigation-focused and open-source, with emphasis on practical GPS/autopilot behavior for hobby/compact platforms.

---

## 🔧 Core capabilities and boundaries

### Navigation features you should expect

- Position hold and altitude hold behavior
- RTH
- Waypoint mission modes (feature-set in docs and community summaries)
- Fixed-wing and multirotor support across supported targets
- Optional advanced features: camera control, lidar/sonar input support, blackbox logging, logic functions/actions

### What is commonly misunderstood

`iNav` is often grouped with `Betaflight` because both are common in RC workflows.  
In practice, `iNav` is not “just tuning-focused”: it is the one that adds flight-stack-level navigational behavior at the FC level.

---

## 🚀 Compatibility and stack role

| Stack | Core focus | Mission behavior | Vehicle bias | Typical ecosystem fit |
|---|---|---|---|---|
| `iNav` | Navigation-focused FC firmware | Position hold / RTH / waypoint modes on board | Fixed-wing + multirotor + small craft | RC-style setup + autonomy-lite workflows |
| `ArduPilot` | Full autopilot stack | Richest mission toolchain + broad ground-station workflows | Fixed-wing / copter / rover / boat / etc. | Research + production pipelines, better official sim/HIL depth |
| `PX4` | Modular autopilot + ROS2 integration | Modern offboard and mission APIs via MAVSDK/ROS2 patterns | Multicopter + fixed-wing + VTOL | Modern autonomy + open toolchain integration |
| `Betaflight` | Aggressive acro/FPV stabilization | Minimal mission behavior; no comparable onboard wayfinding stack | Quadcopter / acro builds | RC racing/freestyle first |
| `Mission Control (external)` | Companion-level policy control | Stack-agnostic control overlays and telemetry bridges | Depends on selected FC | Good when FC remains simple and companion does policy |

---

## 🔧 Installation and upgrade flow (Linux + Windows)

The official docs present a generic, GUI-first flow:

### 1) Install `INAV Configurator`

- Use an official release for your platform (`Windows`, `macOS`, `Linux`).
- If using browser-based legacy workflows, use a modern Chromium-compatible launcher path as described in historical docs.

### 2) Connect and identify the FC

- Connect the board over USB.
- Ensure correct serial port/connection is selected in configurator.

### 3) Flash firmware

- Open `Firmware Flasher`.
- Use `Load Firmware [Online]`.
- Choose the latest stable branch matching your board.
- When flashing first time, enable `Full Chip Erase` if available.
- Flash and confirm `"Programming: SUCCESSFUL"`.

### 4) Backup/restore and upgrade notes

`iNav` users are repeatedly warned that releases can be incompatible across major versions.

- Backup before upgrade.
- For upgrades from older major versions, expect migration behavior.
- The official docs stress release-note review and backup handling.

---

## 🧪 iNav and simulators (SITL / X-Plane) in practice

There are *community* examples of `iNav` + X-Plane workflows, but this remains less formal than `ArduPilot`-documented sim integration.

### Community X-Plane command sample

From an `iNav` discussion thread:

```bash
inav_SITL --sim=xp --simip=192.168.0.162 --simport=49000 --chanmap=S01-01,S02-02,S03-03,M01-04
```

This pattern shows:
- simulator traffic over UDP into `inav_SITL`
- separate channel mapping for serial/RC style control translation

### Practical interpretation

- If your use case is official repeatability, prefer `ArduPilot + SITL` paths first.
- `iNav` is useful as an alternate comparator stack when you are specifically testing:
  - fixed-wing sensor/flight-envelope behavior
  - command bridge tolerance
  - low-latency telemetry/control loops before full migration.

### Caveat

There are still project/issue-level reports where command-output paths did not drive simulator visuals as expected. So the safe mental model is:
- telemetry ingestion into `inav_SITL` works in practice,
- actuator-command reliability and simulator response can require additional tuning.

---

## 🧰 Protocols and interfaces

`iNav` docs and related tools repeatedly include:
- MSP as the primary remote-management and configurator protocol
- support for telemetry transport types including SmartPort/FPort/MAVLink/LTM/CRSF (varies by build)
- CLI hooks for deep tuning and automation

For deep loop integrations, the common split is:

- `MSP` for FC management and RC-style action channels
- companion-side policy in Python/other bridge code for high-level commands
- MAVLink only where explicitly enabled on compatible setups

---

## ✅ iNav vs ArduPilot/Pixhawk in the Brain-in-the-Loop path

In the “brained controller sending setpoints back into a simulator and letting autopilot enforce policy” pattern:

- `ArduPilot` and `PX4` generally have stronger official docs around repeatable `SITL/HITL` and GCS integration.
- `iNav` can be used for similar research if your goal is specific fixed-wing behavior and you are comfortable validating unsupported glue code.
- If this is for broad mission-critical loops (multi-mode mission, deterministic resets, large param surfaces), treat `iNav` as a *prototype comparator* not a guaranteed production control target.

---

## ⚖️ Pros and cons for research workflows

### Strengths

- Great fixed-wing/autonomy-first behavior without a heavy software stack
- Good path for pilots transitioning from manual modes to autonomous primitives
- Good fit for compact sensor setups and hobby hardware budgets
- MSP ecosystem means tooling and telemetry entry points are straightforward for experimentation

### Weaknesses

- Officially less deterministic simulator bridge coverage than `ArduPilot/PX4`
- Hardware support is target-dependent; certain MCUs and old GPS protocol support are de-emphasized in newer versions
- Mission/advanced autonomy depth is narrower than enterprise-grade stacks

---

## 📊 Decision matrix

| Need | Choose iNav | When not to choose iNav |
|---|---|---|
| Fixed-wing with waypoint/autonavigation | ✅ | If you need full ROS2/multi-vehicle mission orchestration |
| Lightweight FC + companion ML loop | ✅ | If you require official dense sim/HIL workflows |
| Beginner-friendly hobby autonomy | ✅ | If your stack needs extensive sensor abstraction and middleware layers |
| Heavy terrain / fleet scale experiments | ⚠️ | If you need large-scale deterministic benchmarking |
| Cross-domain vehicle diversity (rover, boat, etc.) | ⚠️ | If you need same-stack support across many vehicle classes |

---

## 🔗 Related notes

- [[ArduPilot vs Pixhawk]]
- [[Brain-in-the-Loop Flight Sim]]
- [[CRSF]]
- [[MAVLink]]
- [[PufferLib Flight Sim Integration Gaps]]

## 📚 External resources

- INAV repository and release artifacts: https://github.com/iNavFlight/inav
- INAV Configurator: https://github.com/iNavFlight/inav-configurator
- INAV official docs landing page: https://inavflight.github.io
- Feature and mode docs: https://inavflight.github.io/docs/features/Modes/
- GPS/Compass notes for INAV navigation behavior: https://inavflight.github.io/docs/quickstart/GPS-and-Compass-setup/
- Remote management and telemetry overview: https://inavflight.github.io/docs/advanced/INAV-Remote-Management%2C-Control-and-Telemetry/
- Community X-Plane + iNav discussion with command example: https://github.com/iNavFlight/inav/discussions/9246
- Companion-control issue thread: https://github.com/iNavFlight/inav/issues/9247
- INAV SITL French community report (X-Plane + RealFlight): https://www.helicomicro.com/2023/03/08/inav-sitl-tester-les-fonctions-de-inav-avec-un-simulateur-de-vol/

## 🧷 Source notes

[^inav-docs]: iNav official repository and docs list key firmware capabilities, supported FC targets, and setup flows, including releases and configurator downloads.
[^inav-sitl-discussion]: Community X-Plane `inav_SITL` command flow and integration details are from the 2023 `iNav` GitHub discussion for `inav_SITL --sim=xp`.
[^inav-sitl-issue]: The related issue thread reports control-command edge behavior while telemetry and simulator data path appear active.
[^inav-gps]: INAV GPS page documents supported protocol posture and Ublox protocol version guidance for newer releases.
