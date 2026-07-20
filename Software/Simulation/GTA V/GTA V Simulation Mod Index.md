# GTA V Simulation Sandbox for Robots, UGVs, RC, and Driving Research

Use this note as your project index for running robotics/controls work inside GTA V before moving into heavier toolchains.

It groups content into:

- `SP-ready`: Existing GTA V SP mods/scripts you can install in a clean install.
- `Server-framework`: FiveM-style resources (good for multi-entity experiments).
- `Concept`: Project ideas to implement or evaluate.

---

## 0) Why this folder is useful

GTA V has a rich, believable traffic/world simulation and camera stack, but is also scriptable enough to support repeatable control experiments.

Good sandbox targets here:

- **Ground autonomy** (UGV, self-driving control, waypoint following)
- **Aerial control** (FPV, quad/helicopter drones, helicopter logic)
- **RC-style command interfaces** (low-speed precision and stateful control loops)
- **Resource loops** (farming/industry style planning tasks)
- **Control-theory validation** (latency, stability, fail-safes)

---

## 1) Existing mods and frameworks by category

### 1.1 Ground automation / self-driving stack

| Label | Item | Type | Notes | Link |
|---|---|---|---|---|
| SP-ready | **DeepGTAV** | `.asi` plugin + TCP JSON server | Easiest practical vision/telemetry + control baseline for car research. | [GitHub](https://github.com/aitorzip/DeepGTAV) |
| SP-ready | **VPilot** | Python bridge/tooling for DeepGTAV | Removes protocol glue for quick scripts and experiments. | [GitHub](https://github.com/aitorzip/VPilot) |
| SP-ready | **Sentdex/pygta5** | Multi-process trainer/collector/player stack | Useful template for data collection + centralized training. | [GitHub](https://github.com/Sentdex/pygta5) |
| SP-ready | **GANTheftAuto** | Custom mod + Python data/control loop | Good for dataset-style steering baselines and offline behavior. | [GitHub](https://github.com/Sentdex/GANTheftAuto) |
| SP-ready | **VAutodrive** | Assisted driving mod | Useful baseline comparator for controller stability/lag. | [gta5mod.net](https://gta5mod.net/gta-5-mods/scripts/vautodrive-v8-0-6/) |
| SP-ready | **Tesla Autopilot / FSD mod** | Assisted in-game auto-driving script | Useful “legacy autopilot” reference baseline. | [GTA5-Mods](https://www.gta5-mods.com/scripts/tesla-autopilot-full-self-driving) |
| SP-ready | **GTA5Telemetry** | Telemetry extractor | Valuable pattern for low-latency state export and dashboard integration. | [GitHub](https://github.com/TGDSimware/GTA5Telemetry) |

### 1.2 Aerial / FPV / drone-like

| Label | Item | Type | Notes | Link |
|---|---|---|---|---|
| SP-ready | **Quadcopter-Redux** | GTA V plugin script | Best practical FPV sandbox with flight-style control and mission behavior. | [GTA5-Mods](https://www.gta5-mods.com/scripts/quadcopter-redux), [Guide](https://github.com/fredakilla/Quadcopter-Redux-Guide/blob/main/ReadMe.md) |
| Server-framework | **nzkfc_drone** | FiveM Lua resource | Useful for server/client command-state architecture and companion behavior. | [GitHub](https://github.com/nzkfc/nzkfc_drone), [Forum](https://forum.cfx.re/t/nzkfc-drone-a-companion-drone-script/5389587) |
| Server-framework | **fivem-drone** | FiveM resource (QB/ESX conversions) | Good for testing mission/state patterns in an online-style sandbox. | [GitHub](https://github.com/frebespinal/fivem-drone) |
| Server-framework | **HeliCam** | FiveM camera/flight utility | Useful reference for stable POV and camera/target orchestration. | [GitHub](https://github.com/ryans1230/HeliCam) |
| Server-framework | **D_Hover** | FiveM helicopter hover script | State-machine-heavy control pattern to port ideas from. | [Forum](https://forum.cfx.re/t/d-hover-helicopter-hover-script/5215613) |

### 1.3 RC / remote-ground behavior

| Label | Item | Type | Notes | Link |
|---|---|---|---|---|
| Server-framework | **RC car scripts** | FiveM scripts / ESX-QB variants | Good for command loops and small-vehicle behavior state machines. | [Forum 1](https://forum.cfx.re/t/rc-car-script/525015), [Forum 2](https://forum.cfx.re/t/rc-car-esx-qb/5349084) |
| Server-framework | **RC hobby variants** | Script-resource packs | Useful UX examples for gamepad/command wrappers. | [Forum 1](https://forum.cfx.re/t/patamods-rc-car-script-standalone/5041329), [Forum 2](https://forum.cfx.re/t/hd-rccar-realistic-hobby-script/5343633) |

### 1.4 Farming / long-horizon simulation

| Label | Item | Type | Notes | Link |
|---|---|---|---|---|
| SP-ready | **Farming Life Project** | Script mod | Good for long-duration mission cadence and state transitions. | [GTA5-Mods](https://www.gta5-mods.com/scripts/farming-project-mod) |
| SP-ready | **FarmWeed** | Script mod | Clear crop growth/harvest/sell cycle for planning experiments. | [GTA5-Mods](https://www.gta5-mods.com/scripts/farmweed-1-0-legacy-based-on-plantationv) |
| SP-ready | **PlantationV** | Script mod (.NET) | Useful object/lifecycle handling for world mutation. | [GTA5-Mods](https://www.gta5-mods.com/scripts/plantationv-plant-and-sell-drugs-net) |
| SP-ready | **Lua Gardener** | Lua script mod | Minimal world-interaction starter for interaction-state design. | [GTA5-Mods](https://www.gta5-mods.com/scripts/lua-gardener-mod) |
| SP-ready | **Farim** | Simulation-style RPG + farming script | Good for resource logistics/progression loops. | [GTA5-Mods](https://www.gta5-mods.com/scripts/farim-mod) |

### 1.5 Runtime stack you keep in your install/toolchain

| Component | Role |
|---|---|
| `dinput8.dll` + ScriptHookV | ASI/plugin injection entry point |
| ScriptHookVDotNet | .NET script execution route |
| OpenIV | Asset and archive workflows (`.rpf`) |
| FiveM server | Isolated server boundary for multiplayer-style experiments |
| [[ASI Files]] note | Native file/loader internals for plugin behavior |

---

## 2) Project ideas (ready-to-build sandbox tasks)

Each idea below is a concrete project plan you can run and show off.

### 2.1 UGV-style waypoint stack

- **Category:** Ground automation
- **Status:** Concept
- **Start:** DeepGTAV + VPilot + Python control loop
- **Idea:** World-to-vehicle waypoint conversion, PID/PD mission behavior, recovery mode.
- **Validation:** Track completion, obstacle recovery, telemetry dropout tolerance.

### 2.2 Self-driving baseline vs learned controller

- **Category:** Ground automation
- **Status:** Concept
- **Start:** DeepGTAV as environment + custom controller (or VPilot helper)
- **Idea:** Compare handcrafted controller and learned policy in same track set.
- **Validation:** RMS lateral error, overshoot, throttle smoothness, recovery count.

### 2.3 FPV + controller benchmark

- **Category:** Aerial
- **Status:** Concept
- **Start:** Quadcopter-Redux + external bridge process
- **Idea:** Run same controller in two modes: human input baseline and closed-loop policy.
- **Validation:** Stability margins, oscillation, fail/recover timing.

### 2.4 RC plane/helicopter behavior prototype

- **Category:** Aerial + RC
- **Status:** Concept
- **Start:** Use drone/RC behavior patterns from FiveM repos for command/state architecture.
- **Idea:** Implement takeoff-climb-orbit-return mission logic with configurable waypoints.

### 2.5 Farming logistics autopilot

- **Category:** Farming / resource planning
- **Status:** Concept
- **Start:** Farim/FarmWeed as behavior seed
- **Idea:** External planner handles route scheduling, harvesting timing, and inventory costs.
- **Validation:** Throughput, missed windows, reward metric from event logs.

### 2.6 GTA V telemetry broker for PX4 / ArduPilot

- **Category:** Controls / flight stack integration
- **Status:** Concept
- **Start:** GTA5Telemetry or custom plugin export, python/C++ bridge, PX4/ArduPilot SITL.
- **Idea:** Treat GTA V as synthetic world that publishes MAVLink-compatible vehicle state.

---

## 3) ArduPilot / PX4 bridge sketch

Goal: make external flight stacks think GTA V is a sim sensor/plant.

### 3.1 Data path

- GTA-side plugin exports: position/velocity/heading, wheel/rotor-like actuation states, collision status.
- Bridge converts frames and units into SITL message rates and publishes MAVLink:
  - `LOCAL_POSITION_NED`
  - `ATTITUDE`
  - `RAW_IMU`
  - `SYS_STATUS`
  - `SET_POSITION_TARGET_*` / equivalent setpoints back into GTA

### 3.2 Safety envelope

- Enforce timeout watchdogs (drop to manual-safe state on stale data).
- Clamp command rates and acceleration to avoid unstable spikes.
- Log packet timing and replay for debugging.
- Start in SP mode only; reserve FiveM for multi-actor experiments.

### 3.3 Why this is valuable

- Lets you test stack-level controller behavior before real hardware.
- Lets you reuse controller code and compare against the same mission scripts across simulators.
- Keeps project progression incremental: SP `.asi` tests → bridge validation → SITL loop.

Relevant example pattern: [PX4XPlane adapter](https://github.com/alireza787b/px4xplane), [ArduPilot docs](https://ardupilot.org/).

---

## 4) Links to keep open while building

- [[ASI Files]]
- [[GTA V Modding for Simulation]]
- [[TCP]]
- [[Reinforcement Learning]]
- [ScriptHookV](https://www.dev-c.com/gtav/scripthookv/)
- [ScriptHookV critical errors](https://scripthookv.io/script-hook-v-critical-error/)
- [ScriptHookVDotNet](https://github.com/scripthookvdotnet/scripthookvdotnet)
- [OpenIV](https://openiv.com/)
- [FiveM Docs](https://fivem.net/documentation/)
- [MAVLink](https://mavlink.io/en/)

## 5) Before you start

- Use single-player SP install for `.asi` experimentation.
- Keep one unknown mod at a time and verify launch stability.
- Start with a tiny command API (`go/stop`, `set_speed`, `set_heading`) and expand only after logs are stable.
- Record all telemetry and action packets before scaling scenario complexity.
