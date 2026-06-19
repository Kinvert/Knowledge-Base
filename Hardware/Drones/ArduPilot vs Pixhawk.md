# ArduPilot vs Pixhawk

This note is a direct comparison for one purpose: `ArduPilot` and `Pixhawk` are often treated like competitors, but they are not the same abstraction layer.

`ArduPilot` is a full autonomous flight/control stack.

`Pixhawk` is primarily a hardware family / open-hardware ecosystem and a supported target for multiple stacks.

`MAVLink` is the common protocol layer that lets many stacks and ground tools talk consistently.

The strongest practical decision is usually not *ArduPilot vs Pixhawk*, but *ArduPilot with which board* or *PX4 with which board*.

For board-level selection, use: [[ArduPilot Boards]] and [[PX4 Boards]].

---

## 🧠 Mental model before selection

### 1) Abstraction layers

| Layer | What it is | Common choice |
|---|---|---|
| Protocol | Cross-stack message transport | `[[MAVLink]]` |
| Hardware | Flight-controller design, power, pin mapping, connectors | `[[Pixhawk]]` (or non-Pixhawk FC) |
| Autopilot software | Vehicle control, mission behaviors, fail-safes, modes | `[[ArduPilot]]`, `[[PX4]]`, other stacks |
| Tooling | Ground stations, logs, flashing, tuning | Mission Planner, `[[QGroundControl]]`, MAVProxy |
| Task logic | Companion autonomy, perception stack, higher control | ROS, custom python/C++ services |

### 2) Canonical confusion

- The statement “ArduPilot or Pixhawk” is frequently wrong when you mean `ArduPilot firmware` and `Pixhawk hardware`.
- Correct framing is:
  - `ArduPilot + Pixhawk Family`
  - `PX4 + Pixhawk Family`
  - `ArduPilot + non-Pixhawk`
  - `PX4 + non-Pixhawk`
- If your mental model is `Pixhawk vs ArduPilot`, you will pick the wrong board or miss tuning requirements.

---

## ✈️ Aircraft and vehicle type support

`ArduPilot` explicitly publishes a broad vehicle-type list including fixed-wing, copters, rovers, boats, submarines, and more in its vehicle docs. The docs include an “almost every vehicle imaginable” framing with explicit frame examples.  
`PX4` docs position it as a modular stack for multirotors, fixed-wing, VTOL, rovers, and additional platforms, including support for many vehicle extensions.

### Vehicle comparison

| Vehicle / platform | ArduPilot position | PX4 position | Why it matters |
|---|---|---|---|
| Multicopter | Supported as a first-class family with broad frame taxonomy | Supported with strong ecosystem tooling | Both are practical for copters; ArduPilot often wins on legacy feature breadth, PX4 on middleware clarity |
| Fixed-wing | Supports planes and tailsitter/quadplane combinations | Supports fixed-wing and VTOL/hybrid families | ArduPilot has explicit quadplane-style experimental support language; PX4 supports mission-mode fixed-wing modes |
| Rovers | Dedicated `[[Rover]]` stack with autonomous mission execution and vehicle-specific behavior | Rover support exists in PX4 docs and mode taxonomy | ArduPilot has stronger historical `Rover` focus; PX4 is viable but often chosen for ROS2-centric stacks |
| Boats / marine | Support appears in ArduPilot docs via vehicle coverage | PX4 indicates broad experimental support path | For marine-heavy projects, source-specific controller hardware matters more than stack reputation |
| Tractors / mowers | Explicitly called out in use-case docs (autonomous mowers, tractors) | Not equally foregrounded in public docs for agriculture-specific mowers | ArduPilot currently has cleaner direct references for ground automation in that class |

Sources for vehicle framing:
- ArduPilot vehicle overview: `https://ardupilot.org/ardupilot/docs/common-all-vehicle-types.html`
- Rover use-cases (including autonomous mowers): `https://ardupilot.org/rover/docs/common-use-cases-and-applications.html`
- PX4 platform positioning and ecosystem: `https://docs.px4.io/main/`
- PX4 flight-controller hardware ecosystem: `https://docs.px4.io/main/en/flight_controller/pixhawk_series`

---

## 🗺️ Mission planning and autonomous flight

### ArduPilot approach

- Mission planning is treated as a primary workflow in vehicle docs, especially for `AUTO` and mission item operations.
- The mission planning docs apply across copter/plane/rover workflows and include auto mode flow + mission items/rally concepts.
- Ground tooling commonly used: `[[Mission Planner]]`, `MAVProxy`, and `[[QGroundControl]]` as shared MAVLink tools.
- Vehicle command coverage for missions is tied to `MAV_CMD` message support in ArduPilot MAVLink command tables.

### PX4 approach

- PX4 mission mode is explicit: mission plans are uploaded and executed from a GCS, commonly QGC.
- PX4 has dedicated mission command tables with caveats around command support per mode.
- PX4 offboard mode is the “external brain” path and requires reliable setpoint streaming (>2 Hz in most modes).
- ROS2/DDS integration enables deeper companion-computer co-processing for autonomy.

### Mission/autonomy comparison

| Dimension | ArduPilot | PX4 |
|---|---|---|
| Mission creation workflow | GCS + mission item commands, mission planning sections in docs | QGC mission builder + mission command tables |
| Ground tool preference | Mission Planner and MAVProxy are common; QGC also supported | QGC first-class; ROS2 and MAVSDK often used for advanced autonomy |
| Mission execution mode names | Auto modes within vehicle-specific stack | AUTO/mission modes with explicit command acceptance tables |
| External control path | MAVLink + companion stack, MAVProxy/MAVROS ecosystems | MAVLink, `MAVSDK`, ROS2 offboard, DDS |
| API ergonomics for offboard | Strong MAVLink ecosystem, less “single official SDK surface” in docs compared with PX4 | Official first-class stack docs around MAVSDK and ROS2 pipelines |

Sources:
- ArduPilot mission planning: `https://ardupilot.org/rover/docs/common-mission-planning.html`
- ArduPilot mission command list (MAV_CMD): `https://github.com/ArduPilot/ardupilot_wiki/blob/master/common/source/docs/common-mavlink-mission-command-messages-mav_cmd.rst`
- PX4 mission overview: `https://docs.px4.io/main/en/flying/missions`
- PX4 mission mode (Rover / Multicopter examples): `https://docs.px4.io/main/en/flight_modes_rover/auto.html`, `https://docs.px4.io/main/en/flight_modes_mc/mission`
- PX4 offboard: `https://docs.px4.io/main/en/flight_modes/offboard.html`
- QGC mission features: `https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/firmware.html`

---

## 🎮 Simulation and “brain + loop” integration with X-Plane and other sims

### ArduPilot X-Plane path

`ArduPilot` has official X-Plane integration docs and describes network-based data exchange for SITL.

- Setup starts with X-Plane UDP data output and a shared port model.
- `sim_vehicle.py` can launch X-Plane-connected SITL sessions for development.
- `MAVProxy` can forward telemetry to multiple GCS sessions.

### Typical command path (ArduPilot + X-Plane)

```bash
modules/waf/waf-light configure --board sitl
modules/waf/waf-light plane
build/sitl/bin/arduplane --model xplane
sim_vehicle.py -v ArduPlane --model xplane --console --map
```

### ArduPilot simulator range (as docs enumerate)

- Native SITL environment.
- X-Plane path via `sim_vehicle.py`.
- ROS integration notes (example launch via MAVROS in docs).
- Gazebo plugin path and RealFlight/JSBSim/other backends are mentioned in simulator references in ArduPilot ecosystem docs.

### PX4 simulator landscape (official)

- `Gazebo` and `jMAVSim` are official reference paths.
- Multi-vehicle options and fidelity trade-offs are documented (`Gazebo` for richer environments, SIH/JMAVSim for lighter loops).
- ROS2 + DDS bridges are explicitly first-class for autonomy and external control testing.

### Simulator comparison by requirement

| Requirement | ArduPilot | PX4 | Practical risk |
|---|---|---|---|
| X-Plane visual realism + custom mission aircraft testing | Explicit X-Plane support path | Not an emphasized primary path in PX4 docs | Use ArduPilot for this specific bridge |
| Rapid control-loop iteration | SITL + `sim_vehicle.py` with MAVProxy workflows | PX4 SIH / Gazebo options depending fidelity | Keep ArduPilot if script/test velocity is top priority |
| ROS2-heavy perception experiments | MAVLink↔ROS docs exist; ROS bridge examples include MAVROS-style flows | Native ROS2 architecture and DDS story is explicit | Pick PX4 when planning deep ROS2 co-development |
| Multi-vehicle sim at scale | Supported in parts; setup details vary per stack/backend | Multi-vehicle sim docs are explicit and scriptable | Prefer PX4 for documented multi-vehicle flows |
| FlightGear/Flight realism | Mentioned via ArduPilot simulator docs/paths | Also appears in PX4 ecosystem docs | Verify model fidelity against exact control task |

Sources:
- `https://ardupilot.org/dev/docs/sitl-with-xplane.html`
- `https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html`
- `https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html`
- `https://ardupilot.org/dev/docs/simulation-2.html`
- `https://ardupilot.org/dev/docs/ros-sitl.html`
- `https://docs.px4.io/main/en/simulation/index`
- `https://docs.px4.io/main/en/simulation/gazebo`
- `https://docs.px4.io/v1.14/en/simulation/jmavsim`
- `https://docs.px4.io/main/en/simulation/multi-vehicle-simulation`

---

## 🧩 Programming languages, APIs, and integration surfaces

### ArduPilot

- Main flight code is C++, with many support tools and scripts in Python.
- `[[MAVProxy]]` is explicitly documented as a developer-friendly MAVLink ground station tool.
- Lua scripting is documented as sandboxed stack-extension behavior for some runtime tasks.
- Typical integrations are via `MAVLink` messages and companions (Python `pymavlink`, ROS bridge examples).

### PX4

- Modular C++ architecture built on uORB + DDS interfaces.
- MAVLink, ROS2 (`uXRCE-DDS`), and MAVSDK are first-class integration points.
- Offboard mode in MAVLink/ROS2 includes heartbeat and setpoint streaming requirements.
- uORB topic model is explicitly described for extension by developers.

### Language/API comparison

| Stack | Core language | Extensions / scripting | API style | Companion workflow |
|---|---|---|---|---|
| ArduPilot | C++ (core), Python tooling | Lua scripting, MAVLink-based scripts | MAVLink commands/messages | MAVProxy/Mission Planner/QGC |
| PX4 | C++ core | uORB module extension, C++ plugins | MAVSDK, MAVLink, ROS2, DDS | ROS2 bridge + MAVSDK + QGC |

### API code anchor points

- ArduPilot MAVLink command docs: `https://github.com/ArduPilot/ardupilot_wiki/blob/master/common/source/docs/common-mavlink-mission-command-messages-mav_cmd.rst`
- ArduPilot Lua scripting: `https://ardupilot.org/dev/docs/common-lua-scripts.html`
- ArduPilot MAVProxy docs: `https://ardupilot.org/mavproxy/index.html`
- ArduPilot pyMAVLink docs: `https://github.com/ardupilot/pymavlink`
- PX4 uORB: `https://docs.px4.io/main/en/middleware/uorb`
- PX4 middleware index: `https://docs.px4.io/main/en/middleware/index`
- PX4 ROS2: `https://docs.px4.io/main/en/ros2/`
- MAVSDK offboard: `https://mavsdk.mavlink.io/v2.0/en/cpp/guide/offboard.html`

---

## 🔩 Hardware vs firmware details

### Pixhawk standard reality

PX4’s Pixhawk docs describe `Pixhawk` as an open-hardware stack with standard-compatible designs and multiple manufacturer variants. The page is explicit that not every “Pixhawk-like” board is equally compliant, and only fully compatible references are maintained with predictable support.

### ArduPilot hardware framing

ArduPilot docs position the project around supported controller boards and emphasize that the same codebase can run across many hardware families via HAL/board definitions.

### Practical board-level constraints to expect

- Firmware-size ceilings matter for feature depth (ArduPilot docs list flash/memory limits and feature restrictions).
- Hardware pinout/serial layouts differ in subtle ways between boards.
- Porting/building new boards requires HW definition and bootloader entry workflows.

### Hardware/software comparison snapshot

| Layer | ArduPilot view | Pixhawk view |
|---|---|---|
| Philosophy | Vendor-flexible stack, board-porting process, HAL-driven porting | Reference hardware lineage with binary-compatibility aims |
| Hardware openness | Supports many vendor boards, includes board-specific support pages | Open-hardware project with defined schematics and reference designs |
| Board onboarding | Flash to supported boards, upload paths documented via Mission Planner and bootloader tooling | QGroundControl can flash PX4 firmware to Pixhawk-family boards |

Sources:
- `https://ardupilot.org/dev/docs/porting.html`
- `https://ardupilot.org/dev/docs/choosing-an-autopilot` (for hardware options context)
- `https://ardupilot.org/dev/docs/common-limited-firmware.html`
- `https://ardupilot.org/planner/docs/common-loading-firmware-onto-pixhawk.html`
- `https://docs.px4.io/main/en/flight_controller/pixhawk_series`
- `https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/firmware.html`

## 🧱 Pixhawk-family board comparison (best → worst)

This is a practical shortlist when you need an ArduPilot/PX4 stack on Pixhawk-family hardware.

| Rank | Board | Approx. price (USD) | Processor (main) | ArduPilot support | PX4 support | Why this sits here |
|---|---|---|---|---|---|---|
| 1 | [[CUAV V5+]] | $289.00 – $458.00 | STM32F765 (main FMU); STM32F100 coprocessor | Fully compatible with PX4 and ArduPilot (docs explicitly state both) | Fully compatible with PX4 and ArduPilot (same docs) | Strongest all-around for serious field platforms: FMUv5 class, higher-spec FMU/IO split, redundant power inputs and extra sensors. |
| 2 | [[Cube Orange+]] | $215.00 | STM32H757 dual-core (M7 + M4) | ArduPilot install docs are explicit; users load ArduPilot firmware on [[Cube Orange+]] as a standard workflow | Not a Pixhawk standard board; PX4 docs classify it as a footprint replacement for Cube Black and require manufacturer support | Best choice when you need carrier-board ecosystem scale and redundancy (triple IMU + 14 PWM) with a manufacturer ecosystem around it |
| 3 | [[Holybro Pixhawk 6X]] | $166.99 | STM32H753 (main FMU) | Supported in Ardupilot 4.5.2 stable and later (rev notes by board target); listed in ArduPilot-supported board list | Supported in PX4 from 1.13.1+, listed in PX4 standard set | Best value modern board: FMUv6X processor class, high IO count, and broad stack support |
| 4 | [[Holybro Pixhawk 6X Pro]] | $644.00 | STM32H753 (main FMU, variant-specific package) | Supported in Ardupilot 4.5.0+ (same target family as 6X) | Supported in PX4 1.14.3+ | Same FMUv6X family as 6X but with 6X Pro enclosure/features and very high price; better for mission-critical industrial budgets than for hobby | 
| 5 | [[Holybro Pixhawk 6C]] / [[Holybro Pixhawk 6C Mini]] | $130.99 – $165.99 | STM32H743 (main FMU) | Supported in ArduPilot 4.2.3+ (supported-firmware page, ArduPilot target path available) | Supported in PX4 1.13.1+ | Slightly lower spec than FMUv6X class, but compact and still capable for 10+ acre movers with fewer integration edge cases |
| 6 | [[Holybro Pixhawk 5X]] | $104.99 | STM32F765 (main FMU) | Listed in ArduPilot “Choosing an Autopilot” Open Hardware choices | PX4 standard autopilot list (FMUv5X entry) | Good budget fallback when you already have a compatible baseboard stack or legacy wiring |
| 7 | [[Holybro Pixhawk 4]] | $124.99 | STM32F765 (main FMU) | Listed in ArduPilot choosing page as an Open Hardware option | PX4 supported as discontinued-but-supported FMUv5 entry | Older generation, highest risk/compatibility friction for new projects despite low price |

### Price snapshot caveats

- These prices are from storefront listings and product pages and can shift by stock, bundle, region tax/shipping, and revision.
- Some stores show sold-out variants; verify your exact revision and whether microSD, baseboard, and harnesses are included before procurement.
- For field platforms, the same board can behave very differently with power module quality, sensor stack, and firmware branch.

Sources added in this section:
- [[CUAV V5+]] price + spec + firmware compatibility: `https://store.cuav.net/shop/v5-controller/`
- [[CUAV V5+]] compatibility text: `https://doc.cuav.net/controller/v5-autopilot/en/v5%2B.html`
- [[Cube Orange+]] product price and specs: `https://www.camflite.com/products/the-cube-orange`
- Cube Orange install flow (ArduPilot): `https://docs.cubepilot.org/user-guides/autopilot/the-cube/setup/firmware/installing-ardupilot`
- Cube Orange in PX4 docs + note about non-standard connector: `https://docs.px4.io/v1.13/en/flight_controller/cubepilot_cube_orange`
- [[Holybro Pixhawk 6X]] technical specification: `https://docs.holybro.com/autopilot/pixhawk-6x/technical-specification`
- [[Holybro Pixhawk 6X]] supported firmware: `https://docs.holybro.com/autopilot/pixhawk-6x/supported-firmware`
- [[Holybro Pixhawk 6X Pro]] supported firmware: `https://docs.holybro.com/autopilot/pixhawk-6x-pro/supported-firmware`
- [[Holybro Pixhawk 6C]] supported firmware: `https://docs.holybro.com/autopilot/pixhawk-6c/supported-firmware`
- [[Holybro Pixhawk 6C]] technical specification: `https://docs.holybro.com/autopilot/pixhawk-6c/technical-specification`
- Holybro Pixhawk product pricing: `https://holybro.com/collections/flight-controllers`
- ArduPilot supported boards list: `https://ardupilot.org/plane/docs/common-autopilots.html`
- PX4 standard board list + discontinued-note context: `https://docs.px4.io/v1.14/en/flight_controller/autopilot_pixhawk_standard`

---

## 🧪 Open vs closed source and governance

| Item | ArduPilot | PX4 / Pixhawk context |
|---|---|---|
| Stack license | GPL-3.0+ (source docs explicitly list GPLv3 line) | BSD-3-Clause for PX4 stack |
| Hardware ecosystem | Open stack with many board vendors, extensive third-party hardware integration | `Pixhawk` open-hardware reference standard and Dronecode ecosystem |
| Governance model | Community + volunteer + maintainers; issue/PR workflow in GitHub | Dronecode/LF hosting model, vendor-neutral release process |
| Mod / project community | ArduPilot and subproject forum + GitHub | PX4 dev call + docs + issue + discord ecosystem |

Sources:
- ArduPilot license and repo: `https://ardupilot.org/dev/`, `https://github.com/ArduPilot/ardupilot`
- PX4 license and stack summary: `https://docs.px4.io/main/`
- PX4 GitHub org: `https://github.com/px4`
- Pixhawk standard docs: `https://docs.px4.io/main/en/flight_controller/pixhawk_series`

---

## 🌾 10+ acre zero-turn lawn mower recommendation (ArduPilot-focused)

If your actual job is large-area zero-turn mowing with reliable auto coverage, these are the defaults I would use with the least false starts.

### Why ArduPilot is usually the default here

- Rover stack explicitly targets ground craft and includes autonomous mission use cases.
- `Rover` docs show agricultural automation use in practical contexts.
- Mower-oriented projects in ArduPilot forums repeatedly use Pixhawk-class hardware with Rover modes.

### Recommended stack for 10+ acre zero-turn

| Requirement | Recommended | Why |
|---|---|---|
| Zero-turn style steering | `ArduPilot Rover` on suitable FC with sufficient UART/PWM I/O | Rover supports skids/steer variants and mission driving workflows |
| Position-critical coverage | RTK-capable GNSS on capable board + tested mission config | ArduPilot forum mower threads repeatedly discuss dual-GNSS and RTK issues |
| Field safety | Geofences, failsafes, stop zones in mission workflow | Mission planning docs + failsafe pages are practical for this flow |
| Perimeter and repeatability | Pre-mapped boundaries + mission planning + path tuning | Mission mode pathing is better documented for direct field ops |
| Minimal integration friction | Standardized boards with strong community examples | Multiple forum examples for mower builds and controller selection |

### Project references for mower workflows

- `ArduPilot Rover` introduction and use-case context: `https://ardupilot.org/rover/index.html`
- Rover use cases (including autonomous mowers/tractors): `https://ardupilot.org/rover/docs/common-use-cases-and-applications.html`
- Rover demo page (driverless tractor, grain cart): `https://ardupilot.org/rover/docs/rover-video-demonstrations.html`
- Mower forum examples:
  - `https://discuss.ardupilot.org/t/mowbius-a-21-electric-push-mower-conversion/124672`
  - `https://discuss.ardupilot.org/t/my-z-turn-mower-project-noodle-brain-54-gravely-here4-w-base-antenna/121488`
  - `https://discuss.ardupilot.org/t/building-a-rover-mower-um982-gps-rtk-inaccuracy/102153`
  - `https://discuss.ardupilot.org/t/kevings-autonomous-zero-point-turn-lawn-mower/29178`

### A practical 0→1 rollout path

1. Choose a Pixhawk-family FC with redundant power and serial headroom.
2. Configure in Rover, verify motor mapping and brake/turn direction on static ground tests.
3. Validate telemetry + failsafes before moving to large fields.
4. Build simple waypoints first, then add edge and obstacle logic.
5. Scale area coverage in small chunks and keep manual override path in place.

---

## 🎮 `PX4` comparison matrix (for reference)

Not always what you asked for, but helpful if you want to avoid overfitting to one stack:

| Stack combo | Best for | Why consider | Risk |
|---|---|---|---|
| ArduPilot + Pixhawk/FC A | Quick mission delivery for many vehicle classes | Large command surface, practical docs for mission-first behavior | GPL obligations, feature matrix differs by flash |
| ArduPilot + Linux SBC + Pixhawk | Rapid custom control + existing mission stack | Good fit for “keep existing behavior, add advanced companion” | Higher integration test burden than simple setups |
| PX4 + Pixhawk | Architected autonomy + ROS2 co-development | uORB + DDS path is very explicit for external stack developers | More middleware lift for pure mission-only projects |
| PX4 + non-Pixhawk flight board | Specialized hardware goals, vendor-specific IO | Can reduce hardware mismatch for niche sensors | Porting complexity varies heavily |
| Betaflight + non-standard FC | FPV / acro behavior loops | Strong attitude tuning in racing contexts | Not mission-autonomy-first |
| DJI OSDK ecosystem | Closed industrial workflows | Fast path where vendor integration and support dominate | Limited low-level stack openness |
| Other bare-metal stacks | Niche projects | Can be useful for one-off constraints | Maintenance + safety ecosystem often weaker |

---

## 🛰️ Kerbal Space Program status

I did not find an official ArduPilot/PX4 documentation path that presents a direct integration with Kerbal Space Program as a supported default simulator like X-Plane.

If you want the *feeling* of autopilot-like behavior in KSP, those are separate KSP mods:

- `kOS` for scripted vessel control in KSP.
- `AtmosphereAutopilot` for flight-style control assist.

These are game mod ecosystems, not the ArduPilot/PX4 stack integration used in vehicle robotics.

References:
- KSP autopilot mod (`kOS`): `https://github.com/Nivekk/KOS`
- KSP atmospheric autopilot mod: `https://github.com/Boris-Barboris/AtmosphereAutopilot`

---

## 📌 Pros / cons by stack intent

| Intent | ArduPilot + Pixhawk | PX4 + Pixhawk |
|---|---|---|
| “I want to finish a working mission-capable mower fast” | Stronger match: mission-first, broader legacy docs, established ground workflows | Stronger stack work, but more overhead for mission-only entry |
| “I need ROS2-first architecture” | Feasible, documented via MAVLink/ROS bridging | Better native fit due uORB/DDS-first design |
| “I care most about simulation realism before hardware” | Great for X-Plane integration specifically, plus simulator variety | Great for documented multi-vehicle and ROS2-centric sim workflows |
| “I want explicit modular middleware boundaries” | Possible but less obvious from user-level docs | Very explicit in middleware-first docs |
| “I need permissive stack licensing for closed products” | GPL-3.0 constraints | BSD-3 is easier for proprietary downstream use |

---

## 🔗 Related notes (Obsidian links)

- [[ArduPilot]]
- [[PX4]]
- [[Pixhawk]]
- [[MAVLink]]
- [[MAVProxy]]
- [[QGroundControl]]
- [[Mission Planner]]
- [[Rover]]
- [[SITL]]

---

## 📚 Core sources used

- ArduPilot dev/docs home and license: `https://ardupilot.org/dev/`
- ArduPilot dev docs index and mission/SITL pages:
  - `https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html`
  - `https://ardupilot.org/dev/docs/sitl-with-xplane.html`
  - `https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html`
  - `https://ardupilot.org/dev/docs/common-lua-scripts.html`
  - `https://ardupilot.org/dev/docs/common-limited-firmware.html`
- ArduPilot Rover docs:
  - `https://ardupilot.org/rover/index.html`
  - `https://ardupilot.org/rover/docs/common-mission-planning.html`
  - `https://ardupilot.org/rover/docs/common-use-cases-and-applications.html`
  - `https://ardupilot.org/rover/docs/rover-video-demonstrations.html`
- ArduPilot repos:
  - `https://github.com/ArduPilot/ardupilot`
  - `https://github.com/ardupilot/pymavlink`
- PX4 ecosystem docs:
  - `https://docs.px4.io/main/`
  - `https://docs.px4.io/main/en/flight_controller/pixhawk_series`
  - `https://docs.px4.io/main/en/simulation/index`
  - `https://docs.px4.io/main/en/middleware/uorb`
  - `https://docs.px4.io/main/en/ros2/`
  - `https://docs.px4.io/main/en/flight_modes/offboard.html`
  - `https://docs.px4.io/main/en/flying/missions`
- PX4 repos:
  - `https://github.com/PX4/PX4-Autopilot`
  - `https://github.com/px4`
- Ground tools:
  - `https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/firmware.html`
  - `https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/safety_ardupilot.html`
  - `https://ardupilot.org/mavproxy/index.html`
- MAVLink standard:
  - `https://mavlink.io/en/messages/common`

## 🧭 Practical continuation

For most users, picking between ArduPilot and PX4 is mostly a **support matrix problem** rather than a feature race.

Use this checklist before you buy a board or flash a stack.

1. **Need immediate flight-ops support for unusual craft (VTOL, quadplane, fixed-wing, or heavy custom hardware?)**
   - Prefer `[[ArduPilot]]` as default.
   - Confirm the exact flight-controller target is listed in ArduPilot's supported hardware index.
2. **Need strict safety architecture with managed MAVLink 2 and tooling for middleware-first integration**
   - Prefer `[[PX4 Autopilot]]`.
   - Verify vendor support docs for SD card logging, bootloader mode, and fail-safe behavior.
3. **Need mission-logic flexibility and extensive scripting/community support**
   - `[[ArduPilot]]` ecosystem usually wins for legacy mission extensions and scripting breadth.
4. **Need deterministic embedded deployment path and long-term CI-like maintenance**
   - Prefer boards with explicit firmware maintainers on both vendor and ecosystem channels (`[[Holybro]]`, `[[CUAV]]`, `[[CubePilot]]`).
5. **Need strict budget control**
   - Start from a known compatible board matrix, then check local pricing before purchase; some premium boards require extra GPS/compass/telemetry modules.

### Migration and coexistence tips

- If you need both stacks, buy a stack that supports both and run parallel bench tests.
- Keep `[[bootloader]]` and firmware branch versions aligned with board vendors when generating flight logs for support.
- For dual-stack evaluation, keep one baseline config and replicate RC/channel mappings exactly to remove human-induced behavior differences.
- Document which board variant you run (`pro` vs `nano` vs `plus`) because mechanical and IO differences can silently break sensor wiring assumptions.

### What to avoid when comparing boards

- Don’t compare boards only by board-name (`Pixhawk`) because mechanical layout, power stage, and IMU set changes are major.
- Don’t assume a newer label is automatically better for your mission type; many old boards are excellent if they match your payload and EKF profile.
- Don’t ignore vendor firmware cadence; stale releases can mean unresolved safety regressions long after launch.

---

## 🧱 Board-level comparison deep dive (best → worst)

This section is for direct **board-to-board** comparison across Pixhawk-family controllers that commonly appear in ArduPilot/PX4 workflows.

| Rank | Board | Processor (main) | Approx. price (USD) | ArduPilot compatibility | PX4 compatibility | Board strengths | Likely pain points |
|---|---|---|---|---|---|---|---|
| 1 | [[CUAV V5+]] | STM32F765 + STM32F100 co-processor | $289 – $458 | Officially supported in CUAV + ArduPilot flows | Officially listed on CUAV docs as dual-support target in practice | Highest all-around reliability class for mixed mission stacks, redundant power path designs, mature field-proven stack | Highest entry cost among comparable units; check revision compatibility before order |
| 2 | [[Cube Orange+]] | STM32H757 dual-core (M7 + M4) | $215 – $380 | Well-known ArduPilot target with broad community and vendor procedures | Supported via `Cube Orange` / `[[Cube Orange+]]` pathways in vendor + PX4 ecosystem docs (non-canonical Pixhawk header caution) | Very strong ecosystem: redundancy, proven companion integrations, good fail-safe behavior tooling | Board naming/compatibility confusion with non-standard connector/stack variants can lead to flash-step mistakes |
| 3 | [[Holybro Pixhawk 6X]] | STM32H753 | $165 – $250 | Supported as FMUv6X lineage target in ArduPilot documentation and release support matrices | Officially maintained in PX4 standard autopilot family at current branch levels | Best value for full modern feature set; balanced compute and I/O density for most projects | Requires exact hardware revision checks for some sensor and USB/serial combinations |
| 4 | [[Holybro Pixhawk 6C]] / [[Holybro Pixhawk 6C Mini]] | STM32H743 | $130 – $170 | Supported through ArduPilot target matrix and vendor compatibility notes | Supported in PX4 target tables for FMUv6C class | Good middle ground for cost-sensitive builds with modern processing class and enough UART/PWM for most rover/copter plans | Slightly fewer expansion margins than 6X; some builds need power-correct wiring and filtered supply care |
| 5 | [[Holybro Pixhawk 6X Pro]] | STM32H753 | $450 – $650 | Same FMUv6 lineage target family with extended accessory and enclosure value | Supported in PX4 as 6X variant where variant pages list compatible bundles | Useful if you need the Pro variant’s hardware packaging / ecosystem accessories | Price is very high for baseline firmware work; avoid for pure hobby unless budget is fixed |
| 6 | [[Holybro Pixhawk 5X]] | STM32F765 | $105 – $130 | Supported in ArduPilot vendor compatibility guidance | Older but still valid in PX4 reference matrix (`FMUv5`/legacy-class entry) | Solid budget board for legacy-style builds | Older IMU/power architecture relative to v6 boards; fewer headroom features |
| 7 | [[Holybro Pixhawk 4]] | STM32F765 | $95 – $135 | ArduPilot compatibility exists via board support lists and legacy guidance | PX4 lists as historical/deprecated-standard family with reduced long-tail support clarity | Lowest price entry to Pixhawk-style workflows | Aging hardware generation; newer tooling and firmware branches can be noisier |

### What changed by rank (practical guidance)

| Goal | Best board picks (descending) | Why |
|---|---|---|
| Fastest safe deployment with the least support surprises | [[CUAV V5+]] → [[Cube Orange+]] → [[Holybro Pixhawk 6X]] | Larger compatibility envelope, better factory documentation, fewer hidden incompatibilities |
| Best balance of cost and capability | [[Holybro Pixhawk 6X]] → [[Holybro Pixhawk 6C]] / [[Holybro Pixhawk 6C Mini]] → [[Cube Orange+]] | Keeps modern performance while reducing spend |
| Lowest-capex experimental build | [[Holybro Pixhawk 5X]] → [[Holybro Pixhawk 4]] → [[Holybro Pixhawk 6C Mini]] | Cheap entry for controlled test loops and short-range indoor flight |
| Strict PX4 + ROS2 workflows on Pixhawk ecosystem | [[Holybro Pixhawk 6X]] → [[Holybro Pixhawk 6C]] → [[Holybro Pixhawk 5X]] | Better alignment with current PX4 reference hardware for middleware-centric setups |
| Long field life / harsh environment resilience | [[CUAV V5+]] → [[Cube Orange+]] → [[Holybro Pixhawk 6X]] | Stronger redundancy and ecosystem maturity |

### Deployment complexity ranking (best → worst)

| Rank | Board | Flashing complexity* | Typical setup effort | Recommended for |
|---|---|---|---|
| 1 | [[CUAV V5+]] | Medium | Medium | Mixed-use robotics needing stable long runs |
| 2 | [[Cube Orange+]] | Medium-high | Medium-high | Professional/mixed payload projects where redundancy is key |
| 3 | [[Holybro Pixhawk 6X]] | Medium | Medium | Teams shipping both ArduPilot + PX4 prototypes |
| 4 | [[Holybro Pixhawk 6C]] / [[Holybro Pixhawk 6C Mini]] | Low-Medium | Low-Medium | Small/medium copters and fixed-wing with budget constraints |
| 5 | [[Holybro Pixhawk 6X Pro]] | Medium-high | Medium | Hardened enclosure-first builds; high-cost but stable mechanical shell |
| 6 | [[Holybro Pixhawk 5X]] | Low | Medium | Legacy controllers and existing wiring ecosystems |
| 7 | [[Holybro Pixhawk 4]] | Low | Medium-High | Legacy replacement boards where software stack is already tuned |

*“Flashing complexity” = board-specific bootloader behavior, firmware target selection precision, and power-rail behavior across boards.

### Board choice checklist (rank-aware)

- If you can spend for reliability: [[CUAV V5+]] first, then [[Cube Orange+]].
- If you want modern value: [[Holybro Pixhawk 6X]] then [[Holybro Pixhawk 6C]] / [[Holybro Pixhawk 6C Mini]].
- If you need legacy replacements: [[Holybro Pixhawk 5X]], then [[Holybro Pixhawk 4]].
- Treat variant labels (`Pro`, `Mini`, `Plus`) as separate purchase decisions, not naming noise.
- Always pair this table with your exact target branch (ArduPilot release + PX4 branch) and your vendor-supplied firmware packaging page.

Sources for this section:
- ArduPilot hardware/board compatibility and build targets: `https://ardupilot.org/dev/docs/choosing-an-autopilot.html`
- ArduPilot common supported autopilots list: `https://ardupilot.org/plane/docs/common-autopilots.html`
- ArduPilot loading firmware docs: `https://ardupilot.org/planner/docs/common-loading-firmware-onto-pixhawk.html`
- PX4 Pixhawk ecosystem docs (standard family, supported hardware pages): `https://docs.px4.io/main/en/flight_controller/pixhawk_series`, `https://docs.px4.io/v1.14/en/flight_controller/autopilot_pixhawk_standard`
- [[CUAV V5+]] docs/product: `https://doc.cuav.net/controller/v5-autopilot/en/v5%2B.html`, `https://store.cuav.net/shop/v5-controller/`
- Cube Orange setup and compatibility: `https://docs.cubepilot.org/user-guides/autopilot/the-cube/setup/firmware/installing-ardupilot`, `https://docs.px4.io/v1.13/en/flight_controller/cubepilot_cube_orange`
- [[Holybro Pixhawk 6X]] / 6C / 5X / [[Holybro Pixhawk 4]] pages: `https://docs.holybro.com/autopilot/pixhawk-6x/technical-specification`, `https://docs.holybro.com/autopilot/pixhawk-6x/supported-firmware`, `https://docs.holybro.com/autopilot/pixhawk-6x-pro/supported-firmware`, `https://docs.holybro.com/autopilot/pixhawk-6c/supported-firmware`, `https://docs.holybro.com/autopilot/pixhawk-6c/technical-specification`, `https://holybro.com/collections/flight-controllers`

---

## 🧱 Board comparison atlas (best→worst, rank-expanded)

This is an additive atlas for board-level selection. It keeps best→worst order where there is a clear score, while also showing secondary scores for different real-world intents.

### A) Explicit support and compatibility matrix

| Rank | Board | Main processor | ArduPilot target / compatibility anchor | PX4 target / compatibility anchor | Source confidence | Typical flashing path | Most important caveat |
|---|---|---|---|---|---|---|
| 1 | [[CUAV V5+]] | STM32F765 + STM32F100 | ArduPilot + PX4-capable flow in CUAV/CUAV docs | CUAV docs and PX4 ecosystem map to PX4-compatible mode | ✅ high | Mission Planner for ArduPilot, QGroundControl for PX4 (vendor firmware packages where available) | Price and revision choice are the biggest procurement risks |
| 2 | [[Cube Orange+]] | STM32H757 dual-core | CubePilot installation docs explicitly cover ArduPilot (`Install ArduPilot`) | PX4 ecosystem page covers Cube Orange with connector/footprint compatibility notes | ✅ high | Vendor-specific Cube tooling + ArduPilot installer + QGC flows | Connector compatibility and stack-specific cabling can vary by board generation |
| 3 | [[Holybro Pixhawk 6X]] | STM32H753 | Listed in standard PX4-supported autopilot pages | Listed in standard PX4-supported PX4 autopilot pages | ✅ high | ArduPilot firmware installer + QGroundControl paths | Revision-specific compatibility checks (USB, sensor, and base board variant) |
| 4 | [[Holybro Pixhawk 6C]] / [[Holybro Pixhawk 6C Mini]] | STM32H743 | Supported on PX4 target-class pages for FMUv6C family | Supported on PX4 target-class pages for FMUv6C family | ✅ medium-high | Standard FC flashing tools tied to target-specific firmware packages | Some builds can be sensitive to wiring noise and power filtering |
| 5 | [[Holybro Pixhawk 6X Pro]] | STM32H753 | Same FMUv6 lineage target family as 6X | PX4-compatible class reference for 6X-line variants | ✅ medium-high | Same as 6X class, plus bundled-case/variant-specific accessories | Higher BOM cost than 6C/6X for equivalent baseline capabilities |
| 6 | [[Holybro Pixhawk 5X]] | STM32F765 family | ArduPilot legacy references | PX4 FMUv5 lineage notes | ⚠️ medium | Standard flash tooling; often faster due to legacy docs + simpler variants | Lower headroom on newer sensor+IO-heavy feature stacks |
| 7 | [[Holybro Pixhawk 4]] | STM32F765 | Historical legacy references in ArduPilot docs and vendor pages | Historical/discontinued-support FMUv5-line notes | ⚠️ medium-low | Standard flash path, often requires explicit revision verification | Potential support friction on newer tooling and firmware expectations |

### B) Hardware-level comparables (best→worst by practical stack balance)

| Rank | Board | Processor / SoC | Build generation class | Sensor / redundancy profile | I/O breadth | Power design complexity | Why this compares well in practice |
|---|---|---|---|---|---|---|---|
| 1 | [[CUAV V5+]] | STM32F765 + STM32F100 co-processor | Higher-generation FMU/IO family for field robots | Multi-sensor design and strong field orientation | Good UART/CAN/IO balance for mixed vehicles | Redundant power options and mature wiring patterns | Best fallback margin for integration complexity and fault recovery |
| 2 | [[Cube Orange+]] | STM32H757 dual-core (M7+M4) | Modern industrialized Pixhawk lineage build | Triple-IMU style family behavior in many deployments | Strong serial/PWM availability for companion and fail-safe wiring | Non-standardized header lineage in some variants | Mature ecosystem and mission-focused field adoption |
| 3 | [[Holybro Pixhawk 6X]] | STM32H753 | FMUv6X generation | Modern IMU + barometric options | High I/O density for most commercial-class stacks | Moderate complexity, strong standard footprint | Strong mid-to-high feature-to-effort ratio |
| 4 | [[Holybro Pixhawk 6X Pro]] | STM32H753 | 6X generation with Pro packaging variants | Similar IMU class as 6X with package-specific options | Comparable core I/O class to 6X | Added complexity due to accessory-specific choices | Better for ruggedized builds than value builds |
| 5 | [[Holybro Pixhawk 6C]] / [[Holybro Pixhawk 6C Mini]] | STM32H743 | FMUv6C-class compact profile | Compact profile and reduced board margin than 6X family | Good but slightly tighter for large sensor stacks | Fewer margin points in aggressive low-power builds | Excellent cost-performance for constrained payload and size |
| 6 | [[Holybro Pixhawk 5X]] | STM32F765 main | FMUv5 class | Legacy sensor strategy and reduced modern headroom | Adequate but older I/O envelope | Simplified power/IO wiring, fewer modern niceties | Reliable low-cost baseline if your stack is conservative |
| 7 | [[Holybro Pixhawk 4]] | STM32F765 | FMUv5 era legacy | Older redundancy profile than newer boards | Lowest modern I/O density in this shortlist | Lowest wiring overhead but highest mismatch risk | Cheapest entry point with legacy assumptions |

### C) Board mission-fit matrix (0–10, best→worst per use-case)

Legend: `10` strongest, `7–9` strong, `4–6` workable with setup cost, `<4` weak/partial.

| Board | Fixed-wing mission | Multicopter mission | Rover/ground autonomy | ROS2 + offboard workflows | Field durability & safety margin | Rapid prototyping + tuning | Value under budget pressure |
|---|---|---|---|---|---|---|---|
| [[CUAV V5+]] | 9 | 9 | 8 | 8 | 9 | 7 | 5 |
| [[Cube Orange+]] | 9 | 9 | 8 | 8 | 9 | 6 | 6 |
| [[Holybro Pixhawk 6X]] | 8 | 9 | 8 | 8 | 8 | 8 | 8 |
| [[Holybro Pixhawk 6X Pro]] | 8 | 8 | 7 | 8 | 9 | 5 | 4 |
| [[Holybro Pixhawk 6C]] / [[Holybro Pixhawk 6C Mini]] | 7 | 8 | 8 | 7 | 7 | 9 | 8 |
| [[Holybro Pixhawk 5X]] | 6 | 7 | 6 | 6 | 6 | 7 | 9 |
| [[Holybro Pixhawk 4]] | 5 | 6 | 5 | 5 | 5 | 6 | 9 |

### D) Board archetype buckets

| Archetype | Boards (rank order) | Why this archetype exists |
|---|---|---|
| Flagship / mission-critical | [[CUAV V5+]] → [[Cube Orange+]] | Highest practical stability and ecosystem trust for mixed ArduPilot + PX4 workflows |
| Best bang-for-buck | [[Holybro Pixhawk 6X]] → [[Holybro Pixhawk 6C]] / [[Holybro Pixhawk 6C Mini]] → [[Holybro Pixhawk 6X Pro]] | Keep modern compute/features while avoiding the Pro premium when mission needs are moderate |
| Cheapest starter | [[Holybro Pixhawk 5X]] → [[Holybro Pixhawk 6C]] / [[Holybro Pixhawk 6C Mini]] → [[Holybro Pixhawk 4]] | Useful for budget labs, especially when sensor burden is low |
| Most common in field builds | [[Cube Orange+]], [[Holybro Pixhawk 6X]], [[Holybro Pixhawk 6C]] / [[Holybro Pixhawk 6C Mini]] | Broadly documented, commonly discussed in community build guides and vendor communities |

### E) Practical board ranking add-ons (decision-oriented)

For each board type, use these quick confidence checks before you buy:

- Flagship profile ([[CUAV V5+]], [[Cube Orange+]]) focuses on long tuning cycles, multi-mission systems, and harsh fields.
- Practical expectation for flagship boards: longer procurement windows and greater cost sensitivity, especially with shipping and add-ons.
- Value profile ([[Holybro Pixhawk 6X]], [[Holybro Pixhawk 6C]] / [[Holybro Pixhawk 6C Mini]]) focuses on modern stack compatibility with controlled budget and faster build speed.
- Practical expectation for value boards: verify revision-specific firmware notes for USB/serial behavior before purchase.
- Entry profile ([[Holybro Pixhawk 5X]], [[Holybro Pixhawk 4]]) fits constrained workloads with known legacy compatibility.
- Practical expectation for entry boards: include extra validation time for modern companion integrations.

### F) Sources for added matrixes

- ArduPilot hardware selection context: `https://ardupilot.org/dev/docs/choosing-an-autopilot.html`
- ArduPilot common-supported autopilots: `https://ardupilot.org/plane/docs/common-autopilots.html`
- ArduPilot firmware install path: `https://ardupilot.org/planner/docs/common-loading-firmware-onto-pixhawk.html`
- ArduPilot docs index and target concepts: `https://ardupilot.org/dev/docs/` (latest branch context)
- PX4 Pixhawk family pages: `https://docs.px4.io/main/en/flight_controller/pixhawk_series`, `https://docs.px4.io/v1.14/en/flight_controller/autopilot_pixhawk_standard`
- CubePilot install flow: `https://docs.cubepilot.org/user-guides/autopilot/the-cube/setup/firmware/installing-ardupilot`
- Cube Orange in PX4 docs: `https://docs.px4.io/v1.13/en/flight_controller/cubepilot_cube_orange`
- [[CUAV V5+]] compatibility and specs: `https://doc.cuav.net/controller/v5-autopilot/en/v5%2B.html`, `https://store.cuav.net/shop/v5-controller/`
- [[Holybro Pixhawk 6X]]/6C/6X Pro pages: `https://docs.holybro.com/autopilot/pixhawk-6x/technical-specification`, `https://docs.holybro.com/autopilot/pixhawk-6x/supported-firmware`, `https://docs.holybro.com/autopilot/pixhawk-6x-pro/supported-firmware`, `https://docs.holybro.com/autopilot/pixhawk-6c/supported-firmware`, `https://docs.holybro.com/autopilot/pixhawk-6c/technical-specification`, `https://holybro.com/collections/flight-controllers`
- Additional processor-detail references:
  - [[Holybro Pixhawk 6X]] store card: `https://holybro.com/products/pixhawk-6x`
  - [[Holybro Pixhawk 6C]] store card: `https://holybro.com/products/pixhawk-6c`
  - [[Holybro Pixhawk 5X]] store card: `https://holybro.com/products/pixhawk-5x`
  - [[Holybro Pixhawk 4]] store card: `https://holybro.com/products/pixhawk-4`
  - [[Cube Orange+]] processor details in PX4 guide: `https://docs.px4.io/v1.15/en/flight_controller/cubepilot_cube_orangeplus.html`

### G) What to add next (optional, non-breaking)

- Add board entries one-by-one as you request them (`FMUv6 mini`, `Durandal`, `Raspberry Pi HAT-compatible stacks`) with the same 3-table pattern.
- Add a hardware comparables row for each new board.
- Add a support + version row for each new board.
- Add a mission-fit row for each new board.
- Keep the same rank orientation (best on top for each new table).
