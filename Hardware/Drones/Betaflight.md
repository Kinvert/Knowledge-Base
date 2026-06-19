# Betaflight

`Betaflight` is a high-performance open-source firmware for RC flight controllers focused on manual acro/angle flight, low-latency feel, and tuneability.

In this vault it is usually the “fast-flyer / hobby baseline” sibling to `iNav`, `ArduPilot`, and `PX4`.

---

## Core idea

- Real-time flight stabilization and attitude control for racing/freestyle quads, whoops, and acro builds.
- CLI/configurator-driven tuning model (`PID`, rates, filters, receiver settings, OSD hooks).
- Strong emphasis on responsiveness and control feel over onboard mission autonomy.

### Typical role

- Best pick for: manual acro performance, aggressive FPV, small craft loops.
- Less aligned with: complex autonomous mission planning without a companion stack.

---

## Stack comparison position

| Stack | Focus | Onboard mission behavior | Primary ecosystem | Best fit |
|---|---|---|---|---|
| `Betaflight` | ACRO/attitude performance | Minimal (`setpoints/assisted modes` only, not full mission stack) | FPV/builder ecosystems, flight tuners | Acro/freestyle racing, sport FPV |
| `iNav` | Navigation-capable FC | Native nav modes (`RTH`, `NAV POSHOLD`, waypoints) on compact builds | Autonomy-lite hobby + GPS workflows | Fixed-wing / autopilot-lite |
| `ArduPilot` | Full autopilot stack | Strong, broad mission/vehicle support | Large vehicle classes + mature tools | Production mission/autonomy |
| `PX4` | Modular autopilot + ROS2 | Strong, modern autonomy and middleware integration | Offboard, ROS2, MavLink ecosystems | Research + mixed robotics stacks |

---

## Who Betaflight is for

- Build-and-tune mindset where “feels good in air” is the primary success metric.
- Tight motor/gyro loop behavior on small or medium multirotors.
- Fast test cycles where manual control quality is more important than onboard planning.

Not ideal when:
- you need RTH/waypoints/mission scripting as core FC behavior,
- you need deterministic autonomy workflows on the flight controller itself.

---

## Practical setup (high-level)

### Linux

- Install/configure `Betaflight Configurator` (or browser variant if your distro supports it).
- Connect board over USB and identify serial port.
- Flash firmware that matches board target and hardware revision.
- Calibrate:
  - accelerometer
  - radio endpoints
  - gyro/ESC behavior
- Tune:
  - rates / PID
  - filter pipeline
  - feed-forward / dynamic adjustment per frame type

### Windows

- Install `Betaflight Configurator` from project official download path.
- Connect FC, select correct port, flash target image.
- Apply and save PID and rates profiles once stable.

### Linux + Windows mixed workflow

- Use Linux for repeatable config backup/automation.
- Use Windows GUI for fast manual tuning when needed.

---

## Telemetry, logs, and companion integration

- `Betaflight` logs via blackbox where supported.
- Telemetry support exists for common radio OSD/ground ecosystems.
- Companion/automation patterns are possible, but typically done as external layers (`MAVLink`-ish integration is not the default core mode).

---

## Strengths / Weaknesses

### Strengths

- Excellent acro/angle response and low-latency control loop behavior.
- Mature tuning tooling and abundant setup knowledge in FPV community.
- Reliable defaults on many mainstream F4/F7 builds.

### Weaknesses

- Mission/autonomy primitives are narrower than `iNav`/`ArduPilot`.
- Not the first choice for fixed-wing mission-heavy workflows.
- Heavy RL/companion-onboard experiments often need custom bridge architecture.

---

## What Betaflight can do vs what it can’t do

| Capability area | What it can do | Practical confidence |
|---|---|---:|
| Manual stabilization | Very strong in angle rate loops, acro feel, aggressive input response | High |
| PID/filter tuning | Rich PID/filter knobs, fast iteration on rates/feel | High |
| RC link handling | Strong receiver and channel support for mainstream radios | High |
| OSD/telemetry integration | Standard FC telemetry to OSD/radio ecosystems | Medium-High |
| Logging | Blackbox logs for control-quality workflows | High (when enabled) |
| Multirotor workflows | Excellent for quad/micro/mid-size quads and whoops | High |
| Fixed-wing flight support | Supports a limited subset of fixed-wing-specific workflows | Medium |
| Mission logic (RTH/waypoints) | Limited native support in default mission planning sense | Low |
| Full autonomous behavior | Not a full onboard mission/autonomy stack by default | Low |
| Deterministic offboard RL loop | Possible via custom bridge, but not native, high-friction | Low-Medium |
| HIL/SITL parity | Not the best documented official path for strict sim-to-real iteration | Low |
| Sensor-rich autonomy behavior | Can use onboard navigation helpers, but not as deep as `iNav`/`ArduPilot` | Medium-Low |

| Not a fit for | Why it is hard in Betaflight |
|---|---|
| “I need reliable mission planning on FC” | Core focus is manual flight control, not mission graph orchestration |
| “I need broad fixed-wing + rover + marine stack” | Vehicle class coverage is narrower than full autopilot stacks |
| “I need official high-fidelity HIL with scriptable sim loop” | Officially, `ArduPilot`/`PX4` provide deeper standard pipelines |
| “I want zero effort for autonomous policy training” | Policy control usually requires extra middleware/bridge work |

| Good fit for | Why |
|---|---|
| Fast acro tuning | Directly optimized around response/feel control |
| RC and low-latency flight feel validation | Good for control-loop stress tests and handling validation |
| Companion logic at edge | Works if companion owns all high-level mission logic and FC is low-level layer |

### Decision summary for RL and sim-to-real

- Choose `Betaflight` if your RL objective is low-level control policy and latency behavior on acro-like vehicles.
- Choose `iNav` if you want onboard nav primitives (RTH/position hold/waypoint modes) in similar budget hardware.
- Choose `ArduPilot` or `PX4` when mission complexity, `SITL/HITL`, and official toolchain coverage are the priority.

---

## Comparison mini-chart (shortlist)

| Product family | Typical MCU tier | Best use |
|---|---|---|
| `Betaflight` | F4/F7 (entry-mid) | FPV / racing / acro |
| `iNav` | F4/F7/H7 | GPS modes + beginner autonomy |
| `ArduPilot` | F7/H7 | Full autopilot + mission |
| `PX4` | H7 (plus modern companions) | Research + ROS2/advanced autonomy |

---

## Related notes

- [[iNav]]
- [[ArduPilot]]
- [[PX4]]
- [[ArduPilot Boards]]
- [[PX4 Boards]]
- [[Flight Controller]]
- [[Mission Planner]]
- [[QGroundControl]]

## External resources

- https://github.com/betaflight/betaflight
- https://github.com/betaflight/betaflight-configurator
- https://github.com/betaflight/betaflight-configurator/blob/master/README.md
- https://betaflight.com/
- https://github.com/betaflight/betaflight/wiki
- https://github.com/betaflight/betaflight/wiki/Common-Issues
