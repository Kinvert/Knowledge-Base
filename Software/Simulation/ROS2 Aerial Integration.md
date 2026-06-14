---
title: ROS2 Aerial Integration
aliases:
  - ROS2 Aerial Integration
  - ROS2 for Flight RL
tags:
  - simulation
  - reinforcement-learning
  - pufferlib
  - ros2
  - px4
  - ardupilot
  - flight-control
---

# ROS2 Aerial Integration

`ROS2 Aerial Integration` is the missing link between *flight-control stacks* and your simulator loop when you want RL to operate on realistic autopilot-era interfaces rather than raw aerodynamics only.

For realistic RL, this matters because the control surface in many real systems is now ROS2 messages, not simulator keypresses.

---

## Why this matters now

Current work is already strong on pure-flight and visual sim simulators, but not all of those map cleanly to real-aircraft integration paths.

The gap is:
- deterministic environment reset through autopilot state,
- robust action→setpoint translation,
- telemetry stream normalization,
- and timing guarantees in offboard control channels.

This note closes that gap.

---

## Research status

| Area | Confirmed status | Confidence |
|---|---|---:|
| PX4 ROS2 integration | PX4 documents deep ROS2 integration (uORB access via DDS bridges) and a dedicated Offboard path with explicit keep-alive semantics. | High |
| ArduPilot ROS2 support | ArduPilot has a ROS2 section with documented interfaces and topic/service mappings. | Medium-High |
| Hard realtime claims | None of these are “native RL step APIs”; both are message-oriented control loops requiring contract adaptation. | High |
| Reset semantics | Reset often comes from simulator reset/SITL world reset + re-arming and mode transitions, not from a dedicated RL reset primitive. | Medium |

---

## PX4 + ROS2 path (high-priority candidate)

PX4’s ROS2 docs explicitly frame ROS2 as a deep integration path where high-rate data paths are possible, including direct read/write of internal `uORB` topics through `PX4-ROS 2 middleware`.[^px4-ros2]

Important mechanics from the docs:

- Two middleware options are called out: **XRCE-DDS** and **Zenoh**.[^px4-ros2]
- Offboard mode requires continuous proof-of-life streaming (`OffboardControlMode`) or MAVLink setpoint messages; otherwise PX4 exits offboard and triggers a fail-safe parameterized action.[^px4-offboard]
- In ROS2 Offboard Mode, messages from internal/external controllers are not distinguishable inside PX4, so your bridge must strictly guard mode and topic ownership.[^px4-offboard]

### What to use for RL

You generally use either:

- **microRTPS/bridge + `px4_msgs` direct topics** (`TrajectorySetpoint`, `VehicleOdometry`, etc.), or
- a higher abstraction layer if you want frame/mission safety guardrails.

For RL, option A is closest to deterministic control if your stack can tolerate the complexity.

### PufferLib adapter shape

1. Keep PX4 in SITL or hardware-in-loop mode with fixed launch seed.
2. Create one ROS2 node per PufferLib worker to avoid shared-state races.
3. At each `step(action)`:
   - publish `OffboardControlMode`/setpoint according to selected control mode,
   - read back pose/velocity/state topics once per sim tick,
   - compute reward and done in host process.
4. Reset flow:
   - disarm / switch to mode,
   - reset world state via simulator API,
   - re-arm and re-enter Offboard with timeout-aware sequencing.

### Where determinism is fragile

- transport jitter (DDS timing),
- coordinate-frame mismatch (PX4 internal NED / ROS2 conventions),
- offboard stream loss timing (`COM_OF_LOSS_T`),
- accidental setpoint overwrite outside Offboard mode.

These are deterministic bugs if instrumented, not silent ones.

---

## ArduPilot + ROS2 path (secondary candidate)

ArduPilot documents ROS2 integration and, importantly, lists concrete ROS2 sensor/control topics and services (for example telemetry, pose/velocity, joystick/`cmd_vel`, arm/mode/setpoint interfaces).[^ap-ros2][^ap-interfaces]

This is valuable if your baseline is ArduPilot fleets or you already have an ArduPilot SITL stack.

### PufferLib adapter shape

1. `bridge_reset`: launch ArduPilot SITL with deterministic start scripts and time-seeded world.
2. Step loop:
   - map normalized action vector to `cmd_vel`, `GlobalPosition`, or equivalent high-level setpoint topic,
   - optionally inject direct control messages where supported,
   - sample telemetry topics for obs.
3. Reset via startup script + controlled mode transitions + vehicle re-arm.

### Known caveat

The API breadth and maturity are more patchwork than PX4; availability is vehicle-mode dependent and integration details can shift by release.[^ap-interfaces]

---

## Comparison (5+)

| Candidate | API type | Why it helps RL | Determinism risk | Real-time throughput profile |
|---|---|---|---|---|
| [[Microsoft Flight Simulator]] | External simulator API only | High-fidelity environment target | Medium | API bridge bound |
| [[X-Plane]] | Plugin/web API | Mature dataref+callback model | Medium | Medium |
| [[FlightGear]] | Protocol/property bridge | Good for legacy stack composition | Medium | Medium |
| [[JSBSim]] | Native simulation core | Highest controllability + fastest iteration | Low | High (headless vectorizable) |
| [[ROS2 Aerial Integration]] | Message-oriented offboard bridge (PX4/ArduPilot) | Direct path to real vehicle control architecture | Medium-High | Medium (DDS/message overhead + mode/state dependencies) |
| [[CRRCSim]] | Source-level model reuse path | Useful for model corpus/airframe logic reuse | Medium-Low | Variable |

---

## How this fits the current RL plan

Recommended order for realism-first stack:

1. `[[JSBSim]]` training core for high-throughput policy learning.
2. Transfer to visual simulators (`[[FlightGear]]`, `[[X-Plane]]`) for policy robustness checks.
3. Add `ROS2 Aerial Integration` as the hardware-faithful transfer stage before real-aircraft trials.

This preserves sample efficiency while still validating controls in the same abstraction layer used by many actual stacks.

---

## Placeholder benchmark row for this stack

| Sim stack | Adapter | Scale N | SPS mean | p99 step ms | Reset p99 ms | Determinism level | Notes |
|---|---|---:|---:|---:|---:|---|
| ROS2 Aerial Integration (PX4) | DDS bridge | 1 | TBD | TBD | TBD | Medium | Track DDS backlog, proof-of-life misses, and frame conversion cost |
| ROS2 Aerial Integration (ArduPilot) | DDS bridge | 1 | TBD | TBD | TBD | Medium | Track topic availability / mode dependency by vehicle type |
| ROS2 Aerial Integration (PX4) | DDS bridge | 64 | TBD | TBD | TBD | Medium | Requires per-env agents; single-flight-mode contention risk |

---

## Related notes

- [[Flight-Sim RL Contract]]
- [[PufferLib Flight Sim Integration Gaps]]
- [[PufferLib Flight Throughput Benchmark]]
- [[PufferLib]]
- [[PufferLib C99 Environment Authoring]]
- [[JSBSim]]
- [[FlightGear]]
- [[X-Plane]]
- [[Microsoft Flight Simulator]]

---

## Source notes

[^px4-ros2]: PX4 ROS 2 overview and ROS2 user guide emphasize deep PX4 integration, uORB access via ROS2 message translation, and two middlewares (XRCE-DDS and Zenoh). https://docs.px4.io/main/en/ros2/
[^px4-offboard]: PX4 offboard mode documentation requires continuous proof-of-life streams (`OffboardControlMode` or setpoints) and warns about controller collision/race conditions in Offboard mode. https://docs.px4.io/main/en/flight_modes/offboard
[^px4-ros2-offboard-example]: PX4 ROS2 offboard control example includes required ROS2 microRTPS setup and per-tick setpoint publishing behavior. https://docs.px4.io/v1.12/en/ros/ros2_offboard_control
[^ap-ros2]: ArduPilot ROS2 entry documents ROS2 extension points and integration expectations. https://ardupilot.org/dev/docs/ros2.html
[^ap-interfaces]: ArduPilot ROS2 interfaces page lists concrete sensor/control topics and services (including joystick, `cmd_vel`, `GlobalPosition`, arm/mode APIs). https://ardupilot.org/dev/docs/ros2-interfaces.html
