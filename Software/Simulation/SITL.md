# SITL (Software-In-The-Loop)

`SITL` is a simulation workflow where the autopilot flight stack runs as software on a computer, while simulated sensors/physics provide environment feedback. It is the standard first step before hardware testing for `ArduPilot`/`PX4` stack changes.

---

## What SITL is and is not

- **SITL**: flight stack code runs on host software, usually the same machine or LAN, and communicates with simulator worlds over MAVLink/adapter interfaces.
- **HITL** / **HIL**: normal autopilot firmware executes on real hardware, with the simulator providing sensor/state feedback and timing context.
- **Replay/recorded playback**: logs drive replays without running full dynamics in real time.

PX4 describes SITL as running the flight stack on computer and HITL as simulation firmware on real flight-controller hardware; both are part of the same safety-first simulation ladder.[^px4-sim]

---

## Where SITL sits in your loop

### Core loop

- Start simulator process (`simulator + vehicle model`).
- Run autopilot in SITL process (e.g. `sim_vehicle.py` for `ArduPilot`).
- Use MAVProxy/QGC/QGroundControl/Mission Planner to send controls and observe state.
- Log and iterate quickly until behavior is stable.

This is the route when you want to change stack code, tune params, or test mission scripts without risking hardware damage.

### Typical stack entry points

- `ArduPilot`: `sim_vehicle.py` plus simulator backends (`XPlane`, `Webots`, etc.), with GCS bridging via TCP/UDP and optional multiple ground stations.[^ardupilot-sitl]
- `PX4`: unified SITL launcher paths (`make px4_sitl ...` in older workflows, plus container/prebuilt flows), with Gazebo/jMAVSim style simulators depending on the target.[^px4-sitl-index][^px4-prebuilt]

---

## Why SITL matters for UGV

- You can validate rover differential/ackerman behavior before wiring real actuators.
- PX4’s simulator matrix includes rover profiles directly in its SITL command catalog, and `ArduPilot` exposes Rover frame models in SITL vehicle selection.
- Parameter sweeps (wind, delay, fault injection, sensor bias) are usually much faster in SITL than hardware.

For UGV policy training, SITL is useful for deterministic loop reset design, but visual-heavy end-to-end RL should still be validated in heavier environment stacks later.

---

## Comparison chart (UGV workflow)

| Approach | Execution target | Time-to-first-flight | Hardware risk | Realism / sensor chain | Best use |
|---|---|---|---|---|---|
| SITL | Host CPU | Fastest | Lowest | Medium (depends on chosen physics backend) | Rapid control/estimator iteration |
| Hardware-on-Hardware (SoH / Simulation on Hardware) | Host + real FC running extra sim build | Moderate | Low | Higher than plain SITL on I/O timing | Edge-of-flight behavior, failsafe validation |
| HITL/HIL | Real FC + simulator bridge | Slower | Medium | Very high for control firmware behavior | Final pre-flight validation |
| Replay-only testing | Pre-recorded logs | Fast | Lowest | Limited | Regression checks on state transitions |
| Full-world sim (CARLA/Gazebo/AirSim/etc.) + SITL bridge | Host + complex world | Slowest | Low | High scenario realism | Policy robustness + perception stack stress |

---

## Setup and portability notes

### `ArduPilot`-centric

- Run SITL with `sim_vehicle.py --help` for argument surface and build current branch as needed.[^ardupilot-sitl]
- Select exact frame and vehicle (`-v`, `-f`) before test campaigns; frame choice changes parameter set and physics model assumptions.
- You can connect multiple GCSs and route UDP/TCP outputs to ground stations during one SITL run.
- You can bind real serial devices for focused subsystem testing (e.g., GPS/OSD path verification).[^ardupilot-sitl]

### `PX4`-centric

- `PX4` docs describe SITL/HITL as complementary steps in a safe change-validation workflow.
- Prebuilt package/containers allow quick bring-up without a full local build (for quick matrix tests), while direct source builds remain useful for custom vehicle/board work.[^px4-prebuilt]
- Multi-instance modes are available; each instance requires unique port mapping and offsets in typical launch flows.[^px4-prebuilt]

---

## Practical pros and cons

### Good for

- Iterating on controller logic and mission scripts fast.
- CI-level regression for stack changes.
- Safe fault/recovery experiments.
- Parameter-scan sweeps before hardware integration.

### Weak spots

- Not a full replacement for real power electronics, EMI, or transport/bus behavior.
- Scenario realism is bounded by selected backend and model quality.
- Timing and numerics differ from target hardware if real serial timing, IO jitter, and CPU load differ significantly.

---

## Related notes

- [[ArduPilot]]
- [[PX4]]
- [[Gazebo Garden]]
- [[AirSim]]
- [[LGSVL Simulator]]
- [[PufferLib]]
- [[Brain-in-the-Loop Flight Sim]]

---

## Sources

- https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html
- https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html
- https://ardupilot.org/dev/docs/sim-on-hardware.html
- https://docs.px4.io/main/en/simulation/index
- https://docs.px4.io/main/en/simulation/px4_sitl_prebuilt_packages

[^ardupilot-sitl]: ArduPilot SITL docs define startup flow, frame selection, GCS connection options, and `sim_vehicle.py` usage.
[^px4-sim]: PX4 simulation index defines SITL/HITL split and intended safe development use.
[^px4-sitl-index]: PX4 simulation index pages link to backend-specific guides and launch options.
[^px4-prebuilt]: PX4 prebuilt package page provides container/debian flows and model/env arguments.

