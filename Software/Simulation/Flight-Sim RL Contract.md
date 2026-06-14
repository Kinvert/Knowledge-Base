---
title: Flight-Sim RL Contract
aliases:
  - Flight-Sim RL Contract
  - Flight Sim RL Schema
  - Simulator Contract for RL
tags:
  - flight-simulation
  - reinforcement-learning
  - pufferlib
  - simulation-contract
  - vectorized-environments
---

# Flight-Sim RL Contract

The highest-risk part of using multiple simulators for RL is not “which physics engine is better,” but contract drift: action semantics, observation layout, step timing, resets, and units vary by project and wrapper.

This note defines a practical contract that lets you swap [[JSBSim]], [[FlightGear]], [[X-Plane]], and legacy RC sims into a single PufferLib-oriented training pipeline.

---

## Why this exists

If your goal is realistic RL on flight physics, you need all of these to be consistent before tuning algorithms:

- fixed action semantics and bounds
- deterministic reset semantics
- fixed observation schema
- common reward decomposition
- stable timing and seed strategy

Without this, a policy that works in one sim wrapper often fails when moved to another due to hidden unit or frame mismatches, not policy quality.

---

## Core Contract (Minimal v0.1)

## 1) Timestep contract

For every simulator adapter:

- `dt` must be fixed in seconds (no adaptive physics step).
- one `step(action)` maps to exactly one integration update (or one fixed update count, documented in metadata).
- physics speed can be wall-clock accelerated (`sim_speed`), but reward accounting uses sim-step count, not FPS.
- `reset(seed)` must fully clear state and set known initial conditions.

Recommended defaults:

| Field | Value | Reason |
|---|---:|---|
| `dt` | `0.01` (100 Hz) | Safe default for fixed-wing/quad learning loops |
| `max_episode_steps` | `1000` | Gives 10 s episode length to start |
| `fixed_seed` | `uint32` | Deterministic repeatability |

---

## 2) Action contract

Use one normalized action vector internally; each simulator maps channels through an adapter.

### Required channels (fixed-wing baseline)

| Canonical index | Name | Range | Interpretation |
|---|---|---:|---|
| 0 | `elevator` | `[-1, 1]` | Normalize pitch input |
| 1 | `aileron` | `[-1, 1]` | Roll command |
| 2 | `rudder` | `[-1, 1]` | Yaw/heading command |
| 3 | `throttle` | `[0, 1]` | Fuel/collective energy |

### Optional channels

| Canonical index | Name | Typical use |
|---|---|---|
| 4 | `flaps` | Takeoff/landing/drag shaping |
| 5 | `gear` | Binary-turned-continuous (0/1) |
| 6 | `airbrake` | Drag/brake control |
| 7 | `collective` | Helicopter collective (fixed-wing ignored) |
| 8 | `swash` | Swashplate/3-axis RC heli channel abstraction |

### Mapping rule

Each adapter MUST include a machine-readable mapping table:

- canonical index -> sim-specific control name
- scale/offset transform
- saturation behavior
- failure behavior when channel unsupported (hold/scale-to-zero/no-op)

This avoids silent action drift across sims.

---

## 3) Observation contract

Observations should be numeric, fixed-length, contiguous, `float32`, and ordered.

### Core observation set (recommended minimum)

| Index range | Concept | Units |
|---|---|---|
| 0:2 | Position `(x, y, z)` in world/local frame | meters |
| 3:5 | Euler or equivalent angles `(roll,pitch,yaw)` | radians |
| 6:8 | Angular rate `(p,q,r)` | rad/s |
| 9:11 | Linear velocity `(u,v,w)` body/world | m/s |
| 12 | Altitude | meters |
| 13 | Airspeed | m/s |
| 14 | Angle of attack | rad |
| 15 | Sideslip angle | rad |
| 16 | Vertical speed | m/s |
| 17:20 | Control effectors (normalized) | unitless |
| 21 | Gear/gear state | binary/normalized |
| 22 | Reward-shaping auxiliaries (energy/trim error etc.) | derived |

Add extra slots behind this core set for simulator-specific telemetry if needed.

### Frame and unit rules

Contract rules:

- Position/velocity defaults to NED unless simulator cannot expose it; document alternative and convert in adapter.
- Angles are radians in adapter-exposed state, even if UI displays degrees.
- No mixed speed units in the core vector (avoid knots inside primary channels).

---

## 4) Reward contract

Reward is a scalar by default for Puffer compatibility, with optional decomposition.

Recommended policy:

- `reward = Σ reward_components`
- also emit `info["reward_breakdown"]` with component names and values.

At minimum include:

| Component | Goal |
|---|---|
| `r_progress` | Keep forward motion toward target profile |
| `r_attitude` | Penalize high rates/extreme AoA / overload |
| `r_altitude` | Keep altitude envelope |
| `r_energy` | Discourage excessive throttle use |
| `r_event` | +1/+5 on success, large negative on crash |

Keeping reward components in info prevents “reward hack” collapse where only total reward is tuned.

---

## 5) Episode termination contract

Use the split terminal flags:

- `terminated` for physical/goal terminal (crash/goal reached/stall hard-fail)
- `truncated` for time-limit or environment boundary
- `info["reason"]` contains explicit machine-readable reason

This matches modern RL APIs and makes rollout accounting clear across simulators.

---

## 6) Adapter shape requirements for [[PufferLib]]

PufferLib prefers dense contiguous memory for speed (shared state buffer + contiguous arrays). The contract should therefore expose:

- `obs`: `float32[num_envs, obs_dim]`
- `actions`: `float32[num_envs, act_dim]`
- `reward`: `float32[num_envs]`
- `terminated`: `uint8[num_envs]`
- `truncated`: `uint8[num_envs]`
- `info`: fixed-size diagnostics per step or per-episode

Avoid per-step dynamic allocations and nested Python objects inside the hot loop.

For C99 environments, this aligns with the native vectorized binding model in PufferLib’s docs (shared state and vectorized stepping).[^puffer-vectorized]

---

## 7) Determinism requirements

Every adapter should pass these checks:

- state equality for same `(seed, episode_idx, step_idx, action)` tuple
- deterministic reset even after nonzero crash states
- bounded reset latency distribution
- no hidden random sources outside seeded PRNGs
- no asynchronous socket backlog affecting order of updates

When not possible (e.g., GUI-bound or networked sims), label contract support as `"non_deterministic_bridge": true`.

---

## 8) Existing simulator support matrix (current knowledge state)

| Simulator | Action read | Obs read | Reset API | Determinism posture | Contract match quality |
|---|---|---|---|---|---|
| [[JSBSim]] | Yes via C++ property/command flow | Yes via scripted outputs (`FGOutput`) | Yes via init/script flow | High (headless batch) | High |
| [[FlightGear]] | Yes via native/generic controls | Yes via property tree / protocol streams | Partial (launcher/state scripts) | Medium (external protocol + renderer coupling) | Medium |
| [[X-Plane]] | Yes via XPLM dataref APIs and commands | Yes via `DataAccess` | Yes via flight-loop + plane init APIs | Medium (callback timing, plugin/thread limits) | Medium-High |
| [[Microsoft Flight Simulator]] | Yes via SimConnect client API (`Open`, `Request`, `Transmit`, data/events) | Yes via SimConnect data requests and events | Partial/experimental. Out-of-process clients are feasible; no documented step/reset RL contract | Medium-High (API exists; threading/design constraints) | Medium |
| [[ClearView RC Flight Simulator]] | No public programmatic control API found | No public state API found | Unclear/no documented API | Low (GUI-bound assumptions) | Low |
| [[CRRCSim]] | CLI/input and source-level integration possible | Source-level state possible only via code coupling | Source-level init exists; no standardized env API | Medium-Low (legacy) | Medium-Low |
| [[Absolute RC]] | Browser/gamepad control only via UI flows | No published RL state API | No public reset/state API | Low | Low |
| [[Aerofly RC10]] | Desktop controls via local UI (joystick/gamepad/input) only | No public step/telemetry API | No scripted reset/state API found | Medium-Low | Low |
| [[neXt RC]] | No public API found in product docs | No public state API found | No documented scripted reset | Low | Low |

The table is not a pass/fail verdict; it is a wrapper-cost indicator.

---

## 9) Suggested adapter templates by simulator family

### A) Native API adapter (best for PufferLib throughput)

- Example candidates: [[JSBSim]] C++ path, [[X-Plane]] plugin path.
- Minimal process count, shared memory preferred.
- Keep wrappers thin: action mapping + property readout + reset orchestration.

### B) IPC adapter (protocol bridge)

- Example candidates: [[FlightGear]] external protocol, X-Plane web bridge.
- Works with separate process boundaries.
- Requires strict sequence IDs/timestamp checks and drift handling.

### C) Black-box/Viz-only adapter

- For simulators with no exposed state/action APIs (legacy GUI workflows).
- Requires CV reward and vision/state reconstruction.
- Lowest priority for realistic flight-physics RL unless used only for evaluation.

---

## 10) Minimal spec artifact (must-have artifact)

Create one YAML/JSON manifest per environment version:

- `sim_id`
- `sim_build`
- `contract_version`
- `dt`
- `act_map`
- `obs_map`
- `unit_map`
- `seed_policy`
- `termination_rules`

This manifest becomes the truth source for every benchmark, replay, and sim-to-sim transfer check.

---

## 11) Next-step workflow

1. Freeze this contract in v0.1.
2. Implement adapters for [[JSBSim]] and one legacy/visual target.
3. Log action/obs equivalence tests per control event set.
4. Run throughput baseline on the same contract before adding fidelity changes.
5. Only then add additional simulators.

---

## See also

- [[PufferLib Flight Sim Integration Gaps]]
- [[PufferLib Flight Throughput Benchmark]]
- [[PufferLib]]
- [[PufferLib C99 Environment Authoring]]
- [[Vectorized Environments]]
- [[JSBSim]]
- [[FlightGear]]
- [[X-Plane]]
- [[Microsoft Flight Simulator]]
- [[ClearView RC Flight Simulator]]
- [[CRRCSim]]

---

## Source Notes

[^puffer-vectorized]: PufferLib documentation emphasizes contiguous memory and high-throughput vectorized rollouts via environment bindings and shared state design. https://puffer.ai/docs.html
[^jsbsim-io]: JSBSim documentation and API pages show standalone/batch operation and property-oriented execution (`FGFDMExec::Run`, scripts, output/socket channels). https://jsbsim-team.github.io/jsbsim/ and https://github.com/JSBSim-Team/jsbsim
[^fg-protocol]: FlightGear command-line/protocol pages expose native/generic protocol options and property-tree access paths; these are typically external state/control bridges rather than native RL step APIs. https://wiki.flightgear.org/Command_line_options and https://wiki.flightgear.org/Generic_Protocol
[^xplane-sdk]: X-Plane plugin SDK documents callback, dataref, and processing APIs used for external control/state loops. https://developer.x-plane.com/sdk/
[^xplane-web]: X-Plane Web API docs expose local endpoint-based dataref/command interactions for external clients. https://developer.x-plane.com/article/x-plane-web-api/
[^clearview-source]: ClearView documentation surfaces model/control workflows but does not document a public RL-style step API (public source review context in this vault). https://www.rcflightsim.com/
[^msfs-simconnect]: Microsoft Flight Simulator SimConnect SDK docs describe client add-on model, API calls (Open/Request/Transmit), and async communication model; no direct RL loop API is documented. https://docs.flightsimulator.com/html/Programming_Tools/SimConnect/SimConnect_SDK.htm and https://docs.flightsimulator.com/html/Programming_Tools/SimConnect/SimConnect_API_Reference.htm
