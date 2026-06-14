---
title: PufferLib Flight Throughput Benchmark
aliases:
  - Flight Sim Throughput Benchmark
  - PufferLib Flight Throughput Harness
  - Flight Sim SPS Benchmark
tags:
  - flight-simulation
  - reinforcement-learning
  - pufferlib
  - benchmarking
  - pufferlib-throughput
---

# PufferLib Flight Throughput Benchmark

This note defines a practical, reproducible harness for comparing simulator candidates for RL training throughput.

Its goal is to remove speculation and replace it with comparable measured values for:

- steps/s (SPS)
- reset throughput
- step latency distribution
- memory/cost overhead from interface layers

This is specifically tuned for flight RL workflows where sim fidelity is important and throughput can become the training bottleneck.

---

## Why this matters for flight RL

In Puffer workflows, the highest-value simulators are not just those with realistic physics, but those you can iterate on at scale.

A realistic engine with unstable batching or high-step variance can still be less useful than a deterministic, moderate-fidelity core during policy search.

---

## Benchmark assumptions

Do not compare simulators with different assumptions.

Use a locked contract:

- Fixed scenario set (start states, episode goals)
- Fixed `dt` (initially `0.01s`)
- Fixed action bounds and reward structure
- Fixed observation schema (from [[Flight-Sim RL Contract]])
- Fixed hardware (same host, same core pinning policy)
- Fixed policy model/inference cost for all runs (or zero policy by random policy)

---

## Harness architecture

### Control plane

- Launcher
  - creates `N` environments
  - verifies adapter readiness and schema compatibility
- Steppers
  - applies action batches
  - enforces fixed-step loop
- Logger
  - collects timing + throughput counters

### Data plane

Three paths to test:

1. **C++/native in-process** (JSBSim path)
2. **External protocol bridge** (FlightGear/X-Plane IPC path)
3. **GUI black-box** (baseline for simulators with no public API)

### Measurement events

- `step_start`
- `step_end`
- `obs_publish`
- `reward_compute`
- `policy_infer`
- `reset_start`
- `reset_end`
- `episode_done`

---

## Core metrics

Let `T_total` be total wall time for `S_total` environment steps:

- `SPS_overall = S_total / T_total`
- `SPS_per_env = mean(steps_per_second_by_env)`
- `p50_step_ms`, `p90_step_ms`, `p99_step_ms` from per-step timing
- `reset_ms_p50`, `reset_ms_p99`
- `step_coverage = env_steps / (policy_steps + env_steps + infra_overhead)` (debug-only)
- `episode_len_mean`, `episode_len_std`

Include a reproducibility artifact:
- simulator version/commit
- config hash
- benchmark manifest (date, commit, hardware, compiler flags, thread config)

---

## Recommended scale plan

Run three scaling steps per adapter:

1. `N = 1` (single-env signal)
2. `N = 64` (small vectorization stress)
3. `N = 4096` (or nearest feasible on that sim architecture)

Not every candidate can reach all scales. Record maximum feasible scale and why.

---

## Per-simulator harness mode and expected bottlenecks

### [[JSBSim]] path

#### Mode
- Use direct C++ or tightly bound C wrapper into PufferLib C99-style stepping.
- Fixed script and fixed aircraft model per benchmark.

#### Typical bottlenecks
- property read/write bandwidth
- aircraft model complexity
- cache misses in multi-instance vector loops

#### Expected profile
- highest SPS potential among current candidate set
- best scalability if native C path is used

---

### [[FlightGear]] path

#### Mode
- Either external rendering/IPC separation or external FDM driver path.
- Keep one FG process per logical env worker when rendering is disabled/limited.

#### Typical bottlenecks
- protocol marshalling
- scenery/time sync loops
- renderer coupling even in headless-like modes

#### Expected profile
- likely lower than JSBSim C path
- scale behavior depends heavily on protocol and frame limits

---

### [[X-Plane]] path

#### Mode
- Path A: in-process plugin (preferred, highest fidelity among X-Plane options)
- Path B: web API bridge (prototyping baseline)

#### Typical bottlenecks
- callback timing limits and single-instance physics/render coupling
- thread-safety and queue depth

#### Expected profile
- lower than C++-native simulators
- plugin path should beat web path materially

### [[Microsoft Flight Simulator]] path

#### Mode
- Path A: out-of-process SimConnect bridge process (recommended start point)
- Path B: in-process WASM modules for narrow callback tasks (advanced, riskier)

#### Typical bottlenecks
- asynchronous SimConnect callback/message dispatch and thread-safety constraints
- out-of-process IPC overhead vs callback cadence
- version/API compatibility and DevMode/package lifecycle during reset

#### Expected profile
- likely moderate wall-clock-limited throughput (potentially lower than JSBSim-native at scale)
- deterministic behavior possible at adapter level, but not a native batched-step engine

---

### [[ClearView RC Flight Simulator]] path

#### Mode
- no confirmed public step/reset API; any RL harness will be GUI/instrumentation-heavy.

#### Typical bottlenecks
- input synthesis latency
- no known vectorized stepping

#### Expected profile
- low SPS unless you prove a deterministic hidden API

---

### [[CRRCSim]] path

#### Mode
- direct source-level extraction/wrapper preferred over GUI automation.
- if using source-level coupling, benchmark wrapper overhead separately.

#### Typical bottlenecks
- old architecture assumptions
- legacy graphics/input stack coupling

#### Expected profile
- potentially medium-low with wrapper path; low with black-box mode

### [[Absolute RC]] path

#### Mode
- browser-based launch with input automation or screen-state extraction.
- no documented reset/action API, so harness likely includes process-level controls.

#### Typical bottlenecks
- browser event loop scheduling
- local input channel jitter
- screenshot capture/vision reconstruction cost

#### Expected profile
- likely low SPS in public form
- deterministic multi-env batching requires additional undocumented bridging work

### [[Aerofly RC10]] path

#### Mode
- desktop simulation process with input automation and optional replay/file extraction first.

#### Typical bottlenecks
- render/input loop coupling
- process launch/tear-down overhead
- possible lack of direct telemetry channels

#### Expected profile
- low-to-medium practical if replay parsing exists
- vectorized batching unlikely without private integrations

### [[neXt RC]] path

#### Mode
- UI/input automation path only unless developer APIs are obtained.

#### Typical bottlenecks
- multiplayer session timing
- GUI/input dependency
- undocumented state serialization

#### Expected profile
- likely low SPS until explicit headless/state API is found

---

## Procedure (copyable skeleton)

### 1) Build run manifest

```yaml
benchmark:
  name: flight_rl_sps_v1
  sim: jsbsim
  dt: 0.01
  max_episode_steps: 1000
  action_dim: 4
  obs_dim: 23
  seeds: [1234, 1235, 1236, 1237]
  steps_per_seed: 20000
  scales: [1, 64, 4096]
  adapter: native_cpp
  policy: random  # or fixed checkpoint
  pin_cpu: true
  threads: 10
```

### 2) Run repeated deterministic trials

For each seed and each scale:

- launch adapter
- run fixed number of total steps
- gather event logs
- export CSV/JSON summary
- check state-reset determinism against first run

### 3) Compare against baseline

Use one environment as baseline:
- `baseline_env = jsbsim-native-cpp`
- report `speedup = sps_candidate / sps_baseline`
- report variance and p99 regression

---

## Example result table (fill with actual runs)

| Simulator | Adapter | Scale N | SPS mean | p99 step ms | Reset p99 ms | Determinism level | Notes |
|---|---|---:|---:|---:|---:|---|
| JSBSim | C++ native | 1 | TBD | TBD | TBD | High | Baseline candidate |
| JSBSim | C++ native | 64 | TBD | TBD | TBD | High | Baseline scaling point |
| JSBSim | C++ native | 4096 | TBD | TBD | TBD | High | Target scaling target |
| JSBSim | Python wrapper | 64 | TBD | TBD | TBD | Medium | Integration sanity baseline |
| FlightGear | protocol bridge | 1 | TBD | TBD | TBD | Medium | Headless mode caveat |
| FlightGear | protocol bridge | 64 | TBD | TBD | TBD | Medium | Measure IPC backlog |
| X-Plane | plugin | 1 | TBD | TBD | TBD | Medium | Callback-based |
| X-Plane | plugin | 64 | TBD | TBD | TBD | Medium | Watch callback contention |
| X-Plane | web API | 64 | TBD | TBD | TBD | Medium | IPC serialization cost |
| Microsoft Flight Simulator | SimConnect bridge | 1 | TBD | TBD | TBD | Medium | API-based bridge; needs callback pacing |
| Microsoft Flight Simulator | SimConnect bridge | 64 | TBD | TBD | TBD | Medium | Out-of-process + async dispatch overhead |
| ClearView | GUI/automation | 1 | TBD | TBD | N/A | Low | Should be interpreted as worst-case |
| CRRCSim | source/legacy wrapper | 1 | TBD | TBD | TBD | Medium-Low | Source-level coupling required |
| Absolute RC | browser automation | 1 | TBD | TBD | N/A | Low | Browser input and policy capture loop |
| Aerofly RC10 | desktop automation | 1 | TBD | TBD | N/A | Low | Render-loop-bound desktop path |
| neXt RC | desktop automation | 1 | TBD | TBD | N/A | Low | UI/input-coupled legacy path |

---

## Failure modes to watch

- policy inference hidden in timing profile
- dropped messages / stale IPC frames
- nondeterministic seeds after reset
- renderer fallback changing step cadence
- callback queue buildup
- uncontrolled frame caps or VSync coupling

---

## Reporting format

Store each run as:

- `raw_timing_*.parquet` (or CSV)
- `run_manifest.json`
- `env_manifest.json`
- `git_diff.txt` of env code used
- `hardware.txt` (CPU model, driver versions, load averages)

Use this format so a future run can reproduce exact speed comparisons across months.

---

## Interpretation rules

When reporting a simulator candidate:

- separate **core physics loop SPS** from **full-visual rollout SPS**
- never claim wall-clock FPS as RL SPS unless policy is in lockstep with `step()`
- prioritize `p99_step_ms` and reset reliability over raw mean SPS
- if throughput differs by <20% but reset/termination reliability differs by >10x, treat reliability as winner

---

## Source Notes

[^puffer-high-throughput]: PufferLib documentation and blog emphasize vectorized, contiguous-buffer, high-throughput environments (millions of steps/sec for suitable native environments). https://puffer.ai/docs.html and https://puffer.ai/blog.html
[^jsbsim-output]: JSBSim output/input and batch/standalone capabilities are documented via CLI and API pages. https://jsbsim-team.github.io/jsbsim/
[^fg-protocol]: FlightGear exposes protocol options and property-tree I/O interfaces suitable for external wrappers. https://wiki.flightgear.org/Command_line_options and https://wiki.flightgear.org/Property_Tree and https://wiki.flightgear.org/Generic_Protocol
[^xplane-plugin]: X-Plane plugin guidance documents callback and DataRef APIs used in external control loops. https://developer.x-plane.com/sdk/
[^xplane-webapi]: X-Plane Web API docs expose localhost HTTP/WebSocket state and command operations (good for prototyping, serialization overhead expected). https://developer.x-plane.com/article/x-plane-web-api/
[^msfs-simconnect]: Microsoft Flight Simulator SimConnect SDK pages describe out-of-process/in-process clients and supported API calls plus asynchronous communication model; useful baseline for bridge design and throughput risk framing. https://docs.flightsimulator.com/html/Programming_Tools/SimConnect/SimConnect_SDK.htm and https://docs.flightsimulator.com/msfs2024/html/6_Programming_APIs/SimConnect/API_Reference/Communication/Communication_API.htm
