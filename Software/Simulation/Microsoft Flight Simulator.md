---
title: Microsoft Flight Simulator
aliases:
  - MSFS
  - Microsoft Flight Simulator
  - Microsoft Flight Simulator 2020
  - Microsoft Flight Simulator 2024
tags:
  - simulation
  - flight-simulation
  - reinforcement-learning
  - pufferlib
  - simconnect
  - c
  - cpp
  - csharp
---

# Microsoft Flight Simulator

`Microsoft Flight Simulator` (MSFS 2020 and MSFS 2024) is a commercial, closed-source flight simulation platform with a documented add-on programming surface via the Microsoft Flight Simulator SDK and `SimConnect` API.

For RL, the key point is the existence of real API surfaces, not that the flight model itself is directly exposed.

---

## Research status

| Area | What is confirmed | Confidence |
|---|---|---:|
| API layer availability | `SimConnect` SDK exists and documents an explicit client API for both in-process and out-of-process usage (C/C++, .NET). | High |
| In/out-of-process model | Out-of-process add-ons are supported (including C, C++, `.NET`); in-process modules are C++ `WASM`. | High |
| Thread model | SDK docs explicitly state SimConnect clients are not currently thread-safe. | High |
| Core workflow | Add-on client opens a connection to the simulator server, sends/receives events + object/state, and processes responses. | High |
| PufferLib fit | Best chance is a deterministic bridge process (custom C++/C# IPC adapter), not direct embedding. | Medium |
| Rendering | Full simulator rendering stack is closed; no public headless RL runtime mode in SimConnect docs. | Medium |
| Throughput figures | No official RL-style SPS/step benchmark in public SDK docs. | Medium |

---

## 1) What MSFS is good for in this context

The SDK is designed for add-on creation and external tooling:

- package-based add-on workflow via Developer Mode;
- `SimConnect` for events, object state, custom client data, and external telemetry/control;
- official sample/tools ecosystem around DevMode and SimConnect debugger utilities.

That makes MSFS useful as a **high-fidelity visual + interaction validation layer**, especially when you need to test policy behavior against:

- realistic airframes/scenery,
- mission/ATC-like context,
- commercial-quality display and control stack.

It is less ideal as a native RL source of `step()` unless you build your own bridge contract.

---

## 2) Implementation surface (what is actually available)

### SimConnect (core API)

The SimConnect pages list functions for:

- opening/closing simulator connection,
- requesting/reading data definitions (simulation variables),
- transmitting events,
- writing data back to simulation objects,
- AI object/flight state operations,
- custom client data exchange.

The API is event/message driven and explicitly documented in the official function tables.

### In-process vs out-of-process

The docs support both:

- **Out-of-process apps** (`.exe`, C++/C#/VB): easier to debug, safer failure mode, easier to run as dedicated RL bridge.
- **In-process WASM modules**: can be integrated more tightly in simulator lifecycle, but lifecycle and thread-safety behavior is riskier for RL loops.

### Thread-safety and timing

MSFS explicitly notes SimConnect clients are not thread-safe. RL bridges that rely on high-frequency concurrent callbacks must serialize access and keep callback work minimal.

### Communication behavior

The communication APIs are asynchronous (event/message style) and use explicit callback/message dispatch handling, not a synchronous function-per-step RPC loop.

---

## 3) Rendering and architecture implications

MSFS runs a full renderer and world/scenery stack. The SDK docs position `SimConnect` as add-on communication, not as a replacement RL simulator core.

This matters because RL throughput is bounded by:

- IPC/callback scheduling,
- event processing latency,
- frame-bounded simulation progression in some scenarios.

Without documented deterministic fixed-rate "step" in public docs, you should treat this as an **externally sampled control loop** unless you measure otherwise.

---

## 4) PufferLib integration strategy

### Recommended architecture: dedicated bridge process (Pattern A)

**Pattern**: out-of-process native client (`C++`/`C#`) owning the deterministic training loop; simulator runs as separate process.

1. `bridge_init`:
   - `SimConnect_Open`
   - define readable state schema using `SimConnect_AddToDataDefinition`/`SimConnect_RequestDataOnSimObject` analogs
2. `step(action)` in host process:
   - write control changes/events via SimConnect calls
   - call/drive dispatch pump
   - read bounded state snapshot into contiguous buffers
3. `reset(seed)`:
   - load fixed start state through supported load/save or event state APIs where possible
4. expose:
   - `obs, reward, terminated, truncated` in fixed memory layout for PufferLib

### Why out-of-process is the first choice

Official SDK guidance explicitly prefers out-of-process for stability and debug/compatibility reasons.

### Alternate architecture: in-sim module (Pattern B)

Possible for gauge/C++ integration, but higher risk:

- thread-safety restrictions,
- stronger coupling to sim internals/versioning,
- limited isolation if crashes propagate.

### Bridge quality checks

For RL loops, add:

- one client state machine thread only,
- strict timestamped action sequencing,
- bounded queue to avoid dropped dispatches,
- replay check for determinism on fixed seeds.

---

## 5) Throughput estimate (for PufferLib context)

There is no public, official MSFS RL throughput benchmark for `step/s`.

Practical inference for this stack:

- Better than GUI-only automation, because you have structured API access,
- likely far slower than a native C99/JSBSim-style engine due to IPC + callback pump,
- more reproducible than pixel-only automation but still not a true batched RL runtime.

Treat any numbers as measured in your own harness.

---

## 6) Comparison against current candidates

| Candidate | API quality | Rendering overhead | RL integration risk | Best role in your stack |
|---|---|---|---|---|
| **MSFS + SimConnect** | Medium-High (documented C/C++/C# API) | High (full consumer simulator stack) | Medium-High (thread-safety + event cadence constraints) | High-fidelity policy validation and realistic visual regression |
| [[JSBSim]] | High (native dynamics API, headless possible) | Low (no mandatory renderer in core path) | Medium (language bridging + C++ binding path needed) | Primary training core |
| [[FlightGear]] | High (protocol + property stack) | High (full sim/renderer) | Medium | Flight stack visual + full-sim behavior checks |
| [[X-Plane]] | High (C/C++ SDK + web API alternatives) | High | Medium | Secondary high-fidelity benchmark target |
| [[ClearView RC Flight Simulator]] | Low for direct RL | High | High | RC human-practice/reference model |

---

## 7) Candidate score for your goal (realistic RL physics)

- **Physics realism / realism pipeline**: B+
- **Direct RL loop practicality**: C
- **PufferLib integration effort**: B
- **Headless/vectorized throughput expectation**: C
- **Legal/commercial friction**: B (SDK available but commercial platform)

Interpretation: MSFS is a strong **transfer and policy robustness target**, but your high-throughput base loop should still be JSBSim-first.

---

## 8) Useful next artifacts

- `Flight-Sim Migration Funnel` notes (see [[Flight-Sim Migration Funnel]])
- protocol bridge skeleton for SimConnect out-of-process adapter
- replay harness that normalizes `action_map`, `obs_map`, `sim_speed`, and seed lifecycle

---

## 9) Related notes

- [[PufferLib]]
- [[PufferLib C99 Environment Authoring]]
- [[Flight-Sim RL Contract]]
- [[Flight-Sim RL Contract]]
- [[JSBSim]]
- [[FlightGear]]
- [[X-Plane]]
- [[Flight-Sim Migration Funnel]]
- [[ROS2 Aerial Integration]]

---

## Source notes

[^msfs-simconnect]: SimConnect SDK API capability, language support, and API setup details. https://docs.flightsimulator.com/html/Programming_Tools/SimConnect/SimConnect_SDK.htm
[^msfs-simconnect-api]: SimConnect function reference surface (`Open`, data definitions, event transmission, request/write calls). https://docs.flightsimulator.com/html/Programming_Tools/SimConnect/SimConnect_API_Reference.htm
[^msfs-simconnect-comms]: SimConnect communication API notes: JSON for inter-platform payloads, caller IDs, asynchronous calls. https://docs.flightsimulator.com/msfs2024/flighting/html/6_Programming_APIs/SimConnect/API_Reference/Communication/Communication_API.htm
[^msfs-threadsafe]: SimConnect clients are not currently thread-safe (MSFS 2024 SDK). https://docs.flightsimulator.com/msfs2024/html/6_Programming_APIs/SimConnect/SimConnect_SDK.htm
[^msfs-sdk-overview]: MSFS SDK purpose, add-on package workflow, and DevMode-based creation. https://docs.flightsimulator.com/html/Introduction/SDK_Overview.htm and https://docs.flightsimulator.com/html/Introduction/Introduction.htm
[^msfs-release]: MSFS 2024 is a standalone product family successor to MSFS 2020 with cross-platform release context. https://www.flightsimulator.com/microsoft-flight-simulator-2024-release-date-announcement/
