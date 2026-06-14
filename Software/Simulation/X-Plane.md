---
title: X-Plane
aliases:
  - X-Plane
  - XPlane
tags:
  - simulation
  - flight-simulation
  - reinforcement-learning
  - pufferlib
  - c
  - cpp
  - plugin
  - web-api
---

# X-Plane

[[X-Plane]] is a closed-source commercial simulator with a mature plugin ecosystem and a local integration API surface.  
For RL, that means you get rich simulation state/control plumbing, but no native `step/reset` environment contract. You have to build an adapter around the simulator loop.

---

## 🧭 Research Status

| Claim | Confidence | Evidence |
| --- | ---: | --- |
| Closed commercial engine with non-public core physics implementation | High | Official product and developer resources identify a commercial product with vendor-controlled core simulation/runtime; SDK does not expose solver source. |
| SDK integration is C/C++ plugin-based (XPLM) via plugin manager callbacks/datarefs/commands | High | Developer docs and SDK guide specify C/C++ headers, plugin manager lifecycle, and XPLM APIs. |
| There is also a local Web API for datarefs and commands (v3) | High | Developer web API doc and SDK command/docs mention local REST/WebSocket and command execution. |
| Native multi-instance RL vectorized stepping is not present | High | No documented built-in RL step abstraction; integration is callback/API-driven. |
| Headless/high-SPS mode is not a primary advertised mode | Medium | No published simulator-wide SPS/step-rate benchmarks for RL-style loops; Web API and plugin callback paths imply overhead and frame-tied pacing. |

---

## 🧠 What X-Plane Gives You for RL

**What it does well**

- High-fidelity closed engine for flight dynamics and systems, including large ecosystem of add-on aircraft/scenery.
- Plugin APIs (`XPLM...`) for deterministic-enough action/state access if you own the control loop.
- Local Web API for easier, language-agnostic prototyping (dataref read/write, subscriptions, commands).

**What it does *not* give you**

- A built-in `env.step()` or environment manager.
- Open-source FDM core you can inspect/modify.
- A native vectorized rollout runtime that maps directly to PufferLib-style thousands of copies.

---

## 🧩 Integration Surface (Official)

### Plugin/API stack

The SDK stack is centered around the XPLM plugin manager and C API:

- `XPLM` headers + SDK docs for plugin lifecycle.
- Platform plugin binaries are built as shared objects/dlls (macOS `.xpl`, Windows `.xpl`/`.dll`, Linux `.so`) and loaded under `Resources/Plugins`.[^xplm-install]

Core API groups used by RL wrappers include:

- **Data Access (`XPLMDataAccess`)**: locate and read/write simulator variables (“datarefs”).
- **Processing (`XPLMProcessing`)**: register callbacks into the sim loop (good place to apply an action and sample next state).
- **Planes (`XPLMPlanes`)**: access/initialize aircraft state at init boundaries.
- **Plugin Manager (`XPLMPlugin`)**: lifecycle hooks and enable/disable behavior.

These are the expected hooks for a custom PufferLib bridge.[^xplm-data][^xplm-processing][^xplm-planes][^xplm-plugin]

### Rendering context

X-Plane’s modern rendering stack has moved to modern GPU backends (e.g., Vulkan/Metal era in 11.50+ notes), and there are compatibility notes about plugin drawing APIs under those engines. For RL, this matters because rendering and custom drawing extension behavior is less stable than dataref/control access when targeting high-frequency loops.[^xp1150-rn][^xp11-compat][^xp-gl]

### Local Web API

X-Plane exposes a local API server on localhost and documents versioned endpoints.  
It supports:

- capability/version introspection
- dataref subscribe/read/write
- command-style interactions

This is useful for quick external integrations, but message serialization + websocket/HTTP overhead add latency compared with in-process plugins.[^xplane-webapi]

---

## 🛠 PufferLib Integration: Concrete Patterns

You need **one of three adapter patterns**.

### Pattern A — In-process plugin (recommended for control fidelity)

This is usually the best route if you need tight control and better throughput than web calls.

1. Build a C/C++ plugin that registers a flight-loop callback.[^xplm-processing]
2. Host side pushes action vectors into a lock-free/shared queue.
3. In callback:
   - map actions to datarefs/commands;
   - call step of internal logic;
   - read back normalized observation fields into a fixed contiguous buffer.
4. PufferLib reads `obs/action/reward/done/info` from that buffer.

Why this is closest to PufferLib:

- avoids REST overhead,
- avoids per-step JSON parsing,
- gives deterministic placement for `obs`/`reward` layout and reset behavior.

### Pattern B — Local Web API bridge (fastest to bootstrap)

1. Start X-Plane with web server enabled on localhost.
2. Python control process:
   - subscribe to required datarefs,
   - send command/dataref writes per step,
   - map responses into PufferLib tensors.
3. Reward/done computed in host process.
This gets working quickly, but per-step IPC overhead and simulator pacing make SPS hard to push high unless your step semantics are very sparse.

### Pattern C — Hybrid/legacy tooling (`XPlaneConnect`)

- `XPlaneConnect` offers a ready-made external bridge (plugin + client libraries).[^xpc-wiki]
- It is still useful for prototyping, but the README documents compatibility up to X-Plane 11 and shows no modern-version notes; compatibility for current X-Plane 12 should be validated.[^xpc-wiki]
- issue history includes XP12 interoperability concerns.[^xpc-issues]

---

## 🕹 Threading and Determinism Risks

X-Plane plugin guidance repeatedly warns that plugin code is not a free-threaded environment; API callbacks are not designed as fully thread-safe as general multithreaded code.[^xp-thread]  
So:

- keep callback work minimal and bounded,
- do heavy work in a separate process/thread and only perform minimal enqueue/dequeue and dataref reads in callbacks,
- avoid using PufferLib worker counts that expect re-entrant simulator internals.

Because rendering/simulation are one simulator instance, very high vectorized rollouts still require multiple simulator processes/machines.

---

## ⏱ Throughput / SPS Expectations

No authoritative public “SPS” metric exists for X-Plane in RL docs. Two practical bounds:

- **Plugin path:** fastest among X-Plane options, but still tied to simulator main loop and callback latency.
- **Web API path:** easiest but typically lower throughput due serialization + socket transport.

Treat this as a measured quantity in your stack, not a published benchmark.

For realistic RL experiments where SPS is critical, many teams run a pure FDM core (e.g., [[JSBSim]]) for training and use X-Plane as a later-stage sim fidelity check.

---

## 🧭 RL Candidate Comparison (5+)

| Candidate | Physics source | API shape | RL integration difficulty | Rendering overhead | Notes for realism-first RL |
| --- | --- | --- | --- | --- | --- |
| [[X-Plane]] | Closed commercial engine | C/C++ plugin + local web API + XPlaneConnect legacy | Medium | High (full sim + GPU) | Strong fidelity, medium throughput uncertainty |
| [[JSBSim]] | Open 6-DoF FDM core | C++, Python, CLI, socket I/O | Low-Medium | Low (headless) | Best open-source baseline for high SPS training |
| [[FlightGear]] | Open framework + multiple FDMs | CLI, property tree, protocol interfaces | Medium | Medium-High | Strong visual stack, less clean RL contract |
| [[ClearView RC Flight Simulator]] | Proprietary RC sim | No public RL API surfaced | High | High | Better for human RC practice than RL loop |
| [[CRRCSim]] | Open legacy RC sim | CLI/input/manual controls | High | Medium | Useful model corpus, old architecture |
| XPlaneConnect | UDP bridge around X-Plane | Plugin + client libs (legacy) | Medium | Medium | Quick prototype, compatibility risk on new X-Plane |

---

## ✅ Recommendation for your goal (realistic flight-physics RL)

- If your primary objective is **maximum realism plus full flight stack**, X-Plane is a strong reference/validation simulator.
- If your primary objective is **fast research throughput with controllable step semantics**, build your core environment around open dynamics first ([[JSBSim]]) and connect X-Plane via a plugin/web bridge for transfer checks.
- For PufferLib production loops, the pragmatic architecture is:
  1. `[[JSBSim]]`-style physics core for training,
  2. X-Plane plugin/web observer environment for policy transfer and visual/systems validation,
  3. only add full X-Plane rollout density once reward/obs design is stable.

---

## 🔗 Related Notes

- [[PufferLib]]
- [[PufferLib C99 Environment Authoring]]
- [[JSBSim]]
- [[FlightGear]]
- [[ClearView RC Flight Simulator]]
- [[CRRCSim]]
- [[Flight-Sim RL Contract]]
- [[PufferLib Flight Throughput Benchmark]]

---

## 🧾 Source Notes

[^xplm-sdk]: X-Plane SDK landing page and developer ecosystem (language/API expectations): https://developer.x-plane.com/sdk/
[^xplm-plugin]: Plugin architecture, lifecycle, and install packaging details: https://developer.x-plane.com/sdk/XPLMPlugin/
[^xplm-install]: Plugin build/install workflow and directory layout (`Resources/plugins`, fat plugins, API levels): https://developer.x-plane.com/article/building-and-installing-plugins/
[^xplm-processing]: Flight-loop callback and processing APIs: https://developer.x-plane.com/sdk/XPLMProcessing/
[^xplm-data]: Data access API (DataRefs): https://developer.x-plane.com/sdk/XPLMDataAccess/
[^xplm-planes]: Planes API for aircraft/flight-state helpers: https://developer.x-plane.com/sdk/XPLMPlanes/
[^xplane-webapi]: X-Plane Web API docs (local endpoint, versioned API, datarefs/commands/subscriptions): https://developer.x-plane.com/article/x-plane-web-api/
[^xpc-wiki]: XPlaneConnect readme/wiki describing legacy UDP bridge and language clients: https://github.com/nasa/XPlaneConnect/wiki/Getting-Started and https://raw.githubusercontent.com/nasa/XPlaneConnect/master/README.md
[^xpc-issues]: XPlaneConnect issue history includes ongoing compatibility requests for newer X-Plane versions: https://github.com/nasa/xplaneconnect/issues
[^xp11-compat]: X-Plane 11.50 plugin compatibility notes (deprecated drawing callbacks and Vulkan-era constraints): https://developer.x-plane.com/article/plugin-compatibility-guide-for-x-plane-11-50/
[^xp1150-rn]: X-Plane 11.50 release notes and renderer context: https://www.x-plane.com/kb/x-plane-11-50-release-notes/
[^xp-thread]: Plugin thread-safety guidance: https://developer.x-plane.com/2019/02/the-plugin-sdk-is-not-even-remotely-thread-safe/
[^xp-gl]: OpenGL plugin-drawing guidance (and cautions under Vulkan/Metal): https://developer.x-plane.com/article/plugin-guidance-for-opengl-drawing/
