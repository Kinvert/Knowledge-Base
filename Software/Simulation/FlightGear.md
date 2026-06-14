---
title: FlightGear
aliases:
  - FlightGear
  - FGFS
  - FlightGear Flight Simulator
tags:
  - simulation
  - flight-simulation
  - reinforcement-learning
  - pufferlib
  - cpp
  - c
  - nasal
---

# FlightGear

FlightGear is an open-source flight simulator framework with a long engineering history and multiple integration paths, but it is not a “ready-to-plug” RL environment in the Gym/PufferLib sense. It is strongest as a full-stack sim stack (FDM + rendering + scenery + I/O), with external control APIs that can be wrapped into PufferLib runners when you accept the rendering and latency trade-offs.[^fg-about][^fg-wiki][^fg-io]

---

## 🧭 Research Status

| Claim Type | What is known | Confidence |
| --- | --- | --- |
| Licensing / openness | GPL source code, open model, active project, free downloads on Win/macOS/Linux/FreeBSD | High |
| Core internals | Written in C/C++ with Nasal scripts; multiple FDMs; network and property-tree interfaces | High |
| Rendering stack | OpenSceneGraph front-end since v1.9.0; full visual pipeline is substantial | High |
| RL-facing API | No native step/reset API in public docs; integration is via control/data I/O paths | Medium |
| PufferLib fit | Best as external process wrapper (not native environment module); likely lower SPS due rendering | Medium |
| Headless | `--disable-gui` exists, but wiki notes “headless” still incomplete/renderer-costly historically | Medium |

---

## 📌 What FlightGear Is

From the project site and wiki:
- Open-source/free and cross-platform (Windows, macOS, Linux, FreeBSD).[^fg-about][^fg-wiki]
- GPL-licensed with source available for inspection and redistribution.[^fg-about][^fg-license]
- Focuses on research/academic/industry use cases in addition to desktop training.[^fg-about]
- Active status with release cadence continuing into 2026 (latest stable 2024.1.6 on 4 Jun 2026).[^fg-release][^fg-releasepost]

For RL, the practical question is not “can it simulate flight” (yes), but whether it can sustain **high sample efficiency workflows** (depends on how you bypass renderer and coupling).

---

## 🏗️ Core Architecture & Rendering

### Simulation stack

FlightGear core is written in C/C++ and aircraft behavior often uses Nasal scripts; the wiki describes Nasal as a main integration mechanism for property extensions and custom subsystems.[^fg-wiki][^fg-properties]

It supports multiple FDMs (`jsb`, `yasim`, `network`, `pipe`, `external/null`, etc.) and aircraft-specific models via normal aircraft selection flags. FlightGear documents both JSBSim and YASim as its commonly used FDMs, with JSBSim generally positioned for data-driven high-control setups and YASim for easier geometry-started starts.[^fg-manual-fdm][^fg-fdm-vs]

For rendering, FlightGear has used OpenSceneGraph since version 1.9.0 (replacing PLIB), and the rendering stack has a dedicated compositor-style pipeline/history.[^fg-osg]

### High-level architecture context

The wiki’s architecture page explicitly describes a federated, multi-component architecture and multi-core intent, and notes external-language interop goals via separate binaries and threads.[^fg-arch]

Interpretation: the project is built as a full simulator framework, not a thin deterministic environment library.

---

## 📡 APIs / I/O Paths (important for RL)

FlightGear’s exposed automation surfaces are not RL-specific; they are simulator control and streaming interfaces:

- **Command-line protocol channels**: `--native`, `--native-ctrls`, `--native-fdm`, `--native-gui`, `--generic`, plus others via options (`--props`, `--httpd`, `--telnet`).[^fg-cli-protocol][^fg-cli-network]
- **Property tree web server / telnet**: built-in support for HTTP and telnet property access and mutation during runtime.[^fg-property-tree]
- **Generic protocol (XML)**: configure custom input/output data streams over UDP, sockets, files, etc.[^fg-generic]
- **Nasal scripting**: property-tree customization and subsystem scripting, including script-side augmentation without rebuilding C++.[^fg-properties]

Important practical detail: these are all external-state/control channels; there is no documented built-in RL-style `step()/reset()` semantic API in the primary docs set.[^fg-no-rl]

### FDM coupling for external control

FlightGear exposes explicit options for external FDM integration:
- `--fdm=network,...` with host/ports and UDP transport, including note that UDP networking can be non-deterministic (packet loss/reordering risks).[^fg-manual-fdm]
- `--fdm=pipe` as alternative using shared same-machine named-pipe-style property/name-value transport.[^fg-manual-fdm]
- `--fdm=external` (alias null behavior) to disable internal FDM generation for external drivers.[^fg-manual-fdm]

There is also `--native-fdm`/`--native-ctrls` to connect through FlightGear native packet formats.[^fg-cli-protocol]

---

## 🎞️ Rendering and Visual Fidelity

FlightGear is primarily a full simulator frontend:
- renderer: OpenSceneGraph-based pipeline
- realistic scenery/world/world data model
- full cockpit/ATC/multi-aircraft workflows etc., because this is a visual sim stack, not a bare physics service.[^fg-osg][^fg-about]

The rendering pipeline is a major reason to use FlightGear for **visual validation and demo**, and a potential reason to limit it in pure RL throughput experiments.

---

## ⏱️ Headless and Throughput Reality

Current wiki discussions indicate headless handling was historically incomplete; `--disable-gui` reduces GUI interaction but not full visual rendering cost in practice, and users reported coupling between scenery loading and rendering for collision/elevation behavior.[^fg-headless]

So:
- If you need training-at-scale throughput, you should not assume clean “headless RL mode.”
- You can likely run low-window/headless-ish strategies and fixed-rate loops, but this is an **engineering workaround path**, not a clean native RL execution mode.

---

## 🧠 How FlightGear Maps to PufferLib

### Recommended harness patterns

1. **Render-only supervision path (most realistic for FlightGear integration)**
   - Keep FG as visual front-end + state broadcaster (`--httpd` or property tree channel).
   - RL actor loop runs in separate process using PufferLib C99 env for step/reset.
   - Pros: reuse FG visuals/scenery/weather scripts.
   - Cons: extra IPC and lower SPS.

2. **Native-process wrapper environment (advanced)**
   - One FG instance per env worker.
   - Env step implementation:
     - write action/controls via native or generic protocol,
     - read property-tree snapshot/state packets,
     - produce reward/done externally.
   - Pros: true FlightGear stack integration.
   - Cons: environment lifecycle, time-sync, and reset fidelity are hard.

3. **External FDM driver + FG visual proxy (best trade for repeatability)**
   - Use FG with `--fdm=network` or `--fdm=pipe` to consume states from a separate deterministic driver.
   - Keep RL physics logic in your driver; use FG for rendering and sensor/state extraction only.
   - Pros: cleaner deterministic core path; easier to decouple physics from render overhead.
   - Cons: still has IPC + parsing overhead.

### PufferLib-specific implementation notes

- PufferLib’s high-throughput path assumes contiguous buffers and C-native stepping behavior in env bindings.[^puffer-docs]
- That favors a thin adapter that emits fixed-size vectors each step, rather than ad-hoc Python object graphs.
- If you need FPS/SPS benchmarking, avoid starting with `Nasal` roundtrips first; use compiled C/C++ adapters for parsing/protocol pumping first, then optimize.
- Prefer `--fdm=pipe` if you need stronger determinism (single host), because docs explicitly call out UDP network non-determinism tradeoffs for `--fdm=network`.[^fg-manual-fdm]

---

## 📈 Throughput Expectations (SPS) for PufferLib

### What is known
- No official FlightGear step-rate (SPS) figures are published in the core project documentation for generic RL workloads.[^fg-no-sps]
- PufferLib’s native backend is designed for very high throughput on environments that can run in contiguous-memory C loops (and does not guarantee that a GUI simulator can do the same).[^puffer-throughput]

### Inferred estimate (explicit)
- If you run FlightGear in any mode that includes rendering and protocol marshalling, practical wall-clock throughput is likely **well below Ocean-style 1M+ SPS**, especially compared to JSBSim-as-C env wrappers.
- If you disable rendering and run FG in strict fixed-rate headless-like mode, you may improve throughput but should validate empirically; docs still warn that practical headless behavior can remain tied to rendering/scenery loops.[^fg-headless]

This estimate should be treated as **hypothesis until profiled** in your exact architecture (protocol type, number of envs/workers, observation dimensionality, reward computation, rendering mode).

---

## 🆚 Comparison Chart (Current Candidate Set)

| Platform | Primary Physics Core | Rendering | API/Integration Style | RL Harness Difficulty | Best Role vs PufferLib |
| --- | --- | --- | --- | --- | --- |
| FlightGear | Multi-FDM stack (JSBSim/YASim/External) | Full OpenSceneGraph sim | CLI + property tree + native/generic/telnet/http protocols | Medium-High | Good for visual simulation + limited RL throughput |
| [[JSBSim]] | C++ FDM engine (data-driven tables, scriptable) | None (externalized rendering) | C++/Python module, socket properties | Medium-Low | Best when physics-first + high throughput |
| [[X-Plane]] | Closed high-fidelity proprietary FDM | Full renderer | Plugin SDK + datarefs | Medium | Good for fidelity studies, harder reproducible open stack |
| [[ClearView RC Flight Simulator]] | Proprietary RC flight engine | Proprietary | No public RL-like API found | High | Good for manual RC feel; poor for RL control-loop |
| CRRCSim | Open-source RC sim stack | OpenGL-era graphics | Legacy/source-age uncertain | Medium | Niche/legacy; limited modern RL tooling |
| Flightmare | Flexible Python-first with Unity renderer + Python API | Full rendering, RL-oriented wrappers | Python API with batch/parallel multi-agent targets | Low-Medium | Better for RL experiments if you can accept different sim fidelity profile |

---

## 🧪 What to Build First (if your goal is realistic RL on flight physics)

1. Start with a **pure-physics baseline** in JSBSim (already done in other notes) to lock on deterministic dynamics and reward shaping.
2. Add a FlightGear visual harness using one of:
   - FG property tree state pull,
   - FG native protocol,
   - external FDM with FG as renderer.
3. Compare policy transfer across:
   - direct JSBSim env,
   - FG-rendered env,
   - optionally X-Plane dataref baseline (if licensing/availability allows).

The highest-value output is often a **small, consistent observation schema** (AoA, AOA, VCAS, bank/pitch rates, altitude, position, energy, actuator deflections) with hard resets from CLI-defined start states.

---

## 📝 Source Notes

- Most integration facts come from FlightGear wiki/manual pages and official website pages; several historical notes are old but still relevant to current behavior assumptions (especially headless).
- For every “inferred” performance claim, validate against your hardware with a benchmark harness before selecting your final stack.

---

## 🔗 Related Notes

- [[JSBSim]]
- [[ClearView RC Flight Simulator]]
- [[PufferLib]]
- [[PufferLib C99 Environment Authoring]]
- [[Reinforcement Learning]]
- [[Vectorized Environments]]
- [[Nasal]] — FlightGear scripting context and property-tree extension behavior
- [[Flight-Sim RL Contract]]
- [[PufferLib Flight Throughput Benchmark]]

---

## 📚 Source Notes

[^fg-about]: FlightGear official site (“About” and “Features”), including open-source mission, cross-platform statement, GPL code availability, and research/education orientation. https://www.flightgear.org/about/ and https://www.flightgear.org/about/features
[^fg-license]: FlightGear license page, states GPL license and user freedoms. https://www.flightgear.org/about/license/
[^fg-wiki]: FlightGear wiki page with project metadata: GPL, release date, languages C/C++, platforms, and active status. https://wiki.flightgear.org/FlightGear
[^fg-release]: FlightGear homepage listing current release and 2026 timeline. https://www.flightgear.org/
[^fg-releasepost]: 2024.1.6 release post on 4 June 2026. https://www.flightgear.org/blog/release-2024-1-6
[^fg-osg]: OpenSceneGraph integration details and FlightGear using OSG since v1.9.0. https://wiki.flightgear.org/OpenSceneGraph
[^fg-properties]: Property tree integration and Nasal extension path in the property system. https://wiki.flightgear.org/Property_Tree
[^fg-property-tree]: Property tree page: built-in web and telnet interfaces, multiple interface options. https://wiki.flightgear.org/Property_tree
[^fg-generic]: Generic protocol page: concurrent IO, FGNetFDM/native, XML-defined protocol, UDP, and custom generic streams. https://wiki.flightgear.org/Generic_Protocol
[^fg-io]: Command line pages and protocol pages document native/generic/telnet/http/props integration. https://wiki.flightgear.org/Command_line_options , https://wiki.flightgear.org/Property_Tree/Native_Protocol_Slaving , https://wiki.flightgear.org/Property_Tree
[^fg-cli-protocol]: Command-line protocol switches: `--native`, `--native-ctrls`, `--native-fdm`, `--generic` etc. https://wiki.flightgear.org/Command_line_options
[^fg-cli-network]: Network interfaces including `--httpd`, `--telnet`, and other transport options from manual command reference. https://flightgear.sourceforge.net/manual/next/en/getstart-ench4.html
[^fg-manual-fdm]: Flight model options (`jsb`, `yasim`, `network`, `pipe`, `external`) and notes on external FDM transport. https://flightgear.sourceforge.net/manual/next/en/getstart-ench4.html
[^fg-no-rl]: No built-in Gym-style step/reset API is documented in these core integration pages; integration surfaces are protocol/IO-based and runtime state property surfaces. https://wiki.flightgear.org/Command_line_options , https://wiki.flightgear.org/Property_Tree , https://wiki.flightgear.org/Generic_Protocol
[^fg-headless]: Headless notes indicate `--disable-gui` and remaining rendering/scenery coupling concerns in historical docs. https://wiki.flightgear.org/FlightGear_Headless
[^fg-arch]: High-level architecture description with multi-federate and multi-language extension intent. https://wiki.flightgear.org/High-Level_Architecture
[^fg-fdm-vs]: JSBSim vs YASim comparison notes and model realism tradeoffs. https://wiki.flightgear.org/JSBSim_vs_YASim
[^fg-no-sps]: No explicit RL throughput/SPS claims in reviewed FlightGear docs for RL-style workloads. (No equivalent “SPS table” page found in official sources consulted.)
[^puffer-docs]: PufferLib docs showing native/O( C ) environment design, contiguous observation/action/reward/terminal memory, and OMP/CUDA-vectorized execution model. https://puffer.ai/docs.html
[^puffer-throughput]: PufferLib documentation and claims of 1M+ SPS environments are for tuned Puffer stacks and are not FG-specific. https://puffer.ai/docs.html and https://www.mintlify.com/PufferAI/PufferLib/introduction
