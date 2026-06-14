---
title: JSBSim
aliases:
  - JSBSim FDM
  - JSBSim Flight Dynamics Model
tags:
  - simulation
  - flight-simulation
  - reinforcement-learning
  - pufferlib
  - cpp
  - python
---

# JSBSim

JSBSim is a data-driven, non-linear, six-degree-of-freedom flight dynamics model written in C++, with first-class support for scripting and vehicle definition in XML, and explicit hooks for external rendering/control integration. It is one of the strongest options among flight sims for realistic physics modeling in RL when you need a reusable flight engine rather than a full game-like simulator UI.[^jsbsim-home][^jsbsim-manual-overview]

---

## 🧭 Research Status

| Item | What is known | Confidence |
| --- | --- | --- |
| Core technology | C++ FDM core, open source, LGPL-2.1 licensed[^jsbsim-license] | High |
| Simulation modes | Standalone batch mode (no graphics) and integration into larger frameworks[^jsbsim-readme] | High |
| Engine API | C++ `FGFDMExec` + Python binding + CLI; no explicit official C API listed in public docs | High (for listed APIs) |
| Rendering model | No in-library renderer; sends output via FlightGear socket or runs headless batch[^jsbsim-output] | High |
| PufferLib fit | Best via custom C/C++ env binding; Python wrappers are functional but lower throughput path | Medium |
| SPS confidence | No canonical published JSBSim "steps/s" figure in JSBSim docs[^jsbsim-readme] | Medium (inference required) |

---

## 📌 What JSBSim Is (and Isn’t)

JSBSim started for `FlightGear` and remains oriented around being a flight **dynamics engine**, not a full rendered sim.[^jsbsim-home]  

Key points:

- It supports 6-DoF flight dynamics and configurable physics via XML.[^jsbsim-home]
- It is multi-platform and designed for both standalone batch simulation and integration with larger frameworks, including `Unreal` and `FlightGear`.[^jsbsim-home][^jsbsim-readme]
- The executable and Python module installation are explicitly documented as console/terminal-first entry points, not as a native renderer.[^jsbsim-release]

For RL work, that matters: you care about repeatable state transitions and step cost more than cockpit rendering.

---

## 🏗️ Core Engine Architecture

### Physics representation

JSBSim is data-driven and object-oriented: you define the airframe, propulsion, control system, and environment behavior through aircraft/model files (`.xml`) rather than hard-coded flight modes in source.[^jsbsim-home]

The project documents this as:

- configurable control system and propulsion
- configurable landing gear and mass properties
- fully explicit Earth model handling for inertial calculations[^jsbsim-home]

### Runtime cadence and integration

`FGFDMExec` is the central executive in the C++ model.[^jsbsim-fgf]  

Both C++ and Python docs show the same basic cadence:

- set/load aircraft model + script
- initialize from IC
- loop one `Run()` per control decision/timestep (`run()` in Python)[^jsbsim-fgf]

That structure is exactly the kind of “single-step transition” shape RL expects when wrapped carefully.

### CLI and command behavior

The standalone executable exposes command-line control, including help/version flags via `--help`/`--version`, reinforcing that simulation can be scripted, headless, and batched.[^jsbsim-source]

---

## 🧠 Language and API Surface

### Officially documented interfaces

- `C++` core API (`FGFDMExec`, `FGFDMExec::Run`, `RunIC`, etc.)[^jsbsim-readme]
- `Python` module described as feature-equivalent to C++ core in docs and examples[^jsbsim-readme][^jsbsim-fgf]
- CLI wrapper and scripted operation (`jsbsim` executable + `--script`, `--help`, `--version` variants)[^jsbsim-source]

### API extensions / bindings listed in docs

JSBSim docs explicitly list bindings for:

- Python module (LGPL)
- MATLAB S-Function
- Unreal Engine plugin integration

No separate “first-class C API” is advertised in the same surface as the above; this matters if you were hoping to skip bindings and call into a stable C ABI directly.[^jsbsim-readme]

### Data and I/O plumbing

JSBSim supports property-oriented modeling and exposes socket-style input/output channels in the output/input systems:

- `FGInputSocket` / `FGUDPInputSocket` for runtime property updates over sockets
- `FGOutput` supporting CSV, TERMINAL, and FlightGear output packets (`FLIGHTGEAR` type), plus socket output notes

The docs explicitly show socket input flow and `FLIGHTGEAR` as a dedicated output path; this is important for building headless/visualized RL stacks.[^jsbsim-input][^jsbsim-output]

---

## 🎥 Rendering / Visualization

JSBSim itself is not a renderer.[^jsbsim-home][^jsbsim-output]  

Your options for visuals are external:

- `FlightGear` and other frameworks can consume JSBSim state through external interfaces.[^jsbsim-home]
- `FLIGHTGEAR` output path in `FGOutput` can transmit binary packets to an external FlightGear instance.[^jsbsim-output]
- Unreal plugin path is explicitly offered in docs as another integration route.[^jsbsim-readme]

So for RL:
- Use **no-render mode** for throughput experiments.
- Use separate visual path only for human debugging and episode inspection.

---

## 🔄 PufferLib Integration Plan

The important question is not “can it run in Python” but “can it hit PufferLib’s throughput envelope?”

### Why this is a good fit

- deterministic stepping model (`Run`) and clear init/reset points (`RunIC`) map well to `step`/`reset` patterns
- no built-in rendering means fewer GPU/UI bottlenecks at core
- configurable models mean you can choose lightweight and deterministic aircraft for training

### Recommended integration path (practical)

1. Use C++ as the integration surface for the environment core (directly embed or bind `FGFDMExec` to PufferLib C99 environment pattern).[^puffer-docs]
2. Keep state in contiguous observation/action buffers as required by PufferLib and avoid per-step heap allocation.
3. Build a small adapter layer:
   - `reset`: load script, initialize IC, clear previous output
   - `step`: write control properties from action vector, call one simulation tick, read deterministic output properties into preallocated observation array
4. If Python-only prototyping is needed, use `jsbsim` module first; later migrate hot path to C++ once behavior is validated.
5. For debug/visualization, keep a side channel that can run with `FLIGHTGEAR`/socket outputs only, and never let those channels drive the main training state loop.

### What to watch for

- Output type `SOCKET` in `FGOutput` docs is marked “DON’T USE THIS YET” for direct use, while `FLIGHTGEAR` remains a cleaner standard path for external visualization.[^jsbsim-output]
- If using socket input, property order and timestamp handling must be strict to avoid stale/misaligned control packets.[^jsbsim-input]
- You need a consistent canonical action-to-property mapping per aircraft model; JSBSim models are not standardized on control names across all aircraft files.

### Throughput expectations (explicitly inferred)

No official JSBSim step-rate benchmark is cited in JSBSim docs.  
However:

- PufferLib’s native C-style environments are optimized around contiguous buffers, vectorized execution, and OMP threading.[^puffer-docs]
- JSBSim can be run in batch mode without GUI, which removes one major bottleneck.

Therefore:

- **Direct C++ PufferEnv path**: potentially high throughput if model count/batching is tuned and property I/O is memory-efficient.
- **Python wrapper path (`jsbsim` pip module)**: easier to start, but likely below C++ path due interpreter overhead in tight loops.

Use profiling from your concrete model, not published generic claims, because aircraft model complexity and control update cost dominate.

---

## 🆚 Comparison Against Other Sims

| Simulator | Physics Engine | Rendering | Public API/SDK | RL Integration Fit | Why pick vs JSBSim |
| --- | --- | --- | --- | --- | --- |
| **JSBSim** | 6-DoF open FDM with XML model DSL | External (`FlightGear`/sockets/unreal) | C++, Python, CLI, socket I/O, plugins | High for C++/custom env, medium Python wrapper | Strongest for realistic reusable flight dynamics and headless throughput |
| **ClearView RC Flight Simulator** | Proprietary RC-focused simulator model pipeline | Proprietary Windows 3D stack | no publicly documented high-level API/SDK in source | Low direct, higher visual realism for manual practice | Better for real-human practice; weaker for transparent RL pipeline |
| **FlightGear** | Open-source framework using JSBSim and other engines | Built-in full simulator UI | Open-source project with extensibility; not a pure lightweight RL API by default | Medium if you bind to internal simulator interface only | Better for complete sim stack; heavier than JSBSim for RL throughput |
| **X-Plane** | Closed commercial flight engine | Full renderer + ecosystem | Plugin SDK with DLL/SDK pathways and datarefs | Medium with plugin layer | Better for validated commercial content, less convenient for minimal headless pipelines |
| **CRRCSim** | Open-source RC-focused flight sim | Own graphics stack (OpenGL/SDL references in project pages) | SourceForge-era project with limited modern API visibility | Low due age and uncertain API stability | Useful for historical RC feel, less robust for modern RL infra |

---

## 🧭 Candidate Scoring for RL Physics Workflows

- **Technical realism**: JSBSim B+ (strong, configurable, open FDM)
- **RL throughput compatibility**: JSBSim B (if C++ path; otherwise B- for Python-only)
- **Engineering friction**: JSBSim B- (steeper integration work than high-level Gym-only envs)
- **PufferLib attractiveness**: JSBSim A- (best candidate among open FDM stacks when you can invest integration time)

---

## 📘 Practical Notes

- Keep model selection simple while benchmarking (`C172`/simple fixed-wing scripts first).
- Use fixed integration step/time for better reproducibility across workers.
- Start in headless/batch mode; gate rendering behind a separate process or opt-in visual mode.
- If you need a known starting codebase, review open community wrappers (`gym-jsbsim`, `jsbsim-gym`, dogfight benchmarks) as behavior-reference examples before building your own env wrapper.

---

## 🔗 Related Notes

- [[PufferLib]]
- [[Reinforcement Learning]]
- [[Gymnasium]]
- [[PufferLib C99 Environment Authoring]]
- [[Vectorized Environments]]
- [[ClearView RC Flight Simulator]]
- [[Flight-Sim RL Contract]]
- [[PufferLib Flight Throughput Benchmark]]
- FlightGear (external ecosystem note) — https://www.flightgear.org/about/
- X-Plane (SDK-driven plugin model) — https://developer.x-plane.com/sdk/
- CRRCSim project metadata — https://sourceforge.net/projects/crrcsim/

---

## 📚 Source Notes

[^jsbsim-home]: JSBSim official docs and GitHub readme describe C++ core, 6-DoF model, XML aircraft definitions, external integration paths (Unreal/FlightGear), and console/batch operation. Sources: https://jsbsim-team.github.io/jsbsim/ and https://github.com/JSBSim-Team/jsbsim
[^jsbsim-readme]: JSBSim API docs (C++ and Python examples, plus binding list including Python/Matlab/Unreal) from https://github.com/JSBSim-Team/jsbsim
[^jsbsim-license]: JSBSim legal notice and LGPL 2.1 statement from https://github.com/JSBSim-Team/jsbsim
[^jsbsim-release]: Release and packaging details (Windows installer, Ubuntu packages, Python wheel distribution) from https://github.com/JSBSim-Team/jsbsim/releases and the JSBSim README installation section
[^jsbsim-fgf]: `FGFDMExec` workflow and methods from https://jsbsim-team.github.io/jsbsim/python/FGFDMExec.html
[^jsbsim-output]: Output system and `FLIGHTGEAR`/`SOCKET` output types from https://jsbsim-team.github.io/jsbsim/classJSBSim_1_1FGOutput.html
[^jsbsim-input]: Input socket types (`FGInputSocket`, `FGUDPInputSocket`) and their input-read behavior from https://jsbsim-team.github.io/jsbsim/classJSBSim_1_1FGInputSocket.html and https://jsbsim-team.github.io/jsbsim/classJSBSim_1_1FGUDPInputSocket.html
[^jsbsim-source]: `JSBSim` CLI usage and options from JSBSim C++ source documentation: https://jsbsim-team.github.io/jsbsim/JSBSim_8cpp_source.html
[^jsbsim-manual-overview]: Manual overview states JSBSim can run integrated in larger frameworks or faster than real time in batch mode: https://jsbsim-team.github.io/jsbsim-reference-manual/user/overview/
[^puffer-docs]: PufferLib environment architecture and vectorization model from https://puffer.ai/docs.html and https://puffer.ai/blog.html
[^flightgear-about]: FlightGear mission/development and GPL openness from https://www.flightgear.org/about/ and https://wiki.flightgear.org/FlightGear
[^xplane-sdk]: X-Plane plugin/SDK model and integration path from https://developer.x-plane.com/sdk/
[^crrcsim]: CRRCSim project metadata and status from SourceForge (open-source RC sim, OpenGL/SDL interface context): https://sourceforge.net/projects/crrcsim/ and https://sourceforge.net/projects/crrcsim/files/crrcsim/
