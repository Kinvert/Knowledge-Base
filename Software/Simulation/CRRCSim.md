---
title: CRRCSim
aliases:
  - CRRCSim
  - CRRCSim RC Flight Simulator
tags:
  - simulation
  - flight-simulation
  - rc
  - reinforcement-learning
  - pufferlib
  - cpp
  - sdl
---

# CRRCSim

**CRRCSim** is an open-source RC model-airplane simulator with a legacy desktop architecture.  
For RL, it is best treated as a **reference implementation and model corpus**, not a turnkey RL environment.

---

## 🧭 Research Status

| Area | Confidence | Evidence |
|---|---:|---|
| Project scope | High | SourceForge summary positions it as open-source RC sim, GPLv2, Beta, cross-platform. | [SourceForge](https://sourceforge.net/projects/crrcsim/) |
| Language / build stack | High | Debian source/build files indicate C/C++ with autotools-era wiring and legacy C++ dependencies. | [Debian `Makefile.am`](https://sources.debian.org/src/crrcsim/0.9.12-6.1/Makefile.am), [Debian source package](https://packages.debian.org/source/stable/crrcsim) |
| Runtime/control interface | Medium | Public manual describes CLI flags and input modes, not native RL step/reset. | [crrcsim manpage](https://manpages.debian.org/testing/crrcsim/crrcsim.1.en.html) |
| Model artifact usability | High | Aircraft models are XML with aero/power/mass/geometry and launch/render sections. | [superzagi.xml](https://sources.debian.org/src/crrcsim/0.9.12-6.1/models/superzagi.xml/), [heli.xml](https://sources.debian.org/src/crrcsim/0.9.12-6.1/models/heli.xml/) |
| PufferLib direct fit | Low | No public API contract for env-style stepping was found. | [manpage](https://manpages.debian.org/testing/crrcsim/crrcsim.1.en.html) |
| Maintenance recency | Low | Public release set peaks at 0.9.13; long gaps in visible activity. | [SourceForge files](https://sourceforge.net/projects/crrcsim/files/crrcsim/), [support request](https://sourceforge.net/p/crrcsim/support-requests/5/) |

---

## What CRRCSim Is (and Isn’t)

What is clear:

- It is an interactive simulator binary with RC-specific workflows and classic desktop rendering.
- It exposes model/scenery/input files and runtime flags.
- It does **not** expose a documented headless, vectorized, RL-step API.

What is useful:

- The source and model formats are inspectable.
- You can treat the vehicle config corpus as a seed for your own RL engine.

---

## Language, Rendering, and Model Files

### Language and build surface

The code distribution is C/C++ heavy and has historical autotools metadata, with dependencies from the graphics/audio/input stack visible in package metadata and build files (SDL/PLIB/GLU/PortAudio/JPEG).[^debian-makefile]

This creates a plausible path for source-level integration:

- You can compile and link against it.
- You can read/parse model files for your own deterministic loop.
- You should not expect a simple high-level Python API call path.

### Rendering assumptions

The project summary and source metadata point to OpenGL-era rendering with PLIB/SDL style plumbing, not a modern offscreen service-oriented renderer.[^sf-project]

### Model files as an RL input layer

`models/*.xml` entries are structured and concrete:

- `aero` and `power` sections  
- `mass_inertia`, `wheels`  
- `graphics`, `launch`  
- helicopter sections for rotor/engine/control composition

This schema is valuable if you are building a reproducible wrapper layer around model definitions.[^superzagi-xml][^heli-xml]

---

## Public Interface (Automation Reality)

The `crrcsim` CLI manual documents:

- Display and runtime flags (`-f`, `-x`, `-y`, `-u`, `-g`, `-l`, etc.)
- Input mode selection (`-i`): `KEYBOARD`, `MOUSE`, `JOYSTICK`, `RCTRAN`, `SERPIC`, `SERIAL2`, `PARALLEL`, `AUDIO`, `MNAV`, `ZHENHUA`
- Button mapping (`-b`)
- Config file loading defaults (`~/.crrcsim/crrcsim.xml`, override with `-g`)

This is useful for human testing and scripted launch control, but not enough as a PufferLib-style environment contract.[^manpage]

---

## PufferLib Integration Strategy

### Recommended path (practical)

Build your own RL core from CRRCSim’s model format, not a direct environment wrapper:

1. Parse one or two CRRCSim model XMLs into a canonical schema.
2. Implement a C++ step/reset kernel with fixed time-step update rules.
3. Expose contiguous `obs`, `act`, `reward`, `done` buffers for a PufferLib C/C++ adapter.[^puffer]
4. Keep CRRCSim itself as validation/reference, not runtime core.

### Non-recommended path

- Driving the simulator GUI from input events and deriving observations from pixels/files can work for quick experiments, but it is slow and brittle.

### Why this is still worth doing

- CRRCSim gives historically relevant RC-model data organization.
- Reusing the model schema can accelerate early training tasks where writing every airframe from scratch is too expensive.

---

## SPS / Throughput Expectations

No public, authoritative “SPS” figure exists for CRRCSim itself in the reviewed documentation.

| Execution path | SPS expectation | Key reason |
|---|---:|---|
| GUI black-box harness | Low | Render/input loops dominate; hard to vectorize |
| Source-level C++ adapter | Medium-High potential | Depends on your own fixed-step batched scheduler |
| Headless flag-only mode | Medium (if render can be reduced) | Still no documented vectorized state interface |

Treat this table as an inference to be validated after implementation.

---

## Comparison Chart

| Simulator | Physics openness | API/automation | Rendering burden | PufferLib direct fit | Primary role vs RL |
|---|---|---|---|---|---|
| CRRCSim | Open source + XML models | CLI/input, no documented env API | Legacy OpenGL/PLIB/SDL path | Low | Aircraft model corpus and reference dynamics behavior |
| [[JSBSim]] | High (open C++ FDM + XML) | C++/Python/CLI explicit interfaces | Externalized renderer | High | Primary candidate for RL flight physics |
| [[FlightGear]] | High (full framework + external FDMs) | CLI/property/generic protocol | Full visual simulator stack | Medium | Best for visual/full-system studies |
| [[ClearView RC Flight Simulator]] | Proprietary | No public API discovered | Proprietary interactive | Very low | Human practice/qualitative comparison |
| X-Plane | Closed proprietary | Plugin/SDK datarefs | Full renderer | Medium-Low | Realistic benchmark with licensing overhead |
| Absolute RC / AbsoluteSim | Proprietary | Browser/API unclear | Browser/renderer | Unknown | Candidate only if external automation is validated |

---

## Candidate Score (Current Stack Decision)

- **Physics transparency:** B
- **RL integration readiness:** D
- **Scalability with PufferLib:** D
- **Research value as template/reference:** B+

Interpretation: good for model-extraction studies, weak for direct high-SPS RL without redesign.

---

## Related Notes

- [[FlightGear]]
- [[JSBSim]]
- [[ClearView RC Flight Simulator]]
- [[PufferLib]]
- [[PufferLib C99 Environment Authoring]]
- [[Reinforcement Learning]]
- [[Flight-Sim RL Contract]]
- [[PufferLib Flight Throughput Benchmark]]

---

## Sources

[^sf-project]: Source metadata and project summary (GPLv2, platform targets, beta status, renderer/tool hints): https://sourceforge.net/projects/crrcsim/
[^sf-files]: SourceForge file archive showing release versions (including 0.9.13): https://sourceforge.net/projects/crrcsim/files/crrcsim/
[^sf-code]: Source tree listing with code/tags and repository metadata: https://sourceforge.net/p/crrcsim/code/ci/default/tree/
[^sf-support5]: SourceForge support request #5 with compile/toolchain discussion: https://sourceforge.net/p/crrcsim/support-requests/5/
[^manpage]: Official CLI and runtime options from Debian manpage (`crrcsim(1)`): https://manpages.debian.org/testing/crrcsim/crrcsim.1.en.html
[^debian-package]: Debian source package metadata for package split and version lineage: https://packages.debian.org/source/stable/crrcsim
[^debian-makefile]: Debian source `Makefile.am` showing C++ objects and dependency/link flags: https://sources.debian.org/src/crrcsim/0.9.12-6.1/Makefile.am
[^superzagi-xml]: Sample aircraft model XML with physics/graphics/wheels/power/launch blocks: https://sources.debian.org/src/crrcsim/0.9.12-6.1/models/superzagi.xml/
[^heli-xml]: Sample helicopter model XML with rotor and controls-related sections: https://sources.debian.org/src/crrcsim/0.9.12-6.1/models/heli.xml/
[^puffer]: PufferLib docs on contiguous memory and high-throughput design assumptions: https://puffer.ai/docs.html
