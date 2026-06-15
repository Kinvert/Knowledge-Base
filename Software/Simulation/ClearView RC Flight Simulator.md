---
title: ClearView RC Flight Simulator
aliases: [ClearView, ClearView RC, ClearView RC Flight Simulator]
tags: [simulation, rc, flight-simulator, reinforcement-learning, pufferlib, aerodynamics]
---

# ClearView RC Flight Simulator

**ClearView RC Flight Simulator** is a proprietary Windows RC aircraft simulator for planes and helicopters. It is useful as a human-practice simulator and as a case study in how RC flight sims represent models, controls, collision geometry, and tuning parameters, but it is a weak direct target for high-throughput [[PufferLib]] training unless a hidden or unsupported automation path is discovered.

The core research result: ClearView exposes a rich model-authoring format, but I did not find a documented headless mode, step API, state API, telemetry API, plugin SDK, or simulator-speed control suitable for batched RL rollouts.

---

## Research Status

| Area | Current confidence | Evidence |
|---|---:|---|
| Product scope | High | Official homepage lists Windows support, RC planes/helicopters, collision, 3D physics, autorotation, high-alpha simulation, imported models/fields, and controller support.[^cv-home][^cv-help][^cv-controllers] |
| Model format | High | Official model docs describe `params.txt`, `fmodel.ac`, `colbody.ac`, `body.ac`, AC3D files, lift/thrust objects, collision geometry, and version-6 physics.[^cv-models][^cv-model-intro][^cv-plane1][^cv-plane3] |
| Physics engine internals | Medium | Docs describe ClearView's own lift/thrust/servo/component model, but not source code or numerical integrator details.[^cv-model-intro] |
| Rendering API | Low | Docs describe rendered geometry in AC3D `body.ac`, but I found no official DirectX/OpenGL/Vulkan statement for the runtime renderer. Do not assume an API from "DirectX FMS model import."[^cv-home][^cv-model-intro] |
| Implementation language | Low-medium | Old model tutorial instructions mention replacing `clearview.jar` for ClearView 4.98-era code; current runtime language is not proven from public docs alone.[^cv-plane1] |
| RL integration | High negative | Official docs expose model/scenery/controller workflows, but no public RL-style control/state interface was found.[^cv-help][^cv-controllers][^cv-models] |
| SPS expectation | Unknown for ClearView | Needs benchmark. Without headless/vectorized stepping, expect GUI/real-time limits rather than PufferLib-style millions of SPS.[^puffer-docs] |

---

## Product Overview

ClearView is positioned as a professional-quality RC simulator for radio-controlled planes and helicopters. The official homepage says the Windows version includes plane and helicopter models, flying fields, full collision support, 3D physics, autorotation, high-alpha flight simulation, flight recording/replay, autopilot, slow-motion learning, imported panoramas, imported models, servo experimentation, and gyro simulation.[^cv-home]

The official help and controller pages show that ClearView is designed around normal interactive use:

- Windows PC execution.
- Keyboard, Windows gamepad/joystick, or USB RC transmitter adapter input.[^cv-controllers]
- Controller setup through the simulator UI.[^cv-help]
- Troubleshooting through Windows video drivers, `dxdiag`, USB-device isolation, and a local `log.txt` file.[^cv-help]

For RL, this matters because the product surface is an interactive simulator, not an exposed simulation library.

---

## File and Model Architecture

ClearView model authoring is unusually relevant for RL because it exposes how a desktop RC sim decomposes aircraft into physics, collision, visual, and component layers.

Official model docs describe each model as a folder with a text parameter file and supporting data folders. The key files are:

| File | Role | RL relevance |
|---|---|---|
| `params.txt` | Model parameters, component wiring, mass, gravity, gear settings, breakable parts, servos, motors, receivers, gyros, mixers, governors | Best source for extracting state/action semantics and control-chain assumptions |
| `data/fmodel.ac` | AC3D physics geometry | Describes lift surfaces, thrust objects, and drag objects that could be translated into a custom C/PufferLib model |
| `data/colbody.ac` | Simplified collision geometry | Useful for crash/contact approximations and reset/termination logic |
| `data/body.ac` or `model.ac` | Rendered visual geometry | Useful for visual fidelity, but less useful for high-SPS RL |
| `wavdata/` | Sound files | Not relevant for most RL tasks |

The model docs explicitly separate flight simulation, world interaction, and visualization. They also explain that simplified collision geometry is used for performance because visual meshes can contain thousands of surfaces.[^cv-model-intro][^cv-plane1]

This separation is the most reusable idea for a PufferLib harness: parse physics geometry and parameters, then build a separate deterministic training environment instead of trying to automate ClearView's GUI.

---

## Physics Model

ClearView's documented model system is object-based rather than a generic rigid-body CAD import.

The version-6 model docs describe two primary physical object families:

- **Thrust objects**: propellers, tail rotors, and main rotors.
- **Lift/drag objects**: rectangular lift surfaces and drag spheres.

Plane models are built from lift surfaces for wings, body projections, elevator, rudder, ailerons, and a propeller object. Helicopter models use main rotor, tail rotor, drag body, and optional tail surfaces.[^cv-model-intro]

The `params.txt` system then wires model components together in a way that intentionally resembles RC hardware:

- transmitters
- receivers
- servos
- motors
- speed controllers
- mixers
- governors
- gyros
- propellers / rotors
- lift surfaces
- drag spheres

Example parameters documented in the official model files include:

- `centerOfGravity`
- `dimensions`
- `gravity`
- `modelMass`
- gear bounce/friction/spring parameters
- `airfoilDataName`
- `liftCoef`
- `dragCoef`
- `inducedDragCoef`
- `propWashCoef`
- `powerCurveWatts`
- servo axis and movement mapping
- breakable-part definitions

The official plane tutorial shows built-in airfoil names such as `NACA-0009`, `NACA-0012`, and `NACA-0015`, plus custom airfoil file names.[^cv-plane1]

Important limitation: I found no public source code, integrator details, aerodynamic equations, timestep documentation, or deterministic replay guarantees for ClearView itself. The physics model is inspectable at the model-parameter level, not at the solver level.

---

## Rendering and Assets

ClearView's public documentation supports the following rendering-related facts:

- Rendered model geometry is stored separately from physics and collision geometry.[^cv-model-intro]
- Visual model files use AC3D format and named objects/groups for moving parts.[^cv-model-intro][^cv-plane3]
- Moving control surfaces and spinning propellers are linked by naming conventions between physics objects and render objects.[^cv-plane3]
- The official site mentions panorama scenery import and imported models.[^cv-home]
- The troubleshooting docs point users to Windows video-driver updates when encountering black screens or startup failures.[^cv-help]

What is not proven:

- The runtime rendering API.
- Whether the renderer can run offscreen.
- Whether rendering can be disabled.
- Whether the simulation can run faster than wall-clock time.

For RL, the absence of a documented offscreen/headless mode is a major blocker.

---

## Inputs and Controls

ClearView supports keyboard, Windows gamepad/joystick input, and RC transmitter USB adapters.[^cv-controllers] The help docs emphasize mapping channels through the UI and describe heli control regimes such as collective, cyclic, rudder/yaw, idle-up, and throttle hold.[^cv-help]

This gives three possible automation levels:

| Input route | Feasibility | RL quality |
|---|---:|---|
| Keyboard automation | Easy | Poor for continuous control; coarse and OS-timing dependent |
| Virtual joystick / gamepad | Plausible | Better action space, still GUI-bound |
| RC transmitter adapter emulation | Plausible but hardware-specific | Good for human-like channels, still no state API |

Even if input automation works, observations and rewards remain the hard part. Without state/telemetry, a ClearView RL agent would likely need screen capture plus computer-vision rewards, which is sample-inefficient and difficult to debug compared with [[JSBSim]], [[Gymnasium]], or a native PufferLib C environment.

---

## Public API and Automation Surface

I found no official ClearView equivalent of:

- plugin SDK
- Python API
- C/C++ simulation library
- socket API
- telemetry stream
- headless executable mode
- deterministic `reset()` / `step(action)` interface
- batch/vectorized environment API

Documented extension points are instead:

- model folders under `ClearViewRC\clearview\models` or older `SVKSystems\ClearView\models` paths[^cv-models][^cv-model-intro]
- `params.txt` physics/control tuning[^cv-model-intro]
- AC3D files for physics, collision, and rendered geometry[^cv-model-intro]
- scenery import[^cv-home]
- controller mapping through the UI[^cv-help][^cv-controllers]
- flight recorder for interactive replay workflows[^cv-home]

That makes ClearView more valuable as a reverse-engineering/design reference than as a direct high-performance RL backend.

---

## PufferLib Fit

PufferLib's fastest path expects many environment instances, contiguous observation/action/reward/terminal buffers, and cheap stepping. Its docs describe Ocean environments in C, contiguous buffers across thousands of instances, OMP vectorization, CUDA-side training, and headline training throughput in the millions of steps per second.[^puffer-docs]

ClearView conflicts with that shape:

- It appears to be a single interactive GUI simulator.
- It has no documented batch stepping.
- It has no documented direct state access.
- It has no documented direct reward/task API.
- It appears to require Windows graphics/input plumbing.
- It is proprietary, so solver internals cannot be inspected or modified.

### Practical SPS Estimate

Do not claim a ClearView SPS number without measurement.

A black-box GUI harness would likely be bounded by the render/input loop and any real-time limiter. If it can only run real-time, the control rate is likely in the tens to low hundreds of steps per second per instance. This is an engineering inference from the lack of a headless step API, not a sourced ClearView benchmark.

By contrast, a PufferLib-native C environment can be built specifically for thousands of instances and high-throughput PPO training.[^puffer-docs]

### Better PufferLib Use

The best route is probably:

1. Use ClearView docs and model files to understand RC model decomposition.
2. Parse `params.txt` and `fmodel.ac` from a simple aircraft.
3. Rebuild a simplified fixed-wing/heli dynamics model in C for [[PufferLib C99 Environment Authoring]].
4. Use [[JSBSim]] or another open FDM as the higher-fidelity comparison target.
5. Keep ClearView for human practice and qualitative behavior comparison.

For an apples-to-apples performance decision, benchmark any ClearView-derived prototype against [[PufferLib Flight Throughput Benchmark]] before committing to a production RL route.

---

## Harness Options

| Harness | How it would work | Pros | Cons | Verdict |
|---|---|---|---|---|
| GUI black-box harness | Drive virtual joystick, capture screen, infer reward from pixels | Minimal simulator modification | Slow, brittle, no privileged state, hard resets, poor reward observability | Not first choice |
| Flight-recorder mining | Use recorded flights as demonstrations if file format is readable | Could support imitation learning | Format/API not yet verified; may not include state | Research-only |
| Model-file translator | Parse `params.txt` + `fmodel.ac` into a custom C/JSBSim environment | Best PufferLib fit; can be vectorized and tested | Requires implementing/approximating physics | Recommended |
| Windows memory/API instrumentation | Inspect process memory or private calls | Could expose state | Fragile, version-specific, questionable maintenance value | Avoid unless explicitly needed |
| AbsoluteSim browser harness | Use the related browser sim as an automation target | Browser tools may be easier than Win32 automation; site states 100 Hz rigid-body physics | Still proprietary; public API unknown; subscription/login may complicate benchmarking | Separate follow-up note |

---

## Candidate Score for RL Flight Physics

| Criterion | ClearView assessment |
|---|---|
| Human RC practice | Good. It supports RC-style controllers and many aircraft/field workflows.[^cv-home][^cv-controllers] |
| Physics inspectability | Medium. Model parameters and geometry are exposed, solver source is not.[^cv-model-intro] |
| API access | Poor. No public step/state API found. |
| Headless operation | Poor/unknown. No documented headless mode found. |
| Vectorization | Poor. No batch/vectorized execution path found. |
| Reward/state access | Poor. Likely screen-only unless files or memory expose useful data. |
| PufferLib integration | Weak direct fit; good indirect inspiration for a custom C environment. |
| Sim2Real relevance | Moderate for RC control feel; unknown for quantitatively accurate aero without validation. |
| Best use in this project | Study model structure, collect aircraft parameter ideas, and compare qualitative behavior against an open FDM. |

Overall candidate grade for direct PufferLib training: **D**.

Overall candidate grade as a reference for building an RC-flight RL environment: **B-**.

---

## Comparison Chart

This table is intentionally conservative. Later notes should replace placeholder cells with sourced details.

| Simulator | Source / openness | Physics access | API / automation | PufferLib fit | Research note |
|---|---|---|---|---|---|
| ClearView | Proprietary Windows RC sim | Model files exposed; solver closed | No public API found | Weak direct, useful indirect | Good for studying RC model decomposition |
| [[JSBSim]] | Open-source C++ FDM library | High: XML aircraft models, properties, batch/standalone modes | Strong: library, standalone, logging, sockets | Strong | Best current candidate for realistic non-visual flight dynamics[^jsbsim] |
| [[FlightGear]] | Open-source flight simulator | High via open project and FDM options | Stronger than ClearView, heavier than JSBSim | Medium | Good for full simulator integration and research framework[^flightgear] |
| [[X-Plane]] | Proprietary commercial sim | Medium: high-quality sim, closed core | Strong plugin SDK/datarefs, but not open core | Medium-low for high SPS | Good for plugin/dataref experiments, less ideal for batched training[^xplane-sdk] |
| [[CRRCSim]] | Open-source RC model-airplane sim | Potentially high, needs source review | Unknown until source/docs review | Potentially medium | More promising than ClearView for RC-specific open code[^crrcsim] |
| Absolute RC / AbsoluteSim | Proprietary browser/mobile RC sim | Site claims 100 Hz rigid-body physics | Browser app; public sim API unknown | Unknown | Worth separate note because browser automation may be easier[^absolute] |

---

## What To Verify Next

Before spending engineering time on a ClearView harness:

1. Install in an isolated Windows VM or Wine prefix and inspect installed runtime files.
2. Confirm whether current builds still include `clearview.jar` or moved to a native runtime.
3. Search installed docs/configs for command-line flags, telemetry, replay formats, simulation speed, and graphics-disable options.
4. Inspect `startupParams.txt`, model `params.txt`, and flight recorder files.
5. Test whether the sim can run faster than real time.
6. Test deterministic reset behavior with a fixed model, field, wind, and input sequence.
7. Measure actual control-loop throughput with a virtual joystick and screen capture.
8. Try parsing one simple model into a standalone C prototype.

If these checks do not reveal a step/state API or replay telemetry, move on to [[JSBSim]] or an open RC simulator for RL.

---

## Related Notes

- [[PufferLib]]
- [[PufferLib C99 Environment Authoring]]
- [[Gymnasium Environment Authoring for Robotics]]
- [[Vectorized Environments]]
- [[Reinforcement Learning]]
- [[Sim2Real]]
- [[JSBSim]]
- [[FlightGear]]
- [[X-Plane]]
- [[CRRCSim]]
- [[OpenGL]]
- [[DirectX]]
- [[PID Controller]]
- [[XFOIL]]
- [[Flight-Sim RL Contract]]
- [[PufferLib Flight Throughput Benchmark]]

---

## External Resources

- ClearView official homepage: https://www.rcflightsim.com/
- ClearView help and troubleshooting: https://www.rcflightsim.com/help.html
- ClearView controller page: https://www.rcflightsim.com/controllers.html
- ClearView model page: https://www.rcflightsim.com/models.html
- ClearView flight manual PDF: https://www.rcflightsim.com/cvdocs/CVFlightManual.pdf
- ClearView model introduction DOCX: https://time2rc.s3.amazonaws.com/CVManuals/Creating+ClearView+Models_.docx
- ClearView plane model docs:
  - https://time2rc.s3.amazonaws.com/CVManuals/Create+plane+model+1.docx
  - https://time2rc.s3.amazonaws.com/CVManuals/Create+plane+model+2.docx
  - https://time2rc.s3.amazonaws.com/CVManuals/Create+plane+model+3.docx
- AbsoluteSim official page: https://absolutesim.com/
- PufferLib docs: https://puffer.ai/docs.html
- JSBSim docs: https://jsbsim-team.github.io/jsbsim/
- FlightGear about page: https://www.flightgear.org/about/
- X-Plane SDK docs: https://developer.x-plane.com/sdk/
- CRRCSim SourceForge page: https://sourceforge.net/projects/crrcsim/

---

## Source Footnotes

[^cv-home]: ClearView official homepage, https://www.rcflightsim.com/, inspected 2026-06-13. It lists Windows support, bundled models/fields, collision, 3D physics, autorotation, high-alpha simulation, model/scenery import, autopilot, flight recording, and controller-oriented features.
[^cv-help]: ClearView help and troubleshooting page, https://www.rcflightsim.com/help.html, inspected 2026-06-13. It documents Windows troubleshooting, controller setup, RC helicopter controls, idle-up, throttle hold, and autorotation guidance.
[^cv-controllers]: ClearView controller page, https://www.rcflightsim.com/controllers.html, inspected 2026-06-13. It lists keyboard, Windows gamepad/joystick, and USB RC transmitter adapter input routes.
[^cv-models]: ClearView models page, https://www.rcflightsim.com/models.html, inspected 2026-06-13. It links official model conversion manuals and version-6 flight-physics documents.
[^cv-model-intro]: ClearView "Creating ClearView Models" DOCX, https://time2rc.s3.amazonaws.com/CVManuals/Creating+ClearView+Models_.docx, inspected 2026-06-13. It describes model folders, `params.txt`, `fmodel.ac`, `colbody.ac`, render geometry, lift/thrust objects, component sections, and version-6 unified physics.
[^cv-plane1]: ClearView "Create plane model 1" DOCX, https://time2rc.s3.amazonaws.com/CVManuals/Create+plane+model+1.docx, inspected 2026-06-13. It describes plane physics setup, collision simplification, model parameters, airfoil names, motor/servo/receiver/transmitter wiring, and older `clearview.jar` instructions for version 4.98-era tutorials.
[^cv-plane3]: ClearView "Create plane model 3" DOCX, https://time2rc.s3.amazonaws.com/CVManuals/Create+plane+model+3.docx, inspected 2026-06-13. It describes visual model construction, named moving parts, propeller objects, and how render objects follow physics object names.
[^absolute]: AbsoluteSim official page, https://absolutesim.com/, inspected 2026-06-13. It describes the browser version, supported devices, controller routes, aircraft/landscape counts, and claims 100 Hz rigid-body physics.
[^puffer-docs]: PufferLib docs, https://puffer.ai/docs.html, inspected 2026-06-13. It describes PufferLib throughput goals, C Ocean environment authoring, contiguous buffers, vectorization, CUDA training, and reported millions-of-SPS training paths.
[^jsbsim]: JSBSim Doxygen docs, https://jsbsim-team.github.io/jsbsim/, inspected 2026-06-13. It describes JSBSim as an open-source C++ flight dynamics model with XML aircraft specs, standalone/batch modes, properties, logging, and socket output.
[^flightgear]: FlightGear about page, https://www.flightgear.org/about/, inspected 2026-06-13. It describes FlightGear as an open-source simulator for Windows, macOS, and Linux with source code under the GPL and research/academic extensibility goals.
[^xplane-sdk]: X-Plane SDK docs, https://developer.x-plane.com/sdk/, inspected 2026-06-13. It describes the plugin SDK for writing additions that work inside X-Plane without modifying the simulator source.
[^crrcsim]: CRRCSim SourceForge page, https://sourceforge.net/projects/crrcsim/, inspected 2026-06-13. It describes CRRCSim as an open-source model-airplane flight simulation program for Linux with ports to other platforms.
