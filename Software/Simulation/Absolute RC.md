---
title: Absolute RC
aliases:
  - Absolute RC
  - Absolute RC Simulator
  - AbsoluteSim
tags:
  - simulation
  - reinforcement-learning
  - flight-simulation
  - pufferlib
  - web
  - rc
---

# Absolute RC

`Absolute RC` (formerly marketed as Absolute RC Simulator / AbsoluteSim) is a browser-first RC simulator with very broad platform availability and a stated 100 Hz rigid-body model.  
For RL, its strengths are realistic RC training workflows and easy access; its limits are a likely absence of a documented programmatic control/state API and lack of publicly documented physics internals.

---

## Research Status

| Question | What we know | Confidence |
|---|---|---:|
| Delivery model | Browser app, no required install; launch directly in browser | High |
| Physics claim | Uses "full rigid-body physics at 100Hz" and same professional physics across plans | High |
| Physics transparency | No physics source code or integrator documentation surfaced in public docs | Medium |
| Language/runtime surface | Public pages show web delivery and login workflow; no published SDK/remote API docs | Medium |
| Input surface | Gamepad and RC transmitter support; USB devices in-browser through OS device recognition | High |
| PufferLib suitability | Strong for manual/behavioral studies, weak for headless vectorized training unless hidden bridge found | Low |

---

## Product shape and rendering assumptions

Absolute RC is positioned as a practical replacement for desktop RC sims:

- no install required
- desktop/mobile/tablet support
- 450+ models and landscapes (site claims differ by plan labels in marketing lists)
- login via email magic link and session cookie

This implies:

- rendering path and physics stack are managed inside the provided app runtime,
- step pacing is tied to the simulator executable path and not exposed as a standalone FDM service/API.

The simulator also advertises support for "all aircraft have full physics engine" per plan and uses the same "professional physics engine" in every tier.

---

## Physics model visibility

Absolute RC has one major transparency limitation for RL:

- It exposes flight-feel claims (rigid-body, real aerodynamics, ground handling, crash dynamics) at a high level,
- but does not publish a model DSL or solver API that would be directly mapped to RL state transition code.

Where this helps RL research is that it gives a **reference physics behavior** target and a real-world-facing control stack. Where it hurts RL throughput is that it does not advertise a deterministic reset/action hook.

---

## Controls and input layer

From official help:

- Any gamepad/keyboard/USB transmitter that OS exposes as gamepad is accepted in browser.
- `Add to Home Screen` flow allows pseudo-native launch behavior on iOS/Android, but still browser app behavior under the hood.
- The gamepad layer requires initial activity to be discovered by browsers (common Web Gamepad API pattern), and has transmitter setup guidance.

This matters because:

- Input can be scripted at OS/browser level (keyboard/gamepad automation),
- but controller state is not the same as a stable simulation API with semantic action names.

---

## API / automation surface (important for PufferLib)

Claims and gaps from official web surface:

- Login, model browsing, controls, and gameplay pages are documented.
- There is no published public section for:
  - API tokening
  - WebSocket/REST command surface
  - step/reset methods
  - telemetry stream endpoints

Given these pages, a direct PufferLib `step(action)` bridge is not directly evidenced. Any RL integration path would be an external automation path (input/event synthesis plus CV/telemetry extraction), not a native environment API.

---

## PufferLib integration strategy

### High-value route (pragmatic)

1. Use `Absolute RC` as **behavioural reference** for RC control feel and scenario design.
2. Build a standalone environment using custom dynamics from extracted aircraft behaviour expectations.
3. If using a direct bridge, keep it as a **visual-only baseline** for reward supervision and sanity checks.

### Black-box options to compare

| Harness pattern | What it gives you | Main cost |
|---|---|---|
| Browser input automation | Cheapest to start | high latency, brittle, no authoritative state |
| Screen/vision reward extraction | Fast to visualize policy impact | hard reward definition, camera-dependent |
| Flight log scraping via local persistence (if accessible) | Cleaner than raw pixels if logs available | format risk, undocumented lifecycle |
| Private instrumentation/in-memory hook | Could unlock high throughput | high maintenance, fragile, legal/compliance risk |

### Better short path for RL quality

Build direct PufferLib training logic with an open core (JSBSim or FlightGear external dynamics path) and use `Absolute RC` as a transfer target for policy behavior parity checks.

---

## SPS / throughput expectations

No official benchmark exists for RL-style stepping.
Without a headless/step API and without a published sim speed control interface, expected throughput is likely constrained by:

- browser input loop latency,
- render loop timing,
- input event and capture overhead.

For now, any throughput estimate is **placeholder** until a hard harness exists.

---

## Candidate score for RL workflows

| Dimension | Assessment |
|---|---|
| Realism realism for RC control habits | B |
| Open researchability | C- |
| Direct API for RL | D |
| Expected PufferLib fit | D |
| Best use pattern | Human practice + baseline policy comparison, not fast vector RL training |

---

## Comparison with other RL candidates

| Simulator | Physics openness | API/automation | PufferLib integration posture | RL role |
|---|---|---|---|---|
| [[JSBSim]] | High (open C++/XML) | explicit APIs + headless modes | high | primary training core |
| [[FlightGear]] | High (open framework) | property/protocol bridges | medium | visual + full stack transfer |
| [[X-Plane]] | Closed, commercial SDK surface | plugin + Web API | medium | high-fidelity transfer target |
| [[ClearView RC Flight Simulator]] | Closed, model params exposed | no public RL step API | low | control semantics reference |
| **Absolute RC** | Proprietary browser runtime | no public automation API surfaced | low | benchmark realism and task design |
| [[Aerofly RC10]] | Closed desktop, documented controls | no known public telemetry API | low-medium | high-end visual & RC training target |
| [[neXt RC]] | Closed desktop legacy/Unity | no public programmatic RL API | low | older reference for controls and modes |

---

## Related notes

- [[PufferLib Flight Sim Integration Gaps]]
- [[PufferLib Flight Throughput Benchmark]]
- [[Flight-Sim RL Contract]]
- [[FlightGear]]
- [[X-Plane]]
- [[JSBSim]]
- [[ClearView RC Flight Simulator]]
- [[CRRCSim]]
- [[Absolute RC]]

---

## External resources

- https://absolutesim.com/
- https://absolutesim.com/help/gamepad.php
- https://absolutesim.com/models/
- https://absolutesim.com/features.html
- https://absolutesim.com/faq.php
- https://rcflightsim.com/help.php

---

## Source notes

[^abs-home]: Absolute RC homepage, https://absolutesim.com/, inspected 2026-06-13. It states browser play, no download requirement, model/landscape scale, and "Full rigid-body physics at 100Hz."
[^abs-gamepad]: Absolute RC controller page, https://absolutesim.com/help/gamepad.php, inspected 2026-06-13. It documents gamepad/gamepad/button discovery behavior and device setup guidance.
[^abs-features]: Absolute RC features/plan page and FAQ details, https://absolutesim.com/features.html and https://absolutesim.com/faq.php, inspected 2026-06-13. They describe common physics engine behavior across plans.
[^abs-related]: Absolute RC homepage footer note, https://absolutesim.com/, inspected 2026-06-13. It links back to ClearView RC as the original desktop lineage.
[^abs-help]: Absolute RC support entry from earlier RC family portal, https://rcflightsim.com/help.php, inspected 2026-06-13. It exposes only help/transaction flow and activation links, not a public API.
[^jsbsim]: JSBSim note for integration context, https://github.com/JSBSim-Team/jsbsim and https://jsbsim-team.github.io/jsbsim/, inspected 2026-06-13.
