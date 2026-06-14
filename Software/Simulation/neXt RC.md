---
title: neXt RC
aliases:
  - neXt RC
  - neXt
  - neXt RC Simulator
tags:
  - simulation
  - reinforcement-learning
  - flight-simulation
  - rc
  - pufferlib
---

# neXt RC

`neXt RC` is a closed RC flight simulator family from CGM. It appears in two related families (desktop + newer web-connected variants) and advertises broad aircraft, scenarios, and multiplayer modes.  
For RL, it has useful legacy model and multiplayer semantics, but there is still no obvious first-class programmatic RL API in public docs.

---

## Research Status

| Question | What we can confirm | Confidence |
|---|---|---:|
| Distribution | Commercial desktop sim with dedicated client builds | High |
| Physics implementation style | Closed runtime; Unity appears in historical release notes | Medium |
| Rendering | Real-time interactive 3D rendering with offline/online flight modes | Medium |
| Automation API | No published official SDK/API docs in public product pages | Medium |
| PufferLib direct fit | Low without discovered bridge | Low |

---

## Product structure and likely engine profile

Public pages position neXt as a full RC simulator with:

- many RC model categories,
- training/sim scenarios,
- online / multiplayer session features,
- controller-based interaction.

Release-history hints (Unity 2019.4.29f1 and later engine references on some pages) indicate a modern engine build path internally, but this is an implementation detail, not a guaranteed extension API.

For RL, this means:

- user-loop behavior is rich enough for behavior comparison;
- runtime is not documented as a headless batch state machine.

---

## Controls and interaction

Controls are primarily:

- keyboard/joystick/RC tx workflows,
- in-sim configuration,
- menu-driven scenario selection and mode switching,
- multiplayer lobby and meeting/session style features.

This level of polish matters for:

- action mapping design (RC-style control semantics),
- reward shaping through failure/rewardful events,
- reproducible scenario sets in manual mode.

---

## Rendering and runtime assumptions

The public feature list includes multiplayer and user-space session systems; this makes it likely that:

- frame-loop timing is tied to client render/runtime cadence,
- state introspection is primarily for display/debug, not machine learning loops,
- any high-throughput stepping must pass through UI/input simulation unless private APIs are acquired.

---

## API and integration surface for PufferLib

No public API/SDK path is exposed in the primary landing/release pages we reviewed.

That leaves you with four pragmatic options:

### Option A — Direct source/legacy engine bridge
Only viable if you obtain an internal developer/API contract from vendor.

### Option B — Black-box desktop harness
- Drive input channels from automation (joystick key injection).
- Record/parse any accessible flight telemetry or replay artifacts.
- Use vision-based verification if no state stream exists.

### Option C — Offline dataset path
- Use neXt as data generator for demonstrations only (if recordings are exportable).
- Train an internal environment from those episodes.

### Option D — skip as direct RL target
- Keep neXt as comparison baseline and move to simulators with explicit APIs for active training.

For the stated realism-first RL mission, B and C are more practical than A unless vendor access changes.

---

## SPS / throughput expectation

No public measurement for RL or headless batch exists.
Based on product shape:

- vectorized simulation is unlikely,
- frame/input-dependent stepping likely limits raw SPS,
- deterministic resets are not documented as script-only operations.

Treat throughput as **low-to-medium practical**, and only validate with a custom harness if you confirm an automation contract.

---

## Candidate score for your stack

| Dimension | Assessment |
|---|---|
| Realism / RC feel | B |
| Transparency for research | D |
| Direct RL API | D |
| PufferLib fit | D |
| Recommended role | Historical/feature comparison and multiplayer behavior studies |

---

## Comparison position

| Simulator | Openness | API path | Integration risk | Why keep |
|---|---|---|---|---|
| [[JSBSim]] | high | C++/Python | low-medium | best direct RL training path |
| [[FlightGear]] | high | protocol bridge | medium | open architecture + visuals |
| [[X-Plane]] | high (SDK + web API) | plugin + web | medium | modern commercial baseline |
| [[ClearView RC Flight Simulator]] | proprietary | no public step API | high | control workflow reference |
| [[CRRCSim]] | open source | source/CLI only | medium | RC model corpus and old architecture |
| **neXt RC** | proprietary | none discovered | high | legacy RC content + multiplayer behavior |

---

## Related notes

- [[PufferLib Flight Sim Integration Gaps]]
- [[PufferLib Flight Throughput Benchmark]]
- [[Flight-Sim RL Contract]]
- [[FlightGear]]
- [[X-Plane]]
- [[JSBSim]]
- [[Aerofly RC10]]
- [[Absolute RC]]

---

## External resources

- https://www.cgm-online.com/rc-flight-simulator/rc-flight-simulator_e.html
- https://www.cgm-online.com/rc-flight-simulator/download_e.html
- https://www.cgm-online.com/rc-flight-simulator/next-e.html
- https://www.cgm-online.com/rc-flight-simulator/next-history_e.html
- https://www.cgm-online.com/rc-flight-simulator/next-usb.html

## Source notes

[^next-home]: neXt RC product page, including modes/features and market framing, https://www.cgm-online.com/rc-flight-simulator/rc-flight-simulator_e.html, inspected 2026-06-13.
[^next-download]: Official download/distribution page, https://www.cgm-online.com/rc-flight-simulator/download_e.html, inspected 2026-06-13.
[^next-unity]: neXt documentation and release-history references to Unity versions, https://www.cgm-online.com/rc-flight-simulator/next-e.html and https://www.cgm-online.com/rc-flight-simulator/next-history_e.html, inspected 2026-06-13.
[^next-input]: USB/controller and input workflow notes, https://www.cgm-online.com/rc-flight-simulator/next-usb.html, inspected 2026-06-13.
