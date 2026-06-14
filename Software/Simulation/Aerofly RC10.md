---
title: Aerofly RC10
aliases:
  - Aerofly RC
  - Aerofly RC10
  - Aerofly RC10 Simulator
tags:
  - simulation
  - reinforcement-learning
  - flight-simulation
  - rc
  - pufferlib
---

# Aerofly RC10

`Aerofly RC10` is a closed, high-fidelity RC simulator from Ikarus with a desktop-first architecture, extensive control-device support, and a relatively mature scenario/model ecosystem.  
For RL, it is a strong reference for piloting handling and rendering quality, but it is not currently documented as a native headless RL environment.

---

## Research Status

| Question | What we know | Confidence |
|---|---|---:|
| Product type | Desktop RC sim with official Steam and Ikarus distribution | High |
| Physics features | Real-time fixed-wing/helicopter flight model with weather/wind and rotorcraft/airframe parameters in manual | Medium |
| Rendering | Windows-focused flight-viewer stack with modern graphics requirements and VR support | Medium-High |
| API surface | No official SDK/API portal found in primary manuals; public docs center on controls, setup, and controls training | Medium |
| PufferLib suitability | Medium-low direct fit; best as black-box transfer benchmark or reward/vision proxy | Low |

---

## Simulation stack and model control

Primary claims from official docs indicate:

- model folders are user-editable and include model files and setup data;
- flight controls support multiple channel types and flight mode behavior;
- environment/weather/wind behavior is represented in scenario/model setup.

The practical upside is strong data to build your own task ontology (start conditions, weather perturbations, failure conditions, control authority patterns).

The downside for PufferLib is the lack of a published simulator-native step API.

---

## Rendering and runtime assumptions

From official requirement and manual pages:

- target desktop graphics and high-end input stack expectations are documented;
- VR is available as a user feature;
- the manual focuses on cockpit/external viewing and training workflows.

This suggests a **full-frame, user-interactive runtime** rather than an exposed deterministic environment kernel.

If RL instrumentation is needed, expect:

- frame-time coupling unless custom headless mode is found experimentally;
- significant overhead from screenshot/vision pipelines;
- reset/respawn latency driven by desktop process lifecycle.

---

## Inputs and control mapping

Aerofly RC10 exposes a complete user-level control model:

- gamepad/joystick support,
- advanced stick calibrations,
- profile/aircraft assignment workflows,
- wind and autopilot scenario settings,
- training and mission controls in the desktop UI.

This is useful because you can derive stable **control-semantic mapping** (elevator/aileron/rudder/throttle / mode) and keep action preprocessing consistent with real RC workflows.

---

## API / Automation for PufferLib

The official pages and manual index are oriented around playable workflows:

- setup/tutorial,
- control calibration,
- joystick/gamepad integration,
- scenario editing,
- flight recording features,
- multiplayer.

I did not find a public docs page for:

- external gRPC/REST SDK,
- deterministic batch stepping command,
- fixed-step reset callback,
- direct vectorized state stream.

That makes Aerofly RC10 unsuitable as a first-class direct RL env wrapper in its public form.

### Realistic harness options

| Harness pattern | What you get | Cost / caveat |
|---|---|---|
| External desktop automation + vision rewards | fastest to validate policy behavior | brittle and expensive to get deterministic resets |
| Local file replay parsing (if accessible) | potentially cleaner than pixels | requires reverse-engineering save format + sync |
| Remote process proxy | isolate policy process from sim thread | callback jitter + no native contract |

---

## PufferLib adapter fit by architecture

For your goal, the best sequence is:

1. Use Aerofly as **policy behavior / visual realism validator**.
2. Keep `step`/`reset` in a native or semi-native dynamic core (JSBSim/FlightGear protocol) for throughput.
3. Introduce Aerofly after reward shaping and observation design are stable.

If you must include Aerofly in automated loops:

- push action commands through the smallest possible external channel,
- read state from any available structured logging channel first,
- gate all vision-based rewards with explicit temporal alignment and deterministic episode IDs.

---

## SPS / throughput expectation

No official RL-oriented step-rate metric is published.

Given current evidence:

- **No native headless stepping**
- **Likely full-frame rendering during execution**
- **No obvious vectorized instance model in public docs**

SPS is therefore expected to be poor versus in-process open FDM paths, unless a hidden automation bridge is discovered.

---

## Candidate score for this RL stack

| Dimension | Assessment |
|---|---|
| Realism for RC handling | A- |
| Reproducibility of state | C |
| Direct RL API | D |
| PufferLib fit | D |
| Recommended role | End-to-end behavior validation and transfer-quality check |

---

## Comparison position vs other flight sims

| Simulator | Openness | API path | PufferLib integration cost | Why include |
|---|---|---|---|---|
| [[JSBSim]] | high (open FDM) | C++/Python/CLI/outputs | low-medium | best training core |
| [[FlightGear]] | high (open framework) | protocol/bridge patterns | medium | visual + environment realism bridge |
| [[X-Plane]] | commercial + SDK/web API | plugin + web API | medium | high-fidelity and systems comparison |
| [[ClearView RC Flight Simulator]] | proprietary | no public step/state API | high | manual practice, no direct RL |
| [[CRRCSim]] | open source legacy | CLI/source coupling | medium | historical RC model corpus |
| **Aerofly RC10** | closed | no public API docs found | high | visual realism / transfer candidate |

---

## Related notes

- [[PufferLib Flight Sim Integration Gaps]]
- [[PufferLib Flight Throughput Benchmark]]
- [[Flight-Sim RL Contract]]
- [[FlightGear]]
- [[X-Plane]]
- [[JSBSim]]
- [[Absolute RC]]
- [[ClearView RC Flight Simulator]]

---

## External resources

- https://www.aeroflyrc.com/
- https://store.steampowered.com/app/2394350/aerofly_RC_10__RC_Flight_Simulator/
- https://www.ikarus.net/en/rc10-manual/
- https://www.ikarus.net/en/rc10-controls/
- https://www.ikarus.net/en/rc10-userdata/
- https://www.ikarus.net/en/rc10-requirements/

## Source notes

[^aerofly-home]: Aerofly RC10 official landing and Steam pages, https://www.aeroflyrc.com/ and https://store.steampowered.com/app/2394350/aerofly_RC_10__RC_Flight_Simulator/, inspected 2026-06-13. They identify official distribution and desktop RC product positioning.
[^aerofly-manual]: Aerofly RC10 manual index and control documentation, https://www.ikarus.net/en/rc10-manual/ and https://www.ikarus.net/en/rc10-controls/, inspected 2026-06-13.
[^aerofly-usr]: User-data and controls setup pages, https://www.ikarus.net/en/rc10-userdata/, inspected 2026-06-13.
[^aerofly-req]: Requirements and environment guidance, https://www.ikarus.net/en/rc10-requirements/, inspected 2026-06-13.
