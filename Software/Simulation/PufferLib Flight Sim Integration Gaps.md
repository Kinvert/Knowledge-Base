---
title: PufferLib Flight Sim Integration Gaps
tags:
  - reinforcement-learning
  - pufferlib
  - flight-simulation
  - simulation-integration
  - research-plan
aliases:
  - PufferLib Flight Sims Gaps
  - Flight Sim RL Stack Gaps
---

# PufferLib Flight Sim Integration Gaps

This note tracks what is still missing if the goal is realistic RL on flight physics with [[PufferLib]] and multiple simulators.

Current captured notes in the vault:
- [[JSBSim]]
- [[FlightGear]]
- [[X-Plane]]
- [[Microsoft Flight Simulator]]
- [[ClearView RC Flight Simulator]]
- [[CRRCSim]]
- [[Absolute RC]]
- [[Aerofly RC10]]
- [[neXt RC]]
- [[Flight-Sim Migration Funnel]]
- [[ROS2 Aerial Integration]]

Use this as the gap register before adding more simulator notes.

---

## ✅ Coverage We Already Have

- Per-simulator profile with language/rendering/API/throughput assessment (ClearView, JSBSim, FlightGear, X-Plane, Microsoft Flight Simulator, CRRCSim, Absolute RC, Aerofly RC10, neXt RC).
- Explicit “PufferLib fit” notes for each simulator.
- Notes on whether each simulator has a direct RL step API or indirect control channels.
- Initial comparison tables for candidate selection.

---

## ⚠️ Gaps That Could Impact Your Goal

## 1) Missing integration architecture (highest gap)

No shared adapter model exists yet for:
- unified action space translation (sim-specific control names => canonical RL controls)
- canonical observation schema (e.g., attitude rates, AoA, altitude, sideslip, airspeed, control effector states)
- common reset contract (state seed + environment seeding + crash/restart semantics)
- reward factory (dense/sparse reward templates for flight tasks)

Why this matters for PufferLib:
- Without a shared contract, every simulator wrapper becomes custom and brittle.
- High SPS training benefits from consistent observation/action layout and zero-copy buffers.

Recommended note to write next: **Flight-Sim RL Contract** that is simulator-agnostic.

---

## 2) Missing SPS/throughput benchmark methodology

Current notes mostly list “low/medium/high” inferences, but there is no common benchmark harness definition yet.

Need to define and lock:
- fixed sim tick (for example: 50 Hz, 100 Hz, 200 Hz)
- fixed episode length and terminal conditions
- CPU pinning / thread affinity
- single-instance latency vs vectorized throughput
- separate metrics for step time, reset time, observation read/write, and policy inference overlap

Why this matters for PufferLib:
- SPS claims become comparable only with identical episode protocol and fixed determinism assumptions.

Recommended note to write next: **PufferLib x Flight-Sim Throughput Harness**.

---

## 3) Missing migration/validation stack

No dedicated note yet on staged training:
1. train on a fast deterministic core (likely [[JSBSim]])
2. transfer to visual/full-stack sim (e.g., [[FlightGear]] or [[X-Plane]])
3. perform sim-to-real-style policy diagnostics and reward deltas

Needs explicit guidance for:
- policy serialization compatibility between envs
- control gain retuning between sims
- sensor/noise modeling at each stage

Recommended note to write next: **Flight-Sim RL Migration Funnel**.

---

## 4) Missing simulator families beyond current set

The following are still not covered in the note set:
- hardware-in-the-loop/autopilot timing workflows (cross-SITL state-reset determinism and multi-airframe timing traces)

Current priority for remaining depth: document **ROS/ROS2 in-the-loop deterministic reset and reset-policy** semantics (the base connection pattern is now covered in [[ROS2 Aerial Integration]]).

---

## 5) Missing integration-level edge cases

Needs explicit treatment of:
- determinism constraints:
  - plugin callback timing (especially [[X-Plane]])
  - UDP packet loss/jitter (if using external socket bridges)
  - GUI/render loop coupling (especially [[FlightGear]])
- reproducibility:
  - seeding at episode start across processes
  - fixed-point vs floating-point drift and reset boundaries
- legal/commercial constraints:
  - redistribution and automation limits for proprietary simulators

These are lower-level risks than simulator fidelity but high-impact for experiments.

---

## 6) Missing explicit PufferLib adapter patterns by category

There is per-simulator guidance already, but not a canonical comparison by adapter strategy:
- in-process C/C++ plugin/native callback adapter
- external process bridge (socket/HTTP/IPC)
- black-box input + vision reward pipeline
- source-level fork or instrumentation path

For each path we should track:
- expected throughput
- implementation cost
- failure modes
- reset/cleanup strategy

---

## 7) Missing environment/schema compatibility checks

No note yet covers cross-tool compatibility checks for:
- unit consistency (m/s, rad/s, ft/s, knots)
- attitude conventions (NED, ENU, body frame, world frame)
- control axis sign/ordering conventions

These issues can silently invalidate transfer even when step/return code appears stable.

---

## Priority Matrix (Next 6 Notes)

| Priority | Note to add | Why this helps |
|---|---|---|
| 1 | **Flight-Sim RL Contract** (canonical action/obs/reward/done schema) | Makes every simulator wrapper swappable for RL comparisons. |
| 2 | **PufferLib Flight Throughput Benchmark** | Replaces inferred rankings with measurable SPS comparisons. |
| 3 | **JSBSim => FlightGear External FDM Bridge** | Practical path for visual + deterministic physics split. |
| 4 | **X-Plane Plugin Adapter Blueprint** | Formalizes queueing, timing, and callback safety patterns. |
| 5 | **MSFS / SimConnect Adapter Pattern** | Expands to civil-aviation simulation with SimConnect alternatives. (Now partially covered by the MSFS note; add benchmark-focused path next.) |
| 6 | **ROS/ROS2 Aerial Integration Note** | Adds actuator/autopilot stack hooks and real-world flight hardware bridge. (Added.) |

---

## What to write first (if realism is priority)

1. **Done**: first deliverable — [[Flight-Sim RL Contract]] and [[PufferLib Flight Throughput Benchmark]].
2. Next: refactor existing sim notes to a single adapter schema (use the two new notes as schema anchors).
3. Next: add ROS/ROS2 integration stack with explicit API status and deterministic reset templates.
4. Next: pilot policy migration from [[JSBSim]] to [[FlightGear]] or [[X-Plane]] using a fixed contract.

---

## Quick next actions

- Add one integration checklist item per simulator:
  - reset determinism
  - no hidden random seed drift
  - no per-step unbounded allocations
  - bounded IPC and bounded callback duration
- Refactor existing sim notes to explicitly map into [[Flight-Sim RL Contract]] channels/fields.
- Add and publish a first benchmark run in [[PufferLib Flight Throughput Benchmark]] across JSBSim-native, JSBSim-Python, and FlightGear protocol modes.
- Benchmark at three scales: single env, 64 env, 4096 env equivalents (or closest possible for each architecture).

---

## External Notes / References

- [[PufferLib]]
- [[PufferLib C99 Environment Authoring]]
- [[PufferLib Robotics Fit and Limits]]
- [[Vectorized Environments]]
- [[JSBSim]]
- [[FlightGear]]
- [[X-Plane]]
- [[ClearView RC Flight Simulator]]
- [[CRRCSim]]
