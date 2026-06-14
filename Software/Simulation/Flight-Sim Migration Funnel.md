---
title: Flight-Sim Migration Funnel
aliases:
  - Flight Sim Migration Funnel
  - Flight Simulation Migration Plan
tags:
  - flight-simulation
  - reinforcement-learning
  - pufferlib
  - sim2real
  - transfer-learning
---

# Flight-Sim Migration Funnel

This note is the practical sequence for using multiple simulators without losing reproducibility or spending RL wall-clock on noisy transitions.

Use it when your priority is **real flight physics + scalable training with PufferLib**.

---

## Why this funnel

Flight simulators are in three clusters:

1. **Physics runtime candidates** (`JSBSim`, possible future `MSFS`/`X-Plane` core paths),
2. **Full-stack visual simulators** (`FlightGear`, `MSFS`, `X-Plane`, `Aerofly RC10`),
3. **Reference/legacy simulators** (`ClearView`, `CRRCSim`, `neXt RC`, `Absolute RC`) that are valuable for behavior comparison but weak for direct RL throughput.

The funnel separates:

- *learning dynamics efficiently*,
- *validating behavior under realistic rendering/weather/ATC-like conditions*,
- *transfer and sim-to-real risk control*.

---

## Funnel phase 1: Deterministic core policy training

### Target simulators
- [[JSBSim]] (primary candidate)
- `[[PufferLib C99 Environment Authoring]]`-style custom dynamics kernels when needed

### Contract requirements
- fixed `dt`,
- deterministic `step(action)` and `reset(seed)`,
- contiguous `obs/action` buffers,
- explicit reward and termination fields,
- stable units/frame conventions.

### What to prove here
- reward learning converges on synthetic tasks,
- policy is stable under perturbations,
- reset logic handles failure states correctly.

This phase should deliver the highest SPS and best iteration speed.

---

## Funnel phase 2: Visual and systems transfer

### Target simulators
- [[FlightGear]] as externalized-rendering/physics proxy,
- [[X-Plane]] plugin/web bridge path for high-entropy commercial physics comparison,
- [[Microsoft Flight Simulator]] as visual/fidelity sanity target.

### Adapter rules
- keep the **same canonical action map** as phase 1,
- keep the same observation schema at least for:
  - attitude/position/velocity,
  - control output and resource flags,
  - event/mission context.
- use adapter-level reward decomposition only for differences in simulator conventions.

### What to measure
- policy score drop vs phase 1,
- reset consistency under repeated start states,
- crash/instability rate under equivalent disturbances,
- runtime throughput delta (reported separately from fidelity).

---

## Funnel phase 3: Policy selection and sim realism audit

### Target evaluation questions
- Is reduced variance from phase 1 acceptable in richer simulator?
- Do crashes correlate with physically meaningful failures (not adapter bugs)?
- Which reward channels become brittle across engines?

### Decision points
- if policy fails under one stack due to control-rate mismatch, tighten action-rate and dead-zone terms in phase 1,
- if observations diverge due to conventions (e.g., frame conventions), enforce conversion in adapter layer.

---

## Migration check matrix (minimum)

| Stage | Required checks | Pass criteria |
|---|---|---|
| S0 | Contract lock | `act_map`, `obs_map`, units, dt, reward names are versioned |
| S1 | Core training | deterministic resets, stable SPS, no NaNs |
| S2 | Cross-sim transfer | reward correlation above threshold, bounded failure rate |
| S3 | Visual stack integration | no adapter memory leaks, bounded callback latency |
| S4 | Release | stable policy manifest + environment manifest + run manifests |

---

## Recommended schedule

1. Build JSBSim adapter + benchmark harness first.
2. Freeze action and observation contract once phase 1 is stable.
3. Add one visual stack at a time (`FlightGear`, then `X-Plane`, then `MSFS` if desired).
4. Keep one policy per stage and compare episode score deltas, not just raw reward.

---

## Suggested artifact template

```yaml
funnel:
  phase: phase1_core
  sim: jsbsim
  dt: 0.01
  max_steps: 1000
  act_map_version: v1
  obs_map_version: v1
  reward_schema_version: v1
  seed: 1234
  steps: 50000
  callback: false
  render: false
```

For phase2/phase3, keep the same schema manifest and only change:

- sim id,
- bridge transport,
- reset policy,
- renderer policy.

---

## Integration links

- [[Flight-Sim RL Contract]]
- [[PufferLib Flight Throughput Benchmark]]
- [[PufferLib Flight Sim Integration Gaps]]
- [[PufferLib]]
- [[JSBSim]]
- [[FlightGear]]
- [[X-Plane]]
- [[Microsoft Flight Simulator]]
- [[CRRCSim]]

---

## Source notes

No external new sources were introduced in this design note; it operationalizes the strategy described across [[Flight-Sim RL Contract]], [[PufferLib Flight Throughput Benchmark]], and simulator-specific notes.
