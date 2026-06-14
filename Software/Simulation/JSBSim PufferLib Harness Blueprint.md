---
title: JSBSim PufferLib Harness Blueprint
aliases:
  - JSBSim PufferLib Harness Blueprint
  - JSBSim PufferLib Adapter
  - JSBSim RL Harness
tags:
  - flight-simulation
  - reinforcement-learning
  - pufferlib
  - jsbsim
  - c++
  - c
  - c99
---

# JSBSim PufferLib Harness Blueprint

This blueprint is the practical path to get **high SPS RL training** with a controllable, reproducible flight engine.  
The design goal is to make JSBSim the canonical physics core while keeping PufferLib requirements explicit and testable.

Core idea: keep JSBSim as a deterministic step function, and keep PufferLib as the rollout engine.

---

## Why JSBSim first

- JSBSim is a C++ 6-DoF flight-dynamics core with standalone/batch operation and `FGFDMExec::Run`-style stepping.[^jsbsim-ref]
- It is designed to run without a renderer path; rendering is external, so it is a better fit for high-throughput RL loops than full visual stacks.[^jsbsim-home]
- PufferLib’s highest-throughput pattern is contiguous buffers + vectorized stepping; JSBSim can support that if wrapped natively rather than through slow per-step Python calls.[^puffer-docs]

---

## Target architecture

Use this order:

1. **Canonical control loop** in C++ (recommended for production SPS).
2. **Optional JSBSim Python shim** for fast experimentation only.
3. **Optional FlightGear visual adapter** for debugging only.

### Recommended stack

- Physics: JSBSim C++ (`FGFDMExec`)
- RL interface: PufferLib env binding (C99-compatible contiguous arrays)
- Optional debug/visual: separate FlightGear process consuming JSBSim output packets
- Observation/reward/done buffers: stable fixed-size schema from [[Flight-Sim RL Contract]]

---

## Contract assumptions (hard requirements)

In every JSBSim build, enforce:

- fixed `dt` (start with `0.01`)
- `step(action)` = exactly one sim tick
- fixed aircraft model per benchmark run
- deterministic reset by script/initial conditions
- one control message per axis per step
- no dynamic allocation in the hot path
- contiguous NumPy/PyTorch-readable obs/actions in host memory

If you can’t meet these, you will not get reliable SPS scaling.

---

## Data mapping strategy

### Action map (example fixed-wing baseline)

Use normalized actions and apply explicit scales in adapter code:

| Index | JSBSim property path (example) | Default scale |
|---|---|---:|
| 0 | `fcs/aileron-cmd-norm` | `[-1, 1]` |
| 1 | `fcs/elevator-cmd-norm` | `[-1, 1]` |
| 2 | `fcs/rudder-cmd-norm` | `[-1, 1]` |
| 3 | `fcs/throttle-cmd-norm` | `[0, 1]` |

Keep a JSON/CSV `act_map.json` in repo and fail-fast if a property is missing.

### Observation map (starter)

Use at least:

- position (`position/lat-geod-deg`, `position/long-geod-deg`, `position/geod-alt-ft`)
- orientation (`attitude/roll-deg`, `attitude/pitch-deg`, `attitude/heading-deg`)
- rates (`velocities/p-rad_sec`, `velocities/q-rad_sec`, `velocities/r-rad_sec`)
- body speed (`velocities/u-fps`, `velocities/v-fps`, `velocities/w-fps`)
- air data (`velocities/mach`, `velocities/airspeed-kt`)
- control outputs (`fcs/roll-trim` etc. as needed)

All observations should be written into a preallocated `float32` vector in fixed order.

### Unit policy

Pick one canonical unit set at contract level and convert on ingress:

- angle: radians in obs
- velocity: m/s
- altitude: meters
- rates: rad/s

Store conversion once per update; do not convert on the learner side every loop.

---

## Environment shape for PufferLib

Recommended shapes:

- `act_dim = 4` to start (A/E/R/throttle)
- `obs_dim = 32` to leave room for rates, airdata, control history, and flags
- reward scalar, `terminated` and `truncated` as `uint8`

Align with `[[Flight-Sim RL Contract]]` fields:
- `obs`
- `rewards`
- `terminated`
- `truncated`
- `info` (fixed-size fields preferred; JSON-encoded string fallback for non-critical metadata)

---

## Adapter implementation plan

### Phase 1 — JSBSim-native reference env

Implement a minimal C++ environment with:

- `init_sim(model, init_script)` → loads aircraft + script + IC
- `step(action)`:
  1. write control properties
  2. `fdm.Run()`
  3. read outputs into obs buffer
  4. compute reward and done flags
- `reset(seed)`:
  1. clear sim state
  2. reload initial conditions
  3. reseed deterministic RNGs

### Phase 2 — C ABI / C99 glue

Expose only this ABI:

- `env_create(cfg_json)`
- `env_reset(env_id)`
- `env_step(env_id, act_ptr, obs_out_ptr, reward_out_ptr, done_out_ptr)`
- `env_close(env_id)`

This is the cleanest path for PufferLib bindings and vectorization.

### Phase 3 — PufferLib wrapper

- Keep tensor buffers contiguous and typed (`float32` / `uint8`).
- Batch `N` environments per step in a single launcher thread.
- Measure step coverage and reset latency per step batch.

---

## Parallelization strategy

For each worker:

- one JSBSim instance per environment
- one simulator thread (or single-threaded event model)
- one action copy into `FGPropertyManager` inputs
- one `Run()` call per loop

Scale:

- **Phase A:** `N=1` sanity
- **Phase B:** `N=8/16/64` to find cache/callback slope
- **Phase C:** vectorization stress (`N=256+`) only if memory and lock-free state path holds

Do **not** start with one shared JSBSim + many env IDs unless proven thread-safe in your exact build; share only model assets, not mutable runtime state.

---

## Reset contract (critical for RL)

Define reset outcome as deterministic tuple:

- start position
- heading
- speed
- weather profile (even if fixed)
- random seed used by any scripted perturbations

Reset should not “teleport” through ad-hoc scripts; it should run through a known IC sequence and clear transient integrators.

Suggested output in `info`:
- `reset_seed`
- `reset_episode`
- `step_count_at_reset`
- `sim_time_sec`

---

## Determinism checks to run immediately

For each candidate commit:

1. same `(seed, episode_idx, action_seq)` → identical obs/reward on two runs  
2. bounded step timing jitter (`p99`) under fixed load  
3. no property write ordering drift  
4. no per-step heap growth (RSS/alloc counter)  
5. identical reset state hash at episode boundaries

Record:
- `git` revision
- model file hashes
- JSBSim version/hash
- compiler flags / CPU affinity
- benchmark manifest and hardware

---

## SPS and benchmark harness notes

Use [[PufferLib Flight Throughput Benchmark]] scaffold and add a JSBSim-native row:

- `N = 1, 64, 4096(if feasible)`
- include:
  - `SPS_overall`
  - `SPS_per_env`
  - `p99_step_ms`
  - `reset_p99_ms`
  - `episode_len_mean`
  - dropped-step/invalid-state count

Expected profile:
- highest SPS among surveyed options when C++/C99 path is direct
- low variance if action and obs buffers are fixed-size and no GUI/process jitter

---

## Minimal folder layout

Use this layout in your repo:

```text
sim/
  jsbsim_adapter/
    CMakeLists.txt
    include/
      jsbsim_env.h
      flight_contract.h
    src/
      jsbsim_env.cpp
      puffer_iface.cpp
      io_buffers.cpp
  configs/
    jsbsim/
      c172.yaml
      c172_reset_states.yaml
  scripts/
    run_benchmark_jsbsim_puffer.py
    build_env.sh
```

Keep aircraft `.xml` and scripts under config directories with explicit hashes.

---

## Useful implementation pattern (pseudo-C++)

```cpp
// Pseudocode only
step(action) {
  write_norm_control(fdm, action[0], action[1], action[2], action[3]);
  fdm.Run();
  collect_obs(obs_buf, obs_dim);         // fixed offsets only
  compute_reward(reward_buf[env_id]);    // deterministic formula
  check_done(done_term[env_id], done_trunc[env_id]);
}

reset() {
  fdm.SetInitConditions(ic_name);
  fdm.RunIC();
  clear_episode_counters();
  snapshot_reset_manifest();
}
```

---

## What not to do

- Don’t call JSBSim property lookups by string every step (cache IDs/property handles first).
- Don’t route observations through Python lists or per-step dicts.
- Don’t mix frames/units across environments.
- Don’t treat rendering as part of training loop (visual path should be optional).

---

## Deployment checklist

- [ ] C++ env compiles and runs fixed-seed replay
- [ ] Reset reproduces exact initial state
- [ ] obs buffer is contiguous and fixed shape
- [ ] p95/p99 step timing captured
- [ ] `info` includes reset/episode metadata
- [ ] run 3-scale benchmark (1, 64, 256+) and store raw CSV/parquet

---

## Related notes

- [[JSBSim]]
- [[PufferLib]]
- [[PufferLib C99 Environment Authoring]]
- [[Flight-Sim RL Contract]]
- [[PufferLib Flight Throughput Benchmark]]
- [[Flight-Sim Migration Funnel]]

---

## Sources

[^jsbsim-ref]: JSBSim API and runtime references, including `FGFDMExec` and standalone integration posture. https://jsbsim-team.github.io/jsbsim/ and https://github.com/JSBSim-Team/jsbsim
[^jsbsim-home]: JSBSim reference manual/overview documents standalone/batch operation and six-DoF model orientation. https://jsbsim-team.github.io/jsbsim-reference-manual/user/overview/
[^puffer-docs]: PufferLib documentation emphasizes contiguous memory and vectorized env throughput. https://puffer.ai/docs.html
[^fg-output]: JSBSim output and input socket notes for external visualization/debug channels. https://jsbsim-team.github.io/jsbsim/classJSBSim_1_1FGOutput.html and https://jsbsim-team.github.io/jsbsim/classJSBSim_1_1FGInputSocket.html
