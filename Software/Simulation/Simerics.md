---
title: Simerics
aliases:
  - Simerics CFD
  - Simerics-MP
  - PumpLinx
tags:
  - simulation
  - cfd
  - turbomachinery
  - rl
  - pufferlib
---

# Simerics

**Simerics** is a commercial CFD platform (Simerics-MP, Simerics-MP+, and PumpLinx variants) focused on fluid power, pumps, compressors, valves, turbo machinery, vehicle/marine systems, and coupled fluid system design. It is not a game-style simulator or real-time control environment.

---

## Quick profile

| Dimension | What is true |
|---|---|
| Domain | CFD for fluid machinery and fluid-mechanical systems |
| Core products | Simerics-MP, Simerics-MP+, PumpLinx |
| Main physics | Flow, turbulence, conjugate heat transfer, cavitation, aeration, multiphase, particles, FSI variants |
| Deployment style | Primarily GUI workflows with CAD integrations; some workflows are file-driven batch automation |
| RL usability | Best for batch design optimization loops, not direct high-frequency control step API |
| OS profile | Windows-centric product, older public release notes mention Linux + Windows support for MPI/DME modes |
| Source of this note | Official Simerics pages + CFturbo manual + SAE technical abstracts |

Source:
- [[Simerics products]](https://www.simerics.com/products/)
- [[Simerics-MP CFD]](https://simerics.de/simerics-mp/?lang=en)
- [[PumpLinx]](https://www.simerics.com/pumplinx/)
- [[Simerics + CFturbo collaboration]](https://www.simerics.com/simerics-and-cfturbo-sign-collaborative-agreement/)
- [[CFturbo manual > Simerics export]](https://manual.cfturbo.com/en/simerics.html)
- [[Simerics v5.0 release notes]](https://8020engineering.com/wp-content/uploads/2019/07/release_notes_5.0_from_help_manual.pdf)

---

## What this is and what it is not

Simerics is a high-end **virtual testing CFD stack**, not a flight simulator.

It is used for:
- rotating component simulation (pumps, compressors, turbines)
- valve and leakage behavior
- multiphase and cavitation/aeration studies
- integrated system hydraulics and thermal flow

This is relevant to realistic RL when you want a high-fidelity simulation oracle for design/control objectives, but it is not directly suitable as a low-latency per-step MDP environment.

---

## Core products and technical strengths

### Simerics-MP

- Includes core 3D CFD functionality with flow, turbulence, heat transfer, aeration, cavitation, particles, FSI-adjacent features, and moving parts support.
- Core solver methodology is described as RANS finite-volume with steady/unsteady, compressible/incompressible flow coverage and broad non-Newtonian/variable property support. [[MP Flow]](https://www.simerics.com/mp-flow/)
- Commercial marketing repeatedly claims the full solver is used for liquid + gas multiphase flow where appropriate [[Simerics-MP]](https://simerics.de/simerics-mp/?lang=en).

### Simerics-MP+

- Extends MP with workflow-focused features for specific sectors (vehicle, marine, turbo, valves, positive displacement, systems).
- Product positioning lists sector extensions and templates for pumps, valves, turbines, and multi-component systems [[Products]](https://www.simerics.com/products/).

### PumpLinx

- Positioning states PumpLinx is transient 3D fluid systems software (pumps, motors, compressors, valves) with:
  - cavitation/aeration
  - heat/flow/turbulence
  - particle tracking
  - 1-DOF FSI templates (from public release text)
  - fast model-to-simulation path from CAD, with claims like ~15 min for steady-state steady solves in marketed examples [[PumpLinx]](https://www.simerics.com/pumplinx/).
- It is described as Simerics legacy/baseline technology plus rotating/sliding component templates.

### Physics and meshing pipeline

- Simerics emphasizes automated meshing with a proprietary Conformal Adaptive Binary-tree (CAB) approach (Cartesian hexahedral cells, local adaptive refinement, tolerance to non-perfect CAD seams) [[Automated Mesher](https://www.simerics.com/automated-mesher/)].
- For a positive displacement / rotating component benchmark, this matters because it supports micro-gaps and tight clearances without full manual remesh effort.

---

## CAD and workflow integrations

### CAD plugins / CAE interfaces

Simerics positioning uses CAD-centric workflows and file exchange, while implementation details in CFturbo manuals are the most stable source for automation.

- CAD import/export and solver handoff is commonly done via CFturbo as an intermediary step.
- CFturbo’s export interface table explicitly lists Simerics as consuming `.spro` + `.stl` and supports component-wise export to Simerics workflows. [[CFturbo CFD export interfaces]](https://manual.cfturbo.com/en/export_cfd.html)
- The Simerics export settings in CFturbo include solver selection (Simerics-MP or Simerics-MP+), transient/steady controls, and output configuration that lands in the `.spro` artifact. [[Simerics export settings]](https://manual.cfturbo.com/en/simerics.html)
- A publicized Simerics+CFturbo collaboration confirms this as the integration model for a full design → analysis → simulation chain. [[Simerics and CFturbo agreement]](https://www.simerics.com/simerics-and-cfturbo-sign-collaborative-agreement/)

### Public integration model

Public documentation shows a **file/project handoff model** rather than a documented in-process step API:
- `.spro` configuration export + STL geometry export.
- Mesh and solver settings passed through the same file-oriented interface.

That means a realistic PufferLib integration is usually:
1) write/update project/settings files, 2) launch solver process, 3) parse results files into observations/rewards.

### CFturbo coupling

CFturbo documents its export workflow to Simerics:
- Export `.spro` configuration and STL files
- Pass solver options and mesh settings into Simerics pipeline
- Use either Simerics-MP or Simerics-MP+ solver selection
- Support for transient/steady control settings and convergences etc. [[CFturbo Simerics manual]](https://manual.cfturbo.com/en/simerics.html)

This is the practical "handoff point" for industrial design-to-oracle pipelines if you are already in turbomachinery territory.

### GT-Suite coupling example

Simerics CAE coupling docs show 1D/3D coupling examples with GT-SUITE and mention CFturbo export paths for turbo-machine map workflows [[CAE Coupling]](https://simerics.de/anwendungen/process-sumaliton/?lang=en).

---

## Performance / compute context

### Official claims to use carefully

- Simerics released 6.1.2 (March 2026 announcement) with AMD work on thread pinning and process placement for Ryzen Threadripper PRO and Ryzen AI PRO 300 series platforms.
- The announcement states internal tests showed ~25% speedup with thread pinning in that context [[AMD announcement]](https://www.simerics.com/wp-content/uploads/2026/04/Simerics-and-AMD-Announcement.pdf).
- Older release notes describe MPI on multiple computers and DME (Direct Memory Exchange) single-node parallel modes, tested on both Windows and Linux, plus a rich template library for marine, vehicle, and fuel-system setups [[Release notes]](https://8020engineering.com/wp-content/uploads/2019/07/release_notes_5.0_from_help_manual.pdf).

### Practical implication for RL

Even with good scaling, Simerics is still an engineering CFD tool; wall-clock per episode is typically orders of magnitude larger than what PufferLib expects from a control MDP loop if every `step()` is a full CFD solve.

---

## Relevance to PufferLib (practical answer)

### What is likely viable

1. **Parameter-to-performance optimization (recommended first)**
   - Action space = design parameters or operating setpoints
   - Environment step = launch one Simerics campaign for a parameter batch
   - Observation = extracted scalar fields, map points, performance metrics
   - Reward = efficiency, NPSH margins, losses, ripple, etc.

2. **Data generation / simulation-of-simulation**
   - Use Simerics as the expensive ground-truth generator
   - Train a fast surrogate that becomes your `PufferLib` high-SPS environment

3. **System-level co-optimization**
   - Combine geometry variables + controller gains + operating point schedules as higher-level decision variables.

### What is currently weak

- No widely published low-level `step(action)` API in public Simerics docs.
- No public, documented REST or direct Python/CC API in the sources collected here.
- Stronger inference from workflows is to use **file-driven batch execution** (via `.spro` exports and solver process orchestration), not a direct embedded C step hook.
- Public docs emphasize file/project handoff and CLI orchestration: CFturbo batch/interactive commands, batch XML, and parser-friendly outputs.

This mirrors the strategy already used for some "engineering oracle" stacks: optimize policy at a slower loop, then distill to faster differentiable/real-time models.

### What is directly automatable

CFturbo (the primary launch surface around Simerics in many workflows) is explicitly scriptable:

- Batch mode runs without screen display:  
  `cfturbo.exe -batch <batch file> [-verbose] [-export <interface name>] [-log <log file>]`  
  where batch file is `.cft-batch` or `.cft`. [[Batch mode manual]](https://manual.cfturbo.com/en/batch_mode.html)
- `.cft-batch` is XML with `CFturboBatchProject`, `Updates` (input variables), and `BatchAction` blocks (for export/save tasks). [[Batch file format]](https://manual.cfturbo.com/en/batch_mode.html)
- Batch runs can emit parameter deltas in `.cft-res`, useful for constrained post-checks and optimization pipelines. [[Batch output]](https://manual.cfturbo.com/en/batch_mode.html)
- Batch mode can be launched from CLI directly, while interactive mode can also start from project/batch file paths and supports `-readonly` for license-safe inspection. [[Interactive mode]](https://manual.cfturbo.com/en/interactive_mode.html)
- `cfturbo` batch execution exit codes are documented: `0` success, `1` success with warnings, `2` errors. [[Exit codes]](https://manual.cfturbo.com/en/exit_codes.html)
- BLADERUNNER wraps this workflow with queueing of operating points, run-type controls, and `*.sres` result handling for steady/transient runs. [[CFturbo BLADERUNNER flow control]](https://manual.cfturbo.com/BLADERUNNER/simulation_control.html), [[CFturbo BLADERUNNER setup]](https://manual.cfturbo.com/BLADERUNNER/simulation_setup.html)

### Result artifacts to parse for RL

For observation vectors, the practical data sources are solver outputs and metadata, not direct sockets:
- `.sres` files for each completed steady/transient run (status shown in BLADERUNNER and removed only if workspace cleanup occurs). [[BLADERUNNER status and results]](https://manual.cfturbo.com/BLADERUNNER/simulation_control.html)
- `*.cft-res` for realized parameter values after constraint enforcement in batch updates. [[Batch output]](https://manual.cfturbo.com/en/batch_mode.html)
- `.log` files from batch runs (`-log`) for timing/errors and for postmortem triage. [[Batch mode manual]](https://manual.cfturbo.com/en/batch_mode.html)

### Practical implication for PufferLib architecture

If you treat each `step()` as:

1. action -> write `.cft-batch` parameter updates,
2. launch `cfturbo.exe -batch ...` (or CFturbo + BLADERUNNER command),
3. wait for return code and `.sres`/result artifacts,
4. parse a compact metric vector back into obs/reward/termination,

then you get deterministic execution and good auditability. You do not get high-frequency inner-loop control dynamics unless you add a separate reduced model.

### Practical bridge pattern options for PufferLib

#### Option A — direct campaign step (lowest engineering risk, lowest SPS)
Treat each PufferLib `step()` as one batch campaign:
1. map action vector -> `.spro`/parameter input update
2. launch solver process (or CFturbo export+solver chain)
3. parse result files (`.sres`, `.cft-res`, exported outputs) into scalar reward terms
4. return compact observation + reward

This is robust, audit-friendly, and good for design-parameter RL.

### Harness template (minimum viable, reproducible)

1. Parse action -> write/update a `.cft-batch` file from a deterministic template.
2. Launch `cfturbo.exe -batch <batch file> -verbose -log <run.log>`.
3. Check process exit code:
   - `0` no errors,
   - `1` warnings,
   - `2` errors.
4. Parse `.cft-res` and `.sres` outputs into compact observation scalars.
5. Return `obs`, `reward`, `done`, `truncation`, and metadata (`solve_ms`, `warnings`, `license`).

This keeps the environment loop explicit and testable:
- the environment state is just action history + file IDs + parsed result cache.
- BLADERUNNER is a better fit as campaign scheduler, not as a per-step control loop.

#### Option B — two-stage stack (recommended for throughput)
1. Run Simerics as expensive generator and build a dataset.
2. Fit a compact surrogate (`Neural Dynamics` or reduced-order map).
3. Train a high-throughput policy in a custom `PufferLib`-compatible environment.

This is usually the only viable route to high SPS while retaining Simerics-grade physical realism at the training target layer.

## Headless execution and scheduling reality check

For RL this matters because `-batch` execution is specifically non-interactive, but solve time dominates SPS:
- `cfturbo.exe -batch ...` is headless by design. [[Batch mode manual]](https://manual.cfturbo.com/en/batch_mode.html)
- `-readonly` interactive launches avoid extra license use for inspection workflows. [[Interactive mode]](https://manual.cfturbo.com/en/interactive_mode.html)
- BLADERUNNER queues operating points and tracks simulation status/results (`S/T` status markers, `steady/transient` control, `.sres` availability). [[BLADERUNNER control]](https://manual.cfturbo.com/BLADERUNNER/simulation_control.html), [[BLADERUNNER setup]](https://manual.cfturbo.com/BLADERUNNER/simulation_setup.html)
- Batch exit codes are machine-checkable (`0/1/2`). [[Exit codes]](https://manual.cfturbo.com/en/exit_codes.html)

Practical SPS estimate:
- **High-fidelity inner-loop policy steps:** poor.
- **Design/parameter optimization steps:** moderate to low, but schedulable with queue + caching.
- **Multi-stage training setup:** run Simerics/CFturbo as the oracle and train/roll out on a surrogate for throughput.

### PufferLib fit summary

| Use case | Fit for Simerics |
|---|---|
| High-SPS inner-loop control | Poor |
| Design-space RL / Bayesian optimization | Strong |
| Surrogate-model supervision loop | Strong |
| Full fleet optimization with long campaign times | Moderate to strong with robust scheduling |

---

## RL-oriented comparison against other tools

| Tool | Solver style | CAD coupling | Typical throughput profile | Best for |
|---|---|---|---|---|
| **Simerics-MP / PumpLinx** | Commercial CFD + automated meshing | Strong plugin orientation, `.spro` workflows | Low-to-moderate; batch campaign-oriented | Fluid machinery, valve/pump system optimization |
| **OpenFOAM** | Open-source FVM, CLI + case automation | via add-ons/interfaces | Medium-to-high if scripted well | Full control, custom pipelines, research |
| **SimScale (Simerics-MP in cloud)** | Commercial cloud CFD + API-driven flow | CAD import + API workflow orchestration | Medium; campaign style | Cloud-scale parametric sweeps, design review |
| **SU2** | Open-source, adjoint + optimization workflows | script-first | Medium; fast in steady design loops | Aero/shape optimization with gradient methods |
| **Ansys Workbench/CFD (scripted flow)** | Closed CFD + enterprise automation | Strong enterprise CAD/mesh pipelines | Moderate (license-dependent) | Teams with existing Ansys stack and design integration |
| **XFOIL** | 2D panel + viscous correction | limited | Very high | Fast airfoil screening only |
| **CFturbo + Simerics stack** | Parametric geometry + external CFD flowback | Tight with CFturbo UI | Moderate-to-low for direct loop | Pump/rotor design-space sweeps |

---

## Research and papers where Simerics is used

- Variable displacement vane pump with OCV + feedback coupling (SAE WCX 2025): paper abstract explicitly says flow is coupled with valve/controller behavior and uses Simerics-MP+ [[SAE 2025-01-8179]](https://saemobilus.sae.org/papers/a-3-d-cfd-performance-simulation-a-feedback-controlled-variable-displacement-vane-pump-2025-01-8179).
- Centrifugal pump system: comparison of transient vs MRF in Simerics MP+ [[SAE 2022-01-0780]](https://saemobilus.sae.org/papers/cfd-analysis-a-centrifugal-pump-controlling-a-vehicle-coolant-hydraulic-system-a-comparison-mrf-transient-approaches-2022-01-0780).
- DI diesel low-pressure pump: mesh+fast turnaround with PumpLinx workflow claims [[SAE 2017-01-2304]](https://saemobilus.sae.org/papers/3d-cfd-model-di-diesel-low-pressure-fuel-pump-system-2017-01-2304).
- Fully coupled dynamic flip valve compressor model where valve motion and flow are solved together [[ICEC Compressor Paper]](https://docs.lib.purdue.edu/icec/2471/).

The common pattern in these works is **transient coupled simulation + geometry-centric engineering metrics**, which is ideal for batched Bayesian/RL optimization but not direct per-step RL unless you reduce simulation fidelity.

---

## Practical next steps (if you want to use this for RL now)

1. Decide whether your policy outputs:
   - geometric parameters (blade/timing/clearances), or
   - control variables only (valve opening, speed setpoint, duty), or
   - both.
2. Decide whether each action should map to one full CFD run or to a precomputed map lookup + interpolator step.
3. Create a strict experiment schema where each sampled action writes one `.spro` and launches one bounded simulation job.
4. Normalize outputs to deterministic obs vector:
   - scalar map points, pressure/flow/efficiency, force/load time-averages, cavitation metrics
5. Start with vectorized external workers + caching, then use a model-based surrogate for inner-loop RL.

This gives you real-value research speed without waiting for a nonexistent hard real-time Simerics API.

---

## Related notes

- [[CFturbo]]
- [[PufferLib Flight Throughput Benchmark]]
- [[PufferLib Robotics Fit and Limits]]
- [[OpenFOAM]]
- [[SU2]]
- [[CFD]]
- [[Isaac Lab]]

---

## External resources

- [[Simerics Home]](https://www.simerics.com/)
- [[Simerics MP Cavitation]](https://www.simerics.com/mp-cavitation/)
- [[Automated Mesher]](https://www.simerics.com/automated-mesher/)
- [[CAEs and CAD Interfaces]](https://simerics.de/anwendungen/cae-interface/?lang=en)
- [[Simerics + GT-SUITE]](https://simerics.de/anwendungen/process-sumaliton/?lang=en)
- [[PumpLinx]](https://www.simerics.com/pumplinx/)
- [[AMD performance release]](https://www.simerics.com/wp-content/uploads/2026/04/Simerics-and-AMD-Announcement.pdf)
