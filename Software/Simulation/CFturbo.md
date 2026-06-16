---
title: CFturbo
aliases:
  - CFturbo
  - CFturbo GmbH
tags:
  - simulation
  - turbomachinery
  - cfd
  - optimization
  - pufferlib
  - reinforcement-learning
  - engineering-simulation
---

# CFturbo

**CFturbo** is a commercial turbomachinery design stack (pumps, fans, compressors, turbines) with geometry generation, reverse engineering, simulation interfaces, and optimization workflows. It is not a real-time flight simulator.

---

## Core Facts (quick)

| Item | Evidence |
|---|---|
| Primary class | Turbomachinery CAD/geometry + workflow automation software, not a flight sim | [[CFturbo Main Functions]](https://cfturbo.com/software/main-functions) |
| OS | Windows 11 (x86-64), .NET 4.0 is listed in the download requirements | [[CFturbo Download]](https://cfturbo.com/software/download) |
| User-facing modules | Pumps, Fan & Blower, Compressor, Turbine, Reverse Engineering, BLADERUNNER, SMP CFD, FEA | [[CFturbo Main Functions]](https://cfturbo.com/software/main-functions) |
| Automation | Full batch mode + XML (`.cft-batch`) with `-batch`, update/output blocks, and process exit codes | [[Batch mode manual]](https://manual.cfturbo.com/en/batch_mode.html), [[Exit codes]](https://manual.cfturbo.com/en/exit_codes.html) |
| Integration | Exposes many CAD/CFD/FEA export interfaces and links (Ansys/TurboGrid/ICEM/Star-CCM+/Simerics etc.) | [[Interfaces and Workflows]](https://cfturbo.com/software/interfaces-workflows), [[Export interfaces]](https://manual.cfturbo.com/en/batch_mode.html) |
| Flight-relevant | CFturbo BLADERUNNER does not have its own CFD solver; it drives simulations from external CFD engines (currently CFturbo SMP only) | [[CFturbo BLADERUNNER]](https://cfturbo.com/software/cfturbo-bladerunner) |

---

## What CFturbo is, and what it is not

For your RL work this matters a lot:

- It is useful if your project is about **design-optimization loops** for turbomachinery, where policy outputs are design variables and reward is map/efficiency metrics.
- It is **not** a plug-and-play closed-loop vehicle/flight simulator with low-latency control APIs.
- It is strongest as an **offline high-fidelity optimization engine**, then optionally used in a broader learning loop.

---

## Major modules (short technical profile)

### Pump / Fan / Compressor / Turbine modules

All four modules share a common parametric approach:

- Fully parametric geometry with component libraries (impellers, volutes, stators)
- Specific-speed coverage around `8 < n_q < 500` (EU) and `400 < N_s < 22,000` (US)
- Export of models to CFD/FEA/CAD workflows for downstream simulation
- Focus on pump/ventilator/compressor/turbine design variants

Sources:
- [[Pumps]](https://cfturbo.com/software/pumps)
- [[Fans & Blowers]](https://cfturbo.com/software/fans-blowers)
- [[Compressors]](https://cfturbo.com/software/compressors)
- [[Turbines]](https://cfturbo.com/software/turbines)

### Reverse engineering path

CFturbo supports conversion of existing CAD into parametric CFturbo models for fast redesign/optimization (STEP/IGES/Parasolid/BREP import, impeller + stator support). This is relevant if you have legacy geometry but want controllable design parameters for RL/data generation.

Source: [[Reverse Engineering]](https://cfturbo.com/software/reverse-engineering)

### BLADERUNNER and SMP

- **BLADERUNNER** is a turbomachinery CFD workflow manager: run many variants, specify mesh/solver, create performance maps/curves, compare variants.
- It depends on **CFturbo SMP** as the CFD solver path currently.

Sources:
- [[CFturbo BLADERUNNER]](https://cfturbo.com/software/cfturbo-bladerunner)
- [[CFturbo SMP]](https://cfturbo.com/software/cfturbo-smp)

### Simerics path (what you asked)

If you mean **Simerics**:

- CFturbo SMP is explicitly described as a CFD implementation **based on Simerics MP**. That is the primary internal solver relation today. [[CFturbo SMP]](https://cfturbo.com/software/cfturbo-smp)
- The CFturbo ↔ Simerics collaboration covers interfaces to Simerics virtual testing tools **PumpLinx** and **Simerics MP**, used in the turbomachinery design→analysis loop. [[Simerics + CFturbo]](https://www.simerics.com/simerics-and-cfturbo-sign-collaborative-agreement/)
- In CFturbo’s export workflow, the Simerics/`simericMP` path is STL + `.spro` project file based, with meshing + simulation parameters prepared in CFturbo and launched for automation. [[Interfaces and Workflows]](https://cfturbo.com/software/interfaces-workflows), [[Simerics export]](https://manual.cfturbo.com/en/simerics.html)
- Simerics is a broader CFD/virtual-testing vendor (MP/MP+) for positive displacement devices, valves, vehicle/marine/turbomachinery plus 3D coupling workflows; in their material they pitch it as a “complete process for design, analysis, and simulation.” [[Simerics]](https://www.simerics.com/simerics-and-cfturbo-sign-collaborative-agreement/)

### FEA add-on

CFturbo also ships CFturbo FEA (static + deformation + modal) as a GUI front-end to open-source CalculiX, free for active CFturbo users.

Source: [[CFturbo FEA]](https://cfturbo.com/software/cfturbo-fea)

---

## Automation and control surface (important for RL harness design)

### 1) Batch mode (CLI-first)

CFturbo can run without GUI:

```text
cfturbo.exe -batch <batch file> [-verbose] [-export <interface>] [-log <log file>]
```

`<batch file>` is `.cft-batch` (XML) and can include updates + actions (export/save) for multiple projects.

The batch file defines selected parameter nodes, and results are written to `.cft-res` after runs for downstream tooling. Batch exit codes:
- `0` success
- `1` completed with warnings
- `2` errors

Sources:
- [[Batch mode]](https://manual.cfturbo.com/en/batch_mode.html)
- [[Exit codes]](https://manual.cfturbo.com/en/exit_codes.html)

### 2) Interactive mode

You can also launch a project directly:

- `cfturbo.exe [project.cft [-readonly]]`
- `cfturbo.exe <batch-file>`

`-readonly` loads project data for inspection without edit privileges.

Source: [[Interactive mode]](https://manual.cfturbo.com/en/interactive_mode.html)

### 3) Export interface breadth

The manual lists export interfaces including ANSYS/CFD pipelines (STEP, AutoGrid, MeshPro, ICEM-CFD, OpenFOAM, Star-CCM+, SimericsMP, CFD-Pre, PerformanceData, etc.).

This is where practical RL plumbing starts: geometry + operating-state metadata can be exported directly into solver workflows.

Source: [[Batch mode export interfaces]](https://manual.cfturbo.com/en/batch_mode.html)

---

## Licensing and deployment constraints

- CFturbo supports viewer mode for read-only access; changing designs requires a valid license.
- Licensing includes node-locked, floating, and cloud options via RLM.
- Silent install command exists (`/SILENT /VERYSILENT /SUPPRESSMSGBOXES`) and uninstall command equivalents.
- Running on Windows 11 + x86-64 (AVX2) is explicitly listed on the download page; Linux support is not native for core app.

Sources:
- [[Licensing]](https://manual.cfturbo.com/en/license.html)
- [[Silent mode install]](https://manual.cfturbo.com/en/installation-silent.html)
- [[CFturbo download/system requirements]](https://cfturbo.com/software/download)
- CFturbo2ICEM scripts show mixed OS tooling (Windows installer, Linux manual file-copy usage for script integration): [[CFturbo2ICEM]](https://cfturbo.com/software/interfaces-workflows/script-solution-for-ansys-icem-cfd)

---

## PufferLib relevance and practical harness idea

If your objective is realistic RL, CFturbo is best treated as an **outer-loop environment generator**, not the inner-time-step sim itself.

### Recommended pattern

1. **Policy outputs design/action parameters** (blade count, widths, wrap angles, incidence constraints, etc.).
2. **Write/update `.cft-batch` parameters** from policy.
3. **Execute CFturbo batch** headlessly.
4. **Parse `.cft-res`, exported geometry/mesh/CFD outputs** into reward terms.
5. **Return compact metrics** (efficiency, surge margin proxies, power ratio, pressure rise, cavitation margins, map location) to Puffer step.

### Fit profile

- **High SPS RL in-step control loop**: weak/poor fit. CFturbo and linked solvers are heavier than typical policy environments.
- **Batch design optimization RL (black-box / BO / CMA / RL in parameter space)**: strong fit, because batch mode + deterministic file contracts are explicit and scriptable.

This fits better with approaches like:
- evolution strategies over design parameters
- model-based RL over design policy
- hybrid optimization pipelines with external reward evaluators

No public `step/reset` API was found in the official manual/docs set; there is no clearly documented Python SDK for direct sim-time state stepping. Treat the toolchain as command+filesystem integration.

Related pattern in your KB:
- [[PufferLib Flight Throughput Benchmark]]
- [[PufferLib Robotics Fit and Limits]]

---

## Comparison snapshot (design-stack context)

The table below is about **design-space RL workflows** (geometry/parameters -> CFD metric) rather than control-loop flight RL.

| Platform | Primary loop type | API/automation model | Parallelization path | Why/when to use for RL |
|---|---|---|---|---|
| CFturbo + BLADERUNNER | Parametric geometry + external CFD workflow | `.cft-batch` XML + CLI + exports | Multiple isolated workers each running batch jobs and downstream solvers | Strong for closed-form design optimization, weak for high-SPS inner-loop control |
| [[OpenFOAM]] (with scripts) | Full CFD solve per geometry variant | Command-line case directories + Python/bash orchestration | HPC parallelism per case + distributed parameter sweeps | Strong when you need open-source solver flexibility and deep customization |
| [[SU2]] | Aerodynamic/optimization solving and adjoint support | CLI + Python API/config files | Embarrassingly parallel campaigns and adjoint-based batches | Good for aero shape optimization and gradient-aware search |
| [[XFOIL]] | 2D section analysis + polar generation | Non-interactive command input + batch pipelines | Very cheap batch sweeps | Best as fast pre-filter stage before higher-fidelity 3D CFD |
| In-house scripted CFD stack (OpenFOAM/OpenVSP + Python) | End-to-end geometry/mesh/run/reward pipeline | Python/C++ wrappers + case automation | Easy to cluster and custom schedule; build-your-own orchestration | Best when you need full control, custom state/reward definitions, and no vendor lock-in |
| MATLAB/Simulink + custom aero blocks | Reduced-order modeling + design sweeps | Block-diagram API + script/CI automation | Limited unless cloud or distributed MATLAB workers are available | Useful for fast prototyping, often lower fidelity than full 3D CFD |

## Practical integration sketch

```python
def step(action):
    write_batch_updates("cfturbo_run.cft-batch", action)
    subprocess.run(
        [
            "cfturbo.exe",
            "-batch",
            "cfturbo_run.cft-batch",
            "-verbose",
            "-export",
            "General",
        ],
        check=True,
    )
    metrics = parse_cft_res("cfturbo_run.cft-res")
    obs = {
        "efficiency": metrics.get("efficiency"),
        "pressure_ratio": metrics.get("pressure_ratio"),
        "power": metrics.get("shaft_power"),
    }
    reward = reward_fn(metrics)
    done = termination_condition(metrics)
    return obs, reward, done, {}
```

For CFturbo, this pattern is usually the only practical route:
- encode RL action bounds into `.cft-batch` updates
- run command-line `cfturbo.exe -batch`
- parse `.cft-res` and exported outputs for observation/reward

---

## Gaps / checks before committing real experiments

- Confirm whether your RL objective is **control policy** or **geometry/design policy**.
- Decide if you want **single-machine batch** or **cluster job scheduling** for solver saturation.
- Confirm available external solvers: BLADERUNNER currently connects to CFturbo SMP only.
- Validate licensing model for number of concurrent workers (floating/cloud likely required for swarm runs).

---

## Sources

- [[CFturbo Main Functions]](https://cfturbo.com/software/main-functions)
- [[CFturbo Download]](https://cfturbo.com/software/download)
- [[CFturbo Interfaces and Workflows]](https://cfturbo.com/software/interfaces-workflows)
- [[CFturbo Optimization]](https://cfturbo.com/software/optimization)
- [[CFturbo BLADERUNNER]](https://cfturbo.com/software/cfturbo-bladerunner)
- [[CFturbo SMP]](https://cfturbo.com/software/cfturbo-smp)
- [[CFturbo FEA]](https://cfturbo.com/software/cfturbo-fea)
- [[CFturbo Reverse Engineering]](https://cfturbo.com/software/reverse-engineering)
- [[CFturbo Pumps]](https://cfturbo.com/software/pumps)
- [[CFturbo Fans & Blowers]](https://cfturbo.com/software/fans-blowers)
- [[CFturbo Compressors]](https://cfturbo.com/software/compressors)
- [[CFturbo Turbines]](https://cfturbo.com/software/turbines)
- [[Manual: batch mode]](https://manual.cfturbo.com/en/batch_mode.html)
- [[Manual: exit codes]](https://manual.cfturbo.com/en/exit_codes.html)
- [[Manual: interactive mode]](https://manual.cfturbo.com/en/interactive_mode.html)
- [[Manual: batch example]](https://manual.cfturbo.com/en/batch_examples.html)
- [[Manual: licensing]](https://manual.cfturbo.com/en/license.html)
- [[Manual: silent install]](https://manual.cfturbo.com/en/installation-silent.html)
- [[Ansys Workbench extension]](https://cfturbo.com/software/interfaces-workflows/extension-for-ansys-workbench)
- [[CFturbo2ICEM]](https://cfturbo.com/software/interfaces-workflows/script-solution-for-ansys-icem-cfd)
- [[Manual PDF snapshot]](https://manual.cfturbo.com/CFturbo_en.pdf)
