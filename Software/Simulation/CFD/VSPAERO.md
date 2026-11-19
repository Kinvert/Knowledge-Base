# VSPAERO

VSPAERO is NASA’s low- to mid-fidelity external aerodynamics solver that ships with OpenVSP. It is intended for rapid conceptual-level analysis rather than full near-wall 3D CFD.

---

## Overview

The solver supports:

- Vortex-lattice-based thin-surface analysis for lifting configurations
- Panel-method-based thick-surface analysis for finite bodies
- Hybrid cases (combining thin and thick representation)

In current OpenVSP workflows, VSPAERO is usually the first-pass engine for geometry trend screening before moving to higher-fidelity CFD.

---

## What it does well

- fast coefficient sweeps over geometry/angle/Mach design-space points
- control-surface influence studies and simple stability derivatives
- propulsion-airframe interactions via actuator-disk representations
- external solver coupling through the OpenVSP analysis/results APIs

Historically, the 2015 integration launch described VSPAERO as a thin-surface code with iterative wake coupling, iterative GMRES linear solve, and actuator-disk support; the 2019/6.0.0 line added fully unsteady, rotating-body support, noise coupling path (PSU-WOPWOP), and propeller hover support.

---

## Core model behavior

### VSPAERO modes

OpenVSP exposes VSPAERO analysis method enums:

- `VORTEX_LATTICE` (lifting surface approximation)
- `PANEL` (panel formulation for finite-thickness components)

Both remain potential-flow formulations, so expected strengths are fast runs and robust iteration; expected limitations are boundary-layer resolution and wall-modeling depth compared with RANS/LES solvers.

There is no clear, explicit GPU compute path documented for VSPAERO in current OpenVSP references; in practice, the solver is treated as a CPU workflow, with performance controls focused on CPU/threading and linear-solver choices.

### Output character

The solver writes case outputs per run, with result values and residual histories parseable in the VSPAERO Results Manager and exportable to CSV. Common interpreted outputs include force/moment coefficients and spanwise/load distributions, and unsteady case time-history plots when available.

---

## How it is used in OpenVSP

### GUI path

Typical GUI workflow is straightforward:

1. Build geometry in OpenVSP
2. Configure VSPAERO inputs (angles, Mach, conditions, method)
3. Run VSPAERO/Results Manager
4. Inspect `CD`, `CL`, moments, wake/pressure slices

### API path (preferred for loops)

Use `ExecAnalysis("VSPAEROComputeGeometry")` from the Analysis Manager when scripting/design loops.

Minimal API pattern:

```python
import openvsp as vsp

analysis = "VSPAEROComputeGeometry"
geom_id = vsp.AddGeom("WING")
vsp.SetVSPAERORefWingID(geom_id)

vsp.SetAnalysisInputDefaults(analysis)
# print/adjust parameters if needed:
# inputs = vsp.GetAnalysisInputNames(analysis)

res_id = vsp.ExecAnalysis(analysis)
vsp.PrintResultsDocs(res_id)
vsp.PrintResults(res_id)
```

Useful supporting API calls:

- `CheckForVSPAERO(path)` (solver availability check)
- `SetVSPAEROPath(path)` (solver/search path control)
- `SetVSPAERORefWingID(geom_id)` (forces reference geometry)
- `Get/Set*AnalysisInput` + `PrintAnalysisInputs/Docs`
- `PrintResults / Get*Results` for post-processing by script

The dedicated VSPAERO API group has only small compatibility wrappers now; most modern workflows use the analysis manager entry points.

---

## Where it fits in a practical stack

Use it when you need:

- fast geometry triage before expensive meshing
- repeatable design sweeps inside an OpenVSP optimization loop
- baseline force/moment trend validation

Avoid/stop and switch to full CFD when:

- stall, massive separation, or transition-sensitive drag is the target
- near-wall shear/friction prediction is required
- you need mesh-refinement evidence for certification-style confidence

---

## Comparison (5+ tools)

| Tool | Core model | Fidelity level | Typical runtime | Boundary-layer treatment | Best use |
|---|---|---|---|---|---|
| VSPAERO | VLM / panel (potential flow) | Low-mid | Very fast | Approx/integral options only | AOA/Mach/geometry sweep, ranking |
| [[XFOIL]] | 2D panel + BL | Low-mid | Very fast | 2D viscous boundary-layer in section | Section/polar generation |
| [[OpenFOAM]] | Finite-volume Navier–Stokes | High | Medium-high | Full near-wall models, LES/RANS options | Full 3D external/interior flow |
| [[SU2]] | Finite-volume Navier–Stokes + adjoints | High | Medium-high | Full boundary-layer workflows | Optimization loops needing gradients |
| [[OpenVSP CFD Workflow]] | Geometry + multiple analysis choices | Workflow dependent | Very fast / variable | Depends on downstream solver | Geometry-first CAD/analysis pipeline |
| [[Mesh Generation]] | Mesh preprocessor / geometry-to-grid | N/A | N/A | Downstream-dependent | Needed when moving past VSPAERO |

---

## Practical cautions

- OpenVSP release threads have repeatedly warned that major VSPAERO updates can change behavior; rerun validation studies after major/minor upgrades.
- Output filenames and run names matter: rerunning with same model name can overwrite previous VSPAERO result files.
- If you rely on automation, pin OpenVSP + VSPAERO versions for reproducibility.

---

## Related notes

- [[OpenVSP CFD Workflow]]
- [[OpenVSP Facade API]]
- [[CAD APIs for CFD-Driven Parametric Design Loops]]
- [[CFD]]

## Sources

- OpenVSP API docs (analysis manager + result manager): https://openvsp.org/api_docs/latest/group___analysis.html
- OpenVSP API docs (VSPAERO API group): https://openvsp.org/api_docs/latest/group___v_s_p_a_e_r_o.html
- OpenVSP API docs (VSPAERO methods enumeration): https://openvsp.org/api_docs/3.26.0/group___enumerations.html
- OpenVSP API Python docs: https://openvsp.org/pyapi_docs/latest/openvsp.html
- OpenVSP changelog: https://github.com/OpenVSP/OpenVSP/blob/main/CHANGELOG.md
- NASA OpenVSP/VSPAERO overview: https://www.nasa.gov/reference/openvsp-vspaero-basics/
- OpenVSP 3.1.0 release note: https://openvsp.org/blogs/announcements/2015/04/29/openvsp-3-1-0-and-vspaero-released
- OpenVSP 3.19.0 release note: https://openvsp.org/blogs/announcements/2019/11/09/openvsp-3-19-0-released
- OpenVSP 3.45.0 release note: https://openvsp.org/blogs/announcements/2025/07/19/openvsp-3-45-0-released
