# simpleFoam

`simpleFoam` is OpenFOAM’s steady-state, incompressible pressure-based solver class using the SIMPLE algorithm.
It is the go-to baseline for repeated parametric studies where flow is not strongly time-dependent and geometry changes are moderate.

---

## What this actually solves

- Incompressible kinematic continuity:  `∇·U = 0`
- Incompressible momentum equation with optional turbulence source terms
- SIMPLE pressure-velocity coupling in each global iteration

The solver is designed around finite-volume transport equations and dictionary-driven configuration rather than a rigid GUI workflow.

From the official docs: `simpleFoam` is listed as incompressible, steady, and turbulence-capable, and uses the `SIMPLE` algorithm for coupling.

---

## Why this is useful for CAD-driven optimization loops

- Very stable for convergence-to-residual metrics loops when geometry is parameterized (e.g., diffuser half-angle).
- Deterministic and cheap to automate for hundreds of variants because each run is steady.
- Easy KPI extraction with field functions (drag, mass-flow, pressure loss).

Good loop pattern:

1. Generate geometry variant (e.g., diffuser angle = 20°, 22°, 24° …).
2. Mesh/update `blockMeshDict` or `snappyHexMesh` setup.
3. Run
   `simpleFoam -case runDir -parallel` (or serial in small cases).
4. Parse convergence + objective metrics.
5. Persist only winner candidates; iterate bounds via Bayesian/DOE agent.

For external single-phase incompressible projects this is usually the first solver rung because it is the least expensive pressure-corrected steady route.

---

## Expanded comparison context for external single-phase incompressible flow

`simpleFoam` is the default quasi-steady solver rung in this path:

- `simpleFoam` (steady, SIMPLE)
- `pisoFoam` (transient, PISO)
- `pimpleFoam` (transient hybrid, PIMPLE)
- `icoFoam` (transient laminar baseline)

Use this order unless unsteady forcing is explicitly part of the KPI.

---

## What it is best at (external single-phase)

- External wall-bounded sweeps where geometry changes dominate.
- Steady drag/lift ranking with consistent inlet/outlet and turbulence policy.
- Parametric shape studies where you need fast rejection of poor variants.
- Mesh-quality comparison at scale (the same run settings applied across families).

---

## What it cannot do well

- Not for strong transients (valve opening, blade-passing interactions).
- Not for strong acoustic/transient pressure waves or compressibility-driven effects.
- Can produce non-physical oscillatory behavior if turbulence model/time scales are inconsistent with coarse meshes.
- Mesh quality issues (`checkMesh`) still dominate failure rate.

This is not a weakness of `simpleFoam` itself so much as a boundary-condition/mesh/physics mismatch issue: if transients dominate, `pisoFoam`/`pimpleFoam` are the correct next steps.

---

## External flow setup specifics that matter most

For external single-phase runs, these usually control success:

- Velocity boundary quality and consistency at inlet.
- Outlet setup (outflow/pressure references) to avoid artificial recirculation.
- Wall layering quality (especially near leading edges and sharp corners).
- Under-relaxation values in `fvSolution`.
- `wallDist`-dependent wall function behavior for expected boundary layers.

If you are using automated CAD mutations, keep turbulence choices constant in the first pass and gate any model changes as a controlled second-stage experiment.

---

## Command-line and automation notes

`simpleFoam` runs with standard OpenFOAM CLI and accepts `-case`, `-parallel`, `-postProcess`, and `-dry-run` style flags (via the generic application help pattern).

Typical minimal loop:

```bash
simpleFoam -case ../cases/diffuser_A -parallel
simpleFoam -case ../cases/diffuser_A -postProcess -func "forces(name=drag)" -latestTime
```

Useful dictionary checks for stability:

- `system/fvSchemes` (discretization)
- `system/fvSolution` (`SIMPLE`, relTol, under-relaxation)
- `system/controlDict` (`startFrom`, `endTime`, write controls)
- `constant/turbulenceProperties`

Recommended automation schema:

1. `checkMesh` pass/fail gate.
2. `simpleFoam` run with bounded iteration schedule.
3. Residual/continuity extraction into canonical JSON.
4. KPI extraction from function objects.
5. Candidate triage with explicit hard reject criteria (divergence, continuity error growth, NaN fields).

---

## Comparison table for a bluff-body style use case

| Solver | Steady / transient | Compressible | Main algorithm | Best use in CAD loop | Typical risk |
|---|---|---|---|---|---|
| [[simpleFoam]] | Steady | No | SIMPLE | Fast angle sweep on fixed mass-flow/pressure setup | Under-relaxation sensitivity on rough meshes |
| [[pisoFoam]] | Transient | No | PISO | Unsteady inlet transients and startup effects | Can be slower and needs smaller `Δt` |
| [[pimpleFoam]] | Transient | No | PIMPLE | Moderate CFL transients with geometric sweeps | Less cheap per sample than SIMPLE |
| [[rhoSimpleFoam]] | Steady | Yes | SIMPLE | Gas/warm air loops where density changes matter | More model setup (thermophysical) |
| [[buoyantSimpleFoam]] | Steady | Compressible/buoyant | SIMPLE + buoyancy | Natural convection with heating/cooling | More complex property setup |
| [[OpenFOAM]] | Framework | Framework | Framework | Fully custom scripted loop | Requires solver selection discipline |

---

## Staged strategy: when to leave `simpleFoam`

1. Run all candidates with `simpleFoam` first.
2. Mark candidates with clear unsteady sensitivity markers:
   - persistent vortex-shedding suspected by residual noise/physical pattern,
   - KPI oscillation in post-run samples,
   - inlet forcing changes requiring phase-sensitive response.
3. Re-run top-ranked survivors with:
   - `pisoFoam` for minimal transient fidelity,
   - `pimpleFoam` if transient coupling is stiff or you need larger time steps.

---

## Practical tuning checklist (no code-level tuning myths)

- Tune under-relaxation only after discretization and BCs are stable.
- Increase mesh-independent residual consistency before touching exotic schemes.
- Keep turbulence model family fixed until geometry trends are stable, then run sensitivity pass.
- For very large parametric families, cap max iterations for cheap triage, then re-run finalists with stricter convergence.

This preserves throughput while still giving you an objective basis for escalation to more expensive solvers.

---

## How it fits in a managed workflow

If you need a fully closed loop:

- Keep case templates per solver family in version control.
- Parameterize only diffs between variants (`angle_deg`, `Re`, `turb_model`).
- Validate once with a trusted high-fidelity reference (e.g., [[pimpleFoam]] or [[interFoam]] if needed), then use [[simpleFoam]] as cheap screening only.

Recommended workflow topology:

- Template generation
- Geometry hash + mesh hash + solver hash
- Deterministic run envelope
- Structured metric logging
- Controlled escalation to transient solvers

---

## Known pitfalls and quick fixes

- Diverging pressure with stable velocity:
  - reduce pressure under-relaxation; tighten boundedness-limiting schemes.
- Slow convergence on acceptable geometry:
  - check wall treatment and fvSchemes compatibility before increasing iterations.
- Spurious recirculation in external wake:
  - confirm box domain extents and outlet boundary behavior before blaming numerics.
- Noisy KPI in a supposedly steady run:
  - switch to `pisoFoam` pilot cases to detect true unsteady behavior.

---

## Source-code and documentation traceability

If you want the same direct link style used for `pisoFoam`, `simpleFoam` can be traced as:

- OpenFOAM docs: https://doc.openfoam.com/2306/tools/processing/solvers/rtm/incompressible/simpleFoam/
- Code path shown in docs: `$FOAM_SOLVERS/incompressible/simpleFoam`
- GitHub mirror (historical but directly browsable): https://github.com/OpenFOAM/OpenFOAM-7/blob/master/applications/solvers/incompressible/simpleFoam/simpleFoam.C
- Generated API reference: https://api.openfoam.com/2506/simpleFoam_8C.html

## Related notes

- [[OpenFOAM]]
- turbulence models (RANS/LES)
- [[CFD]]
- [[snappyHexMesh]]
- [[pisoFoam]]
- [[pimpleFoam]]
- [[Ansys CFX]]

## External resources

- https://doc.openfoam.com/2306/tools/processing/solvers/rtm/incompressible/simpleFoam/
- https://doc.openfoam.com/2312/tools/processing/solvers/rtm/incompressible/simpleFoam/
- https://api.openfoam.com/2506/simpleFoam_8C.html
- https://doc.openfoam.com/2312/tools/processing/solvers/pressure-velocity/
