# SIMPLE

`SIMPLE` is the **Semi-Implicit Method for Pressure Linked Equations**, OpenFOAM’s default steady pressure-velocity coupling pattern for many incompressible runs.

In practice it is the right choice when your goal is high-throughput steady screening and you want predictable runtime while keeping pressure-velocity coupling explicit and stable.

---

## Core mechanics

- Solve momentum equations with a guessed pressure field.
- Build a pressure correction equation from mass conservation.
- Solve pressure, then correct velocity and fluxes.
- Repeat through under-relaxed outer loops until residuals converge.

`simpleFoam` is the canonical OpenFOAM implementation of this pattern.

--- 

## What SIMPLE gives in external single-phase geometry loops

For geometry-to-CFD loops where unsteady shedding is not the immediate KPI driver, SIMPLE is usually the cheapest high-coverage rung:
- fast candidate ranking,
- low run-to-run complexity,
- stable automation behavior under moderate mesh quality.

That is why it is frequently paired with pseudo-transient checks and then escalated when unsteadiness dominates metrics.

--- 

## Controls and where people waste time

### High-leverage `fvSolution` patterns

- `SIMPLE` dictionary entries like relaxation factors and non-orthogonal correctors.
- tight/loose `residualControl` for stable early triage.
- `consistent yes;`/`no` choice depending on coupling sensitivity.

### Common tuning mistakes
- over-relaxing pressure/velocity simultaneously,
- switching to very strict residual targets before mesh/base-model consistency is fixed,
- using transient-style time-step constraints while still running true steady SIMPLE.

--- 

## Comparison chart (SIMPLE against adjacent workflows)

| Choice | Primary coupling mode | Time behavior | Speed per sample | Unsteady fidelity | Typical reason to choose |
|---|---|---|---:|---:|---|
| `simpleFoam` (`SIMPLE`) | pressure correction loop with one outer global cycle | steady | High | Low | fastest broad screening |
| `pisoFoam` (`PISO`) | transient pressure correction loops in each step | transient | Medium | High (short-time) | wake/shakeup and startup physics |
| `pimpleFoam` (`PIMPLE`) | outer+inner loops (hybrid) | transient | Medium | Higher | stronger coupling under stiffness |
| `rhoSimpleFoam` (`SIMPLE` + variable density) | pressure-based compressible | steady/transient | Medium | Medium | temperature-density effects matter |
| `potentialFoam` + `potential` warm start | irrotational initialization before full momentum solve | initialization only | Very high | Low | fast starting field generation |
| `steady potential` + SIMPLE sequence | staged hybrid (potential then SIMPLE) | staged | Very high | Low to medium | reliable warm starts on large design sets |

For our use case, SIMPLE should be first except when transients are clearly driving force/pressure metrics.

--- 

## AI-loop decision rule

- Use **SIMPLE** first on every candidate.
- Escalate to PISO only when unsteadiness shifts KPI ranking.
- Escalate again to PIMPLE if transients are present but still too stiff for simple PISO settings.
- Keep SIMPLE runs versioned with fixed `turbulenceModels` and mesh-size family for comparability.

--- 

## Source links

- OpenFOAM solver algorithm index (pressure-velocity context): https://doc.openfoam.com/2212/tools/processing/solvers/pressure-velocity/
- OpenFOAM pressure-velocity algorithms page with SIMPLE sequence: https://doc.openfoam.com/2212/tools/processing/solvers/pressure-velocity/simple/
- OpenFOAM quickstart (mentions SOLVE-ALGORITHMS including SIMPLE/PISO/PIMPLE): https://doc.openfoam.com/2606/quickstart/
- OpenFOAM `simpleFoam` reference page: https://doc.openfoam.com/2306/tools/processing/solvers/rtm/incompressible/simpleFoam/
- OpenFOAM user guide section on solution control (SIMPLE/PISO/PIMPLE): https://www.openfoam.com/documentation/user-guide/6-solving/6.3-solution-and-algorithm-control
- OpenFOAM standard solver catalog: https://www.openfoam.com/documentation/user-guide/a-reference/a.1-standard-solvers
- Source path (OpenFOAM tree): `$FOAM_SOLVERS/incompressible/simpleFoam`  
  Historical code reference: https://github.com/OpenFOAM/OpenFOAM-11/blob/master/applications/solvers/incompressible/simpleFoam/simpleFoam.C

--- 

## Related notes

- [[simpleFoam]]
- [[Pressure-velocity coupling]]
- [[PISO]]
- [[PIMPLE]]
- [[CFD Solvers]]
- [[potentialFoam]]
