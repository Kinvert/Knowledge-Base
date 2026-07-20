# CAD APIs for CFD-Driven Parametric Design Loops

## Why this exists

This note is about running geometry generation in a loop:

`params (json/xlsx)` → `CAD build` → `STEP/mesh` → `snappyHexMesh/CFD` → `objective` → `next params`.

If CAD can’t run headless and parse stable parameter files, optimization becomes manual and slow.

The goal is a practical ranking for human-in-the-loop CFD studies where an agent is allowed to call commands and modify parameter files.

---

## What makes a CAD package useful for agentic loops

- Stable API or scripting path (not GUI-only macros).
- Non-interactive CLI/headless launch.
- Deterministic parameter input from text files.
- Export of CAD-neutral formats (preferably STEP + STL/mesh).
- Easy process return codes/logging for failure handling.
- No hard coupling to desktop licensing workflows.

For this note, “API” includes:

- Native Python/C++ APIs.
- Dedicated script language with CLI.
- CLI flags for parameters or scripts.
- REST APIs (usually cloud-hosted).

---

## At-a-glance comparison (CFD-loop suitability)

| Tool | License / Price | Automation Surface | CLI / Headless | Parameter Injection | Native Exports | Best Notes for CFD Loops |
|---|---|---|---|---|---|---|
| FreeCAD | Open source, free | Python modules + macro system | `freecadcmd` (headless) | script reads JSON/CSV/XLSX | STEP, STL, etc. via script and GUI modules | Best broad “human CAD + agent batch” hybrid |
| CadQuery | Open source, free | Python API + CQGI | `cq-cli` | `--params` JSON/file or string | STEP, STL, BREP via codecs | Best pure-code parametric geometry |
| OpenSCAD | Open source, free | language + script runtime | `openscad` command-line | `-D` and customizer presets (`-p`, `-P`) | STL/Binary mesh; STEP via translation chain | Best deterministic math-style geometry |
| SALOME | Open source, free | Python module + application scripting | `salome start`, `salome shell` + script args | script args parsing + file-based params | STEP + mesh formats + advanced postprocess | Best if you already use Salome for meshing/ops |
| OpenVSP | Open source, free | API + script + command mode | CLI script runner (`vspscript`) | scripts can consume variables | STEP + STL depending on script/export path | Excellent for aero-specific parametric families |
| SolveSpace | Open source, free | Command verbs (`solvespace-cli`) | headless CLI | script/template workflow | STL and 2D outputs | Good for small-to-medium geometry blocks |
| BRL-CAD | Open source, free | CSG command language (`mged`) | `mged` batch mode | text transcripts | export commands + translators | Legacy but stable for scripted CSG |
| Gmsh | Open source, free | `.geo` + Python API | `gmsh` CLI | `-setnumber`, `-setstring`, Python injection | mesh exports (msh, vtk, etc.), geometry import/export helpers | Not full parametric CAD, but strong geometry+mesh loop tool |
| Blender (mesh path) | Open source, free | `bpy` Python API | background CLI (`--background`) + Python | script-level JSON | STL/OBJ/3D mesh formats | Good mesh pre/post step, weak native CAD tree |
| Onshape | Proprietary, paid cloud plans | REST API | no local CLI in product | API call payloads + external orchestration | STEP + CAD-neutral via API endpoints | Strong for enterprises already using Onshape |

Sources for claims:

- FreeCAD API + docs and headless command reference:  
  https://freecad.github.io/API/modules.html,  
  https://github.com/FreeCAD/FreeCAD-documentation/blob/main/wiki/Start_up_and_Configuration.md,  
  https://manpages.debian.org/unstable/freecad/freecadcmd.1.en.html
- OpenSCAD CLI/customizer references:  
  https://manpages.debian.org/testing/openscad/openscad.1.en.html,  
  https://en.wikibooks.org/wiki/OpenSCAD_User_Manual/Customizer
- CadQuery API and CLI project:  
  https://cadquery.readthedocs.io/en/stable/cqgi.html,  
  https://pypi.org/project/cadquery-cli/
- SALOME command docs:  
  https://docs.salome-platform.org/latest/tui/KERNEL/salome_command.html,  
  https://docs.salome-platform.org/latest/tui/KERNEL/running_salome_page.html
- OpenVSP API docs:  
  https://openvsp.org/api_docs/3.43.1/
- SolveSpace CLI docs:  
  https://manpages.debian.org/testing/solvespace/solvespace-cli.1.en.html
- BRL-CAD scripting references:  
  https://brl-cad.github.io/docs/articles/mged/gui.html,  
  https://brl-cad.github.io/docs/wiki/MgedFAQ.html
- Gmsh docs:  
  https://gmsh.info/doc/texinfo/gmsh.html
- Onshape API + auth + pricing:  
  https://onshape-public.github.io/docs/api-intro/,  
  https://onshape-public.github.io/docs/auth/apikeys/,  
  https://www.onshape.com/en/pricing

---

## Recommended stack pattern for CFD optimization

For a human-guided optimization loop on a bluff-body diffuser angle family, the highest-priority stack is typically:

1. **CadQuery** for pure headless geometry generation and validation.
2. **OpenSCAD** for compact, deterministic parameter expressions.
3. **FreeCAD** when human designers need direct GUI touchpoints.
4. **SALOME** when you want geometry, cleanup, meshing, and field prep in one framework.
5. **Gmsh** for scripted meshing if you need very tight control over mesh controls in the same loop.

Use this order if your loop is local and speed-focused; switch to **Onshape** if your CAD governance is already cloud-first and API credentials are already maintained.

---

## FreeCAD (Open Source): best general-purpose pick

Why this is strong:

- Python API covers many domains including sketches, Part features, Boolean ops, and data export.
- `freecadcmd` exists explicitly as headless mode.
- Human can maintain `.FCStd` template and script around param sets.

Where it gets tricky:

- Startup overhead is heavier than OpenSCAD/CadQuery.
- Headless behavior can differ between environments with missing GUI plugins.
- You need explicit unit discipline and constraints if scripts come from spreadsheet-driven values.

What to automate:

- Parse xlsx/JSON into a normalized run dictionary.
- Run a template macro that builds from names like `diffuser_angle_deg`, `flange_radius_m`, etc.
- Export STEP and STL in the same script after each run.
- Fail fast if geometric validation checks fail (mass, bounding box, non-manifold checks).

Useful links:

- API modules: https://freecad.github.io/API/modules.html
- Startup/CLI options: https://github.com/FreeCAD/FreeCAD-documentation/blob/main/wiki/Start_up_and_Configuration.md
- Headless CLI reference: https://manpages.debian.org/unstable/freecad/freecadcmd.1.en.html

---

## CadQuery + cq-cli: strongest API-first automation profile

Why this is strong:

- Python-native modeling, designed for scripting.
- `cq-cli` supports JSON parameter ingestion and parameter-schema extraction.
- `--codec step` style exports integrate well with solvers and meshers.

Strength / caveat:

- Very strong for reproducibility.
- Requires coding model generation from scratch (less GUI convenience than FreeCAD).
- Great for controlled parametric families and CI-style sweep control.

CLI points:

- `--codec` for output format
- `--params` for JSON values
- `--getparams` to print schema before loops
- `--validate` to catch bad input early

Useful links:

- CQGI API (for parameterized execution): https://cadquery.readthedocs.io/en/stable/cqgi.html
- `cadquery-cli` package docs and options: https://pypi.org/project/cadquery-cli/

---

## OpenSCAD: strongest deterministic text modeler

Why this is strong:

- The model is a text program first.
- CLI supports direct value overrides (`-D`) and preset sets.
- Very compact command loops (`for angle in ...`).

Tradeoffs:

- Not a true B-rep feature history tree like FreeCAD.
- STEP is not native primary output in older patterns; you usually route through a translation step.

API/CLI notes:

- `-D` for single values in shell loops.
- `-p` customizer JSON, `-P` preset name.
- Mesh outputs are robust and deterministic.

Useful links:

- CLI manual: https://manpages.debian.org/testing/openscad/openscad.1.en.html  
- Customizer: https://en.wikibooks.org/wiki/OpenSCAD_User_Manual/Customizer

---

## SALOME: one environment for parametric geometry + simulation prep

Why this is strong:

- It is Python-scriptable and supports CLI launching of scripts.
- Useful when you want geometric preprocessing and meshing in the same environment.
- Mature for engineering workflows (mesh, field exports, scriptability).

What can hurt:

- Bigger stack than needed for light geometry-only loops.
- You need more setup effort than CadQuery/OpenSCAD for tiny projects.

API/CLI notes:

- `salome start` and `salome shell` are documented for script execution.
- Script arguments can be passed after `args:` style tokens in command patterns.
- Good for pipeline continuity with downstream mesh steps.

Useful links:

- https://docs.salome-platform.org/latest/tui/KERNEL/salome_command.html
- https://docs.salome-platform.org/latest/tui/KERNEL/running_salome_page.html

---

## OpenVSP: niche but excellent for aero parameter studies

Why this is strong:

- Built for aircraft and aero body families (fuselage/nacelle/fairings etc.).
- Works with scripting for shape families and trade studies.
- Headless script workflows fit design-of-experiments loops when domain matches.

Watch out:

- It is narrower in geometric domains versus general-purpose part design.
- You may need conversion steps if your CFD importer has strict geometry expectations.

Useful links:

- API docs: https://openvsp.org/api_docs/3.43.1/
- CLI/scripting section includes command-oriented usage patterns and script support.

---

## SolveSpace: lightweight CLI with scripted export

Why this is useful:

- Open-source and CLI-first export pipeline for quick batch generation.
- Useful for constrained geometry tasks and fast iteration at moderate model complexity.

Limitations:

- Less ergonomic for very deep automation than CadQuery/OpenSCAD.
- Parametric inputs usually need external generation of `.slvs` or prebuilt templates.

Useful links:

- https://manpages.debian.org/testing/solvespace/solvespace-cli.1.en.html

---

## BRL-CAD: stable CSG batch scripting

Why this is useful:

- Very mature batch and transcript workflow.
- Can run command scripts in non-interactive mode.
- Good choice if you already own CSG templates and want predictable headless reruns.

Limitations:

- User experience is command-oriented rather than modern feature tree.
- Geometry style may feel old vs parametric kernels.

Useful links:

- https://brl-cad.github.io/docs/articles/mged/gui.html
- https://brl-cad.github.io/docs/wiki/MgedFAQ.html

---

## Blender (mesh-first automation path)

Use this when the loop is mesh-heavy:

- Has `bpy` Python API for full scriptability.
- Can run in background/CLI mode.
- Solid for CAD-ish geometry cleanup, custom mesh transformations, and procedural operations.

Important caveat:

- It is not a CAD feature-history kernel in the same sense as parametric mechanical CAD.
- Keep Blender later in stack unless you mainly need mesh operations or rapid shape scripting.

Useful links:

- Blender CLI syntax discussions: https://manpages.ubuntu.com/manpages/jammy/man1/blender.1.html  
- Blender docs (Python API entry): https://docs.blender.org/api/current/

---

## Gmsh: the honest “not a CAD kernel” option

Gmsh is often mistaken as CAD replacement. It is stronger as:

- scripted geometry and meshing through `.geo` language
- Python API + CLI-driven meshing
- very useful for loop-based mesh parameterization after CAD generation

Useful if:

- the shape family is geometry-light or already represented by primitives and boolean scripts.

Not useful if:

- you need full parametric feature modeling with constraints and design intent editing.

Useful links:

- https://gmsh.info/doc/texinfo/gmsh.html

---

## Onshape: strongest cloud API option, but cloud-first

Why Onshape matters:

- Modern REST API and API-first workflows.
- Strong enterprise permissions/versioning model.
- Good fit where teams already standardize on Onshape documents and governance.

Why it is harder for local agent loops:

- No native local CAD headless executable.
- You must authenticate and go through API calls each cycle.
- Rate limits and API quotas still apply.
- Public plan naming is usually `Free`, `Standard`, `Professional`, and `Enterprise`; seat and quota details vary by contract.

Useful links:

- API intro: https://onshape-public.github.io/docs/api-intro/
- API keys/auth: https://onshape-public.github.io/docs/auth/apikeys/
- Onshape pricing plans: https://www.onshape.com/en/pricing

---

## Human-to-agent workflow (the one that works)

Use this exact data path and naming:

1. Human creates **template model** in chosen CAD with clearly named parameters.
2. Human creates **schema** (param names, units, bounds) in a JSON Schema file.
3. Agent generates variation rows from `xlsx` or `json`.
4. Agent calls CAD CLI.
5. Export geometry as STEP/STL + basic checksums/metadata.
6. Agent runs mesher and CFD, returns residuals and KPIs.
7. Human reviews top candidates and optionally adds constraints.

This preserves both automation and accountability:

- `params.json` stays source of truth.
- Every run has hash-traceable output files.
- You can rerun a failing geometry exactly.

Folder layout that scales:

- `designs/` CAD templates
- `params/` run specs + xlsx snapshots
- `jobs/{run_id}/` generated CAD, mesh, logs, CFD results
- `artifacts/` for accepted geometries

---

## Concrete comparison by use-case

### Best when you need:

- **Maximum automation and low maintenance** → CadQuery
- **Human editing + scripting hybrid** → FreeCAD
- **High-volume deterministic sweeps** → OpenSCAD
- **Single-tool geometry→mesh pipeline** (pre/post/mesher) → SALOME
- **Aerospace parametric families** → OpenVSP

### Avoid if:

- You need pure local batch without any cloud auth yet still want enterprise PLM → Onshape can be cumbersome.
- You need full STEP-first parametric editing + strong GUI editing + huge ecosystem out of box → not free except with compromises.
- You need rich meshing controls + deterministic scripts → pair Gmsh with your CAD generation stage.

---

## Recommended "bluff-body diffuser angle" baseline stack

For your stated workflow (diffuser angle sweep), start with:

1. **CadQuery** for generation from param table.
2. **Gmsh** for local deterministic mesh refinement control.
3. Your existing OpenFOAM chain:
   - `blockMesh` for background control cells
   - `snappyHexMesh` for surface refinement
   - `paraFoam` for visualization

Then add **FreeCAD** as a visual QA stage:

- Every N runs (e.g. every 20), open top candidates in FreeCAD GUI for a human sanity check, while the loop still remains automated.

## Per-tool workflow notes in this vault

- [[FreeCAD CFD Workflow]]
- [[CadQuery CFD Workflow]]
- [[OpenSCAD CFD Workflow]]
- [[SALOME CFD Workflow]]
- [[OpenVSP CFD Workflow]]
- [[SolveSpace CFD Workflow]]
- [[BRL-CAD CFD Workflow]]
- [[Gmsh CFD Workflow]]
- [[Blender CFD Workflow]]
- [[Onshape CFD Workflow]]

---

## Per-tool price / access summary

The numbers and names below are intended for quick triage, not legal procurement advice. Always verify current pricing pages before committing procurement.

| Tool | Cost model | Practical note |
|---|---|---|
| FreeCAD | Free/Open source | No license cost; local-only operation |
| CadQuery | Free/Open source | Install via Python packaging; free |
| OpenSCAD | Free/Open source | Free |
| SALOME | Free/Open source | Free |
| OpenVSP | Free/Open source | Free |
| SolveSpace | Free/Open source | Free |
| BRL-CAD | Free/Open source | Free |
| Gmsh | Free/Open source | Free |
| Blender | Free/Open source | Free |
| Onshape | Proprietary with free/paid plans | Cloud auth + API key billing per seat/plan |

---

## Related notes to read next

- [[snappyHexMesh]]
- [[OpenFOAM]]
- [[Mesh Generation]]
- [[FreeCAD CFD Workflow]]
- [[CadQuery CFD Workflow]]
- [[OpenSCAD CFD Workflow]]
- [[SALOME CFD Workflow]]
- [[Gmsh CFD Workflow]]
