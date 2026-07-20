# FreeCAD CFD Workflow

This workflow assumes a human creates a reusable FreeCAD template and parameter names are stable (`diffuser_angle_deg`, `outlet_radius_m`, etc.).

## 1) What this file is for

- Generate many bluff-body variants from one parameter set (JSON or XLSX).
- Export STEP and STL for meshing in Gmsh/OpenFOAM flows.
- Keep every run auditable with hashes and per-run metadata.

## 2) Project layout

```text
project/
  designs/
    bluff_template.FCStd
    make_bluff.py
  params/
    sweep.json
    runs/
      run_001.json
  jobs/
    run_001/
      bluff.step
      bluff.stl
      geometry.log
      run.json
```

## 3) Parameter schema (`params/schema.json`)

```json
{
  "type": "object",
  "required": ["diffuser_angle_deg", "inlet_radius_m", "outlet_radius_m"],
  "properties": {
    "diffuser_angle_deg": { "type": "number", "minimum": 10, "maximum": 50 },
    "inlet_radius_m": { "type": "number", "minimum": 0.05 },
    "outlet_radius_m": { "type": "number", "minimum": 0.05 }
  }
}
```

## 4) FreeCAD script (`designs/make_bluff.py`)

The script reads one JSON file and rebuilds a parametric object.

```python
import json, os
import FreeCAD as App
import Part
import Mesh

PARAM_PATH = os.environ.get("PARAM_PATH", "params/runs/run_001.json")
with open(PARAM_PATH, "r") as f:
    p = json.load(f)

angle = float(p["diffuser_angle_deg"])
rin = float(p["inlet_radius_m"])
rout = float(p["outlet_radius_m"])

doc = App.newDocument("bluff")
base = Part.makeCylinder(rin, 1.0, App.Vector(0, 0, 0))
tip = Part.makeCone(rin, rout, 1.0, App.Vector(0, 0, 1.0))
geo = base.fuse(tip)

out = doc.addObject("Part::Feature", "BluffBody")
out.Shape = geo
doc.recompute()
obj = App.ActiveDocument.addObject("Part::Feature", "Diffuser")
obj.Shape = geo
doc.recompute()

step_path = os.environ.get("STEP_PATH", "jobs/output/bluff.step")
stl_path = os.environ.get("STL_PATH", "jobs/output/bluff.stl")
Part.export([out], step_path)
Mesh.export([out], stl_path)
```

This is a minimal geometry example; replace it with your `bluff_template` constraints and feature logic.

## 5) Run commands (headless)

```bash
# one-off: convert xlsx -> params JSON before this
export PARAM_PATH="params/runs/run_001.json"
export STEP_PATH="jobs/run_001/bluff.step"
export STL_PATH="jobs/run_001/bluff.stl"
freecadcmd designs/make_bluff.py
```

`freecadcmd` is the documented non-GUI entrypoint for automated workflows.

## 6) Convert xlsx to params

Use a lightweight driver in your loop orchestrator:

```python
import json
import openpyxl

wb = openpyxl.load_workbook("params/design_matrix.xlsx", data_only=True)
ws = wb["sweep"]
for i, row in enumerate(ws.iter_rows(min_row=2, values_only=True), start=1):
    run = {"diffuser_angle_deg": row[0], "inlet_radius_m": row[1], "outlet_radius_m": row[2]}
    with open(f"params/runs/run_{i:04d}.json", "w") as f:
        json.dump(run, f, indent=2)
```

## 7) Recommended checks before mesh

- Non-zero mass/volume.
- Bounding box within expected extents.
- Manifold/solidity check where possible.
- Validate schema before starting FreeCAD generation.

If checks fail, do not queue meshing.

## 8) When to prefer this workflow

- Use FreeCAD here when domain knowledge lives in GUI-driven feature workflows.
- Use CadQuery if pure headless speed and repeatability dominate over GUI editing.

## 9) References

- FreeCAD API modules: https://freecad.github.io/API/modules.html
- Startup/CLI docs: https://github.com/FreeCAD/FreeCAD-documentation/blob/main/wiki/Start_up_and_Configuration.md
- `freecadcmd` man page: https://manpages.debian.org/unstable/freecad/freecadcmd.1.en.html
