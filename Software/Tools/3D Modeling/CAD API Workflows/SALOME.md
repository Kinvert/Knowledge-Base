# SALOME + Agentic Geometry-to-Mesh Workflow

Use SALOME when you want CAD generation and meshing in the same scripted toolset.

## 1) Run model from JSON using SALOME shell

`designs/build_bluff.py`

```python
import json
import sys
from salome.smesh import smeshBuilder
import salome
import GEOM

# SALOME passes script args after args:
raw = sys.argv[1:]  # e.g. [ "diffuser_angle_deg=32", "inlet_radius=0.22", ... ]
args = {}
for item in raw:
    if "=" in item:
        k, v = item.split("=", 1)
        args[k] = float(v)

angle = args.get("diffuser_angle_deg", 30)
rin = args.get("inlet_radius_m", 0.20)
rout = args.get("outlet_radius_m", 0.30)

import salome
salome.salome_init()
geompy = salome.geompy

cyl = geompy.MakeCylinderRH(rin, 1.0, geompy.MakeVectorDXDYDZ(0, 0, 1))
cone = geompy.MakeConeRH(rin * 1.05, rout, 0.8, geompy.MakeTranslation(geompy.MakeCylinderRH(1,1),0,0,1.0))
part = geompy.MakeFuse([cyl, cone], [1])[1]
geompy.addToStudy(part, "bluff_body")

smesh = smeshBuilder.New()
mesh = smesh.Mesh(part)
mesh.AutomaticHexahedralization()

out_step = f"jobs/{args.get('run_id','run')}/bluff.step"
out_mesh = f"jobs/{args.get('run_id','run')}/bluff.msh"
geompy.Export(part, out_step, "STEP")
mesh.ExportUNV(out_mesh)

print(f"OK {out_step} {out_mesh}")
```

## 2) Headless launch command

```bash
mkdir -p jobs/run_001
salome shell -- -t designs/build_bluff.py args:diffuser_angle_deg=32,inlet_radius_m=0.22,outlet_radius_m=0.30,run_id=run_001
```

Note: command syntax differs slightly by SALOME version; test once locally and keep your proven wrapper.

## 3) Convert xlsx to runs

```python
import json
import openpyxl

wb = openpyxl.load_workbook("params/design_matrix.xlsx", data_only=True)
ws = wb["sweep"]
for i, row in enumerate(ws.iter_rows(min_row=2, values_only=True), start=1):
    run = {
      "run_id": f"run_{i:04d}",
      "diffuser_angle_deg": row[0],
      "inlet_radius_m": row[1],
      "outlet_radius_m": row[2],
    }
    with open(f"params/runs/run_{i:04d}.json", "w") as f:
      json.dump(run, f, indent=2)
```

## 4) Best practice

- Keep `run_id` in every output path.
- Export geometry + at least one mesh-ready intermediary to avoid rerunning all CAD steps on failure.
- Keep mesh settings versioned separately (`mesh_settings.json`).

## 5) References

- SALOME command docs: https://docs.salome-platform.org/latest/tui/KERNEL/salome_command.html
- SALOME shell examples and arguments: https://docs.salome-platform.org/latest/tui/KERNEL/running_salome_page.html
