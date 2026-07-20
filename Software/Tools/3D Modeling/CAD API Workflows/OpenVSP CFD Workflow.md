# OpenVSP CFD Workflow

OpenVSP is niche, but strong for aerospace-like parametric families.

## 1) What to model in script mode

Use a script file containing geometry generation from numeric parameters. In practice this usually means using `.vspscript` or equivalent API call paths described in OpenVSP docs.

`designs/bluff.vspscript` (conceptual skeleton):

```c
// Pseudocode-like script structure used by OpenVSP CLI automation
SetActiveView(0,0,0);
SetVehicleName("bluff");
SetDiffuserAngle(${diffuser_angle_deg});
SetInletRadius(${inlet_radius_m});
SetOutletRadius(${outlet_radius_m});
Update();
WriteVSPFile("jobs/${run_id}/bluff.vsp3");
```

## 2) Parameter file generation

Generate JSON and replace placeholders inside `.vspscript` with a template engine.

```python
import json
import openpyxl

from pathlib import Path
tpl = Path("designs/bluff.vspscript.in").read_text()
wb = openpyxl.load_workbook("params/design_matrix.xlsx", data_only=True)
ws = wb["sweep"]

for i, row in enumerate(ws.iter_rows(min_row=2, values_only=True), start=1):
    context = {
      "diffuser_angle_deg": row[0],
      "inlet_radius_m": row[1],
      "outlet_radius_m": row[2],
      "run_id": f"run_{i:04d}",
    }
    script = tpl
    for k, v in context.items():
        script = script.replace(f"${{{k}}}", str(v))
    Path(f"jobs/run_{i:04d}/bluff.vspscript").write_text(script)
```

## 3) Run headless script

```bash
mkdir -p jobs/run_001
vspscript jobs/run_001/bluff.vspscript
```

## 4) Optional: convert to STEP for meshing

Most OpenVSP loops use a translation/export step to STEP/STL after geometry generation (command depends on build/version; validate once for your install).

## 5) Why/when this workflow

- High fit when geometry is aerodynamic/body families.
- Lower fit for mechanical parts with complex parametric constraints and fillets.

## 6) References

- API docs and scripting entry points: https://openvsp.org/api_docs/3.43.1/
