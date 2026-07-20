# SolveSpace + Agentic Sweep Workflow

Use SolveSpace when model complexity is moderate and you want CLI-driven regeneration/export.

## 1) Strategy

SolveSpace CLI is command-oriented; for true parametric sweeps it usually needs a pre-generated `.slvs` template or generated sketch constraints.

`params` from sheet should be converted into either:
- a prepared `.slvs` family (different files), or
- a scripted edit stage in an upstream process.

## 2) Example run loop

```bash
solvespace-cli export-mesh \
  --export-format stl \
  --quiet \
  --output jobs/run_001/bluff.stl \
  models/bluff_template.slvs
```

Depending on your installed version, commands like `export-view`, `export-wireframe`, and `thumbnail` can be used for alternate outputs.

## 3) Converting `xlsx` to batch-specific `.slvs` (recommended helper)

Because direct param replacement is not the core path, generate `.slvs` copies:

```python
import json
import openpyxl
from pathlib import Path

tpl = Path("templates/bluff_template.slvs").read_bytes()
wb = openpyxl.load_workbook("params/design_matrix.xlsx", data_only=True)
ws = wb["sweep"]

for i, row in enumerate(ws.iter_rows(min_row=2, values_only=True), start=1):
  run_id = f"run_{i:04d}"
  Path(f"models/{run_id}").mkdir(parents=True, exist_ok=True)
  Path(f"models/{run_id}/bluff.slvs").write_bytes(tpl)
  with open(f"jobs/{run_id}/params.json", "w") as f:
      json.dump({"diffuser_angle_deg": row[0], "inlet_radius": row[1], "outlet_radius": row[2]}, f, indent=2)
```

At minimum, keep the mapping from each `.slvs` to parameter file explicit.

## 4) Reliability tips

- Export all generated STL plus geometry logs.
- Keep one template and avoid hand-editing binary `.slvs` without regeneration scripts.
- If `solvespace-cli` outputs fail silently, run with explicit logging and compare file timestamps.

## 5) References

- https://manpages.debian.org/testing/solvespace/solvespace-cli.1.en.html
