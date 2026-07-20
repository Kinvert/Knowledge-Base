# BRL-CAD + Batch CSG Loop Workflow

BRL-CAD can be useful in strict batch contexts when geometry is maintained as CSG and command transcripts.

## 1) Canonical pattern

- Keep a stable `.g` geometry database as input.
- Build a per-run transcript of `mged` commands from param files.
- Run in batch mode and export with command tools.

## 2) Convert spreadsheet rows to run commands

`runner/build_transcript.py`

```python
import json
import openpyxl

wb = openpyxl.load_workbook("params/design_matrix.xlsx", data_only=True)
ws = wb["sweep"]

for i, row in enumerate(ws.iter_rows(min_row=2, values_only=True), start=1):
    angle = float(row[0]); rin = float(row[1]); rout = float(row[2])
    run_id = f"run_{i:04d}"
    cmd = [
        f"units mm",
        f"setparam bluff angle {angle}",
        f"setparam bluff rin {rin}",
        f"setparam bluff rout {rout}",
        f"g blender {run_id} bluff",   # conceptual pattern depending on command set
        "save",
    ]
    with open(f"jobs/{run_id}/commands.mged", "w") as f:
        f.write("\n".join(cmd))
```

## 3) Execute transcript in batch

```bash
mged -c base_model.g < jobs/run_001/commands.mged > jobs/run_001/mged.log
```

## 4) Export

After regeneration, call appropriate BRL-CAD export translators for mesh outputs (depending on installed toolchain in your environment).

## 5) Loop notes

- Treat transcript templates as code.
- Keep all `.g` inputs immutable for reproducibility.
- Fail run if the generated log contains geometry errors.

## 6) Why this is here

Not modern parametric CAD, but robust where legacy pipelines are already CSG-based.

## 7) References

- `mged` batch usage and options: https://brl-cad.github.io/docs/articles/mged/gui.html
- Legacy command docs and workflow notes: https://brl-cad.github.io/docs/wiki/MgedFAQ.html
