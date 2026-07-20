# CadQuery + cq-cli CFD Sweep Workflow

This is the most automation-friendly path when geometry definition can be fully expressed in Python code.

## 1) Install

```bash
pip install cadquery
pip install cq-cli
```

## 2) Param schema-first model pattern

In your CAD file, declare parameter defaults and optional typing assumptions so scripts can introspect expected inputs.

`designs/bluff.py`

```python
import cadquery as cq

def build(params):
    angle = float(params["diffuser_angle_deg"])
    rin = float(params["inlet_radius_m"])
    rout = float(params["outlet_radius_m"])

    body = (
        cq.Workplane("XY")
        .circle(rin)
        .extrude(0.2)
        .faces(">Z")
        .circle(rout)
        .extrude(0.5)
    )
    return body

def make(params):
    return build(params)
```

`params` can come from `--params` JSON in `cq-cli`.

## 3) Generate params from Excel

```python
import json
import openpyxl

wb = openpyxl.load_workbook("params/design_matrix.xlsx", data_only=True)
ws = wb["sweep"]
for row_i, row in enumerate(ws.iter_rows(min_row=2, values_only=True), start=1):
    params = {
        "diffuser_angle_deg": float(row[0]),
        "inlet_radius_m": float(row[1]),
        "outlet_radius_m": float(row[2]),
    }
    with open(f"params/runs/run_{row_i:04d}.json", "w") as f:
        json.dump(params, f, indent=2)
```

## 4) Run per variation

```bash
rundir=jobs/run_001
mkdir -p "$rundir"
cq-cli --infile designs/bluff.py --codec step --outfile "$rundir/bluff.step" --params params/runs/run_001.json
```

Optional schema validation:

```bash
cq-cli --infile designs/bluff.py --getparams params/schema.json
cq-cli --infile designs/bluff.py --validate --params params/runs/run_001.json
```

Then generate STL:

```bash
cq-cli --infile designs/bluff.py --codec stl --outfile "$rundir/bluff.stl" --params params/runs/run_001.json
```

## 5) Loop skeleton for CFD iteration

```bash
for run in params/runs/*.json; do
  run_id=$(basename "$run" .json)
  mkdir -p "jobs/$run_id"
  cq-cli --infile designs/bluff.py --codec step --outfile "jobs/$run_id/bluff.step" --params "$run"
done
```

## 6) Failure handling

- `cq-cli` exit code non-zero means invalid params or script failure.
- Save the `stderr` stream into each job folder for diagnosis.
- Reject runs before meshing if schema validation fails.

## 7) Why this is the strong choice for your case

CadQuery is usually the best for:

- large sweep counts
- reproducibility
- zero GUI dependency
- simple parameter-driven optimization loops

## 8) References

- `cq-cli` package: https://pypi.org/project/cadquery-cli/
- CQGI docs: https://cadquery.readthedocs.io/en/stable/cqgi.html
