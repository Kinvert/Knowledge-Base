# OpenSCAD CFD Sweep Workflow

OpenSCAD is useful when the geometry can be expressed as a compact parametric script.

## 1) Core model with declared defaults

`designs/bluff.scad`

```scad
$fn = 120;
diffuser_angle_deg = 30;   // degrees
inlet_radius = 0.20;      // meters
outlet_radius = 0.35;     // meters
height = 0.5;

module bluff(diffuser_angle_deg, inlet_radius, outlet_radius) {
  translate([0,0,0]) cylinder(h = height * 0.25, r1 = inlet_radius, r2 = inlet_radius * 1.2, $fn=$fn);
  translate([0,0,height * 0.25])
    cylinder(h = height * 0.75, r1 = inlet_radius * 1.2, r2 = outlet_radius, $fn=$fn);
}

bluff(diffuser_angle_deg, inlet_radius, outlet_radius);
```

## 2) Generate params from xlsx

```python
import json
import openpyxl

wb = openpyxl.load_workbook("params/design_matrix.xlsx", data_only=True)
ws = wb["sweep"]
runs = []
for row_i, row in enumerate(ws.iter_rows(min_row=2, values_only=True), start=1):
    runs.append({
        "name": f"run_{row_i:04d}",
        "params": {
            "diffuser_angle_deg": row[0],
            "inlet_radius": row[1],
            "outlet_radius": row[2],
        },
    })
```

For each row build a preset JSON in OpenSCAD customizer format:

```python
import json

with open("params/presets.json", "w") as f:
    json.dump({
        "fileFormatVersion": "1",
        "parameterSets": [
            {"name": item["name"], "params": item["params"]} for item in runs
        ]
    }, f, indent=2)
```

## 3) Run CLI sweeps

Direct override per run:

```bash
openscad \
  -o jobs/run_001/bluff.stl \
  -D 'diffuser_angle_deg=32' \
  -D 'inlet_radius=0.22' \
  -D 'outlet_radius=0.36' \
  designs/bluff.scad
```

Preset-driven batch mode:

```bash
openscad -o jobs/run_001/bluff.stl -p params/presets.json -P run_001 designs/bluff.scad
```

## 4) Output handoff

- STL is native; for STEP, route through conversion (e.g., CAD-to-STEP bridge or remeshing step).
- Keep one STL per run and a JSON manifest with source params.

## 5) Practical loop tips

- Use `-D` for one-off DOE steps.
- Use `-p/-P` for large grids to avoid quoting issues.
- Add a small geometry guard script after mesh generation (volume > 0, facet count bounds).

## 6) When OpenSCAD is ideal

- You need fast, deterministic geometry from formulas.
- You do not need advanced feature-tree editing each iteration.
- You have moderate topology and do not need direct native STEP emission.

## 7) References

- CLI docs: https://manpages.debian.org/testing/openscad/openscad.1.en.html
- Customizer: https://en.wikibooks.org/wiki/OpenSCAD_User_Manual/Customizer
