# Blender (mesh-first) + CFD Parameter Loop

Use Blender when shape generation has a procedural mesh need and not strict mechanical feature-history requirements.

## 1) Minimal pipeline

- `run_id` and params come from JSON/XLSX.
- Blender runs headless with Python.
- Export clean STL/OBJ for meshing.

## 2) JSON-driven script

`designs/bluff_mesh.py`

```python
import json, os, bpy, math

param_path = os.environ.get("PARAM_PATH", "params/runs/run_001.json")
run_id = os.environ.get("RUN_ID", "run_001")

with open(param_path) as f:
    p = json.load(f)

angle = math.radians(float(p["diffuser_angle_deg"]))
r_in = float(p["inlet_radius_m"])
r_out = float(p["outlet_radius_m"])

bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)

mesh = bpy.ops.mesh.primitive_cylinder_add(radius=r_in, depth=0.2, location=(0,0,0))
obj = bpy.context.object
obj.name = "Bluff"

out_dir = f"jobs/{run_id}"
os.makedirs(out_dir, exist_ok=True)
bpy.ops.export_mesh.stl(filepath=f"{out_dir}/bluff.stl")
```

## 3) Headless execution

```bash
mkdir -p jobs/run_001
export PARAM_PATH=params/runs/run_001.json
export RUN_ID=run_001
blender --background --python designs/bluff_mesh.py
```

## 4) Recommended integration

- Keep Blender for mesh transforms and cleanup.
- Export to mesh-only if your CFD chain already expects STL-like geometry.
- If you need direct B-rep exchange, route via upstream CAD tool (FreeCAD/CadQuery/OpenSCAD).

## 5) References

- Blender CLI docs: https://manpages.ubuntu.com/manpages/jammy/man1/blender.1.html
- Python API: https://docs.blender.org/api/current/
