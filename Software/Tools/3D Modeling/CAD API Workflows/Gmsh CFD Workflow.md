# Gmsh CFD Workflow

Gmsh is ideal as a scripted geometry+meshing stage after CAD generation, and in some cases can be the only geometry generator for sweeps.

## 1) Use-cases in CFD loops

- Use Gmsh if you can express shape as primitives/boolean operations in `.geo`.
- Use as post-CAD stage for precise mesh-size control from parameters.

## 2) Parameterized `.geo` sketch (`designs/bluff.geo`)

```geo
lc = 0.02; // mesh size, overwritten with -setnumber
Angle = 30; // deg
Rin = 0.2;
Rout = 0.35;
Height = 0.5;

// define points / splines / loops from parameterized variables
Point(1) = {0, 0, 0, lc};
Point(2) = {Rin, 0, 0, lc};
Circle(3) = {2, 1, 2};
```

## 3) Run with CLI parameters

```bash
gmsh -2 designs/bluff.geo -setnumber Angle 32 -setnumber Rin 0.22 -setnumber Rout 0.35 -o jobs/run_001/bluff.msh
```

`-2` asks for 2D mesh workflow in this example; use 3D modes according to model setup.

## 4) Python API loop pattern

```python
import gmsh
import json

with open("params/runs/run_001.json") as f:
    p = json.load(f)

gmsh.initialize()
gmsh.open("designs/bluff.geo")
gmsh.option.setNumber("Mesh.CharacteristicLengthMin", float(p["min_cl"]))
gmsh.option.setNumber("Mesh.CharacteristicLengthMax", float(p["max_cl"]))
gmsh.model.mesh.generate(3)
gmsh.write("jobs/run_001/bluff.msh")
gmsh.finalize()
```

## 5) Why in this note

Gmsh is not a full replacement for modern parametric CAD for every project, but it is often the fastest scripted mesh side of a CFD loop.

## 6) References

- https://gmsh.info/doc/texinfo/gmsh.html
