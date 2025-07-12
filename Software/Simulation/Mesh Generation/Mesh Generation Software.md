# Mesh Generation Software

**Mesh Generation Software** consists of tools and libraries designed to create discrete meshes from continuous geometries. These meshes are essential for applications in simulation (e.g., FEM, CFD), manufacturing (e.g., 3D printing), graphics (e.g., rendering), and robotics (e.g., environment modeling, collision detection).

This page catalogs the most commonly used tools, grouped by interface type, domain, and format support.

---

## 🛠️ GUI-Based Tools

| Tool        | Domains               | STL Support | Notes                                               |
|-------------|------------------------|-------------|------------------------------------------------------|
| [[Gmsh]]    | FEM, CFD, General      | ✅ Yes      | Scripting + GUI, high-quality 2D/3D meshing         |
| [[FreeCAD]] | CAD, FEM               | ✅ Yes      | CAD modeling with built-in meshing and solver tools |
| [[Blender]] | Graphics, 3D Printing  | ✅ Yes      | Mesh modeling, repair, and sculpting                |
| [[MeshLab]] | Graphics, Printing     | ✅ Yes      | Repair, remesh, filter, decimate, convert           |
| [[Salome]]  | FEM, Multiphysics      | ✅ Yes      | Modular GUI for meshing, geometry, and solver setup |
| [[Netgen]]  | FEM                    | ✅ Yes      | Lightweight mesh generator with surface cleanup     |
| [[CloudCompare]] | Point Cloud, GIS | ✅ Yes      | Includes meshing tools for clouds and DEMs          |

---

## 🧪 CLI or Library-Based Tools

| Tool         | Language | Domain        | Notes                                           |
|--------------|----------|---------------|--------------------------------------------------|
| [[TetGen]]   | C++      | 3D FEM, CFD   | High-quality tetrahedral meshing (Delaunay)     |
| [[CGAL]]     | C++      | Computational Geometry | Rich library with remeshing and triangulation  |
| [[snappyHexMesh]] | C++ | CFD (OpenFOAM) | Hex-dominant mesh generator                     |
| [[OpenSCAD]] | Script   | 3D Printing    | Generates STL from programmatic models          |
| [[Triangle]] | C        | 2D Tri Mesh    | Robust Delaunay triangulation for 2D domains    |
| [[gmsh CLI]] | C++      | General        | Powerful scripting via `.geo` files             |

---

## 🧱 STL and 3D Printing-Oriented Tools

| Tool        | Functions                         | Notes                                     |
|-------------|-----------------------------------|-------------------------------------------|
| [[MeshLab]] | Repair, remesh, filter             | STL/OBJ cleaning and simplification        |
| [[Blender]] | Edit, sculpt, decimate             | High-quality mesh shaping and optimization |
| [[Netfabb]] | Repair, validate                   | Autodesk tool for fixing 3D print meshes   |
| [[PrusaSlicer]] | Preview & mesh-based slicing | STL slicer that respects mesh topology     |

---

## 🌊 CFD + FEM Integration

| Tool             | Compatible With     | Notes                                          |
|------------------|---------------------|------------------------------------------------|
| [[Gmsh]]         | [[OpenFOAM]], FEniCS, Elmer | Supports high-quality boundary layer meshing |
| [[snappyHexMesh]]| [[OpenFOAM]]        | Best for hex-dominant CFD domains             |
| [[Salome]]       | Code_Aster, Elmer    | Excellent for multi-domain meshing            |
| [[Netgen]]       | FEniCS, CalculiX     | Lightweight, GUI or CLI                       |

---

## 📦 File Format Compatibility

| Tool        | STL | OBJ | STEP | IGES | VTK | MSH (Gmsh) | Med | Custom |
|-------------|-----|-----|------|------|-----|------------|-----|--------|
| Gmsh        | ✅  | ✅  | ✅   | ✅   | ✅  | ✅         | ✅  | `.geo` |
| MeshLab     | ✅  | ✅  | ❌   | ❌   | ✅  | ❌         | ❌  | -      |
| Blender     | ✅  | ✅  | ❌   | ❌   | ✅  | ❌         | ❌  | -      |
| FreeCAD     | ✅  | ✅  | ✅   | ✅   | ✅  | ❌         | ❌  | `.FCStd` |
| Salome      | ✅  | ✅  | ✅   | ✅   | ✅  | ❌         | ✅  | `.hdf` |
| TetGen      | ✅  | ❌  | ❌   | ❌   | ❌  | ❌         | ❌  | `.poly` |

---

## 🔗 Related Concepts

- [[Mesh Generation]]  
- [[Finite Element Method]]  
- [[CFD]]  
- [[Gmsh]]  
- [[snappyHexMesh]]  
- [[3D Printing]]  
- [[Surface Reconstruction]]  
- [[Point Cloud]]  
- [[CAD Modeling]]  
- [[Simulation Visualization]]  
- [[STL File]]  
- [[Remeshing]]

---

## 📚 Further Reading

- Gmsh documentation: scripting and mesh tuning  
- TetGen and Triangle official papers  
- MeshLab and Blender tutorials on mesh repair workflows  
- OpenFOAM tutorials for mesh preparation  
- Comparative benchmarks for 3D mesh generators

---
