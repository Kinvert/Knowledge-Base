# OpenFOAM

**OpenFOAM** (Open Field Operation and Manipulation) is an open-source C++ toolbox for the development and application of computational fluid dynamics (CFD) simulations. It is widely used in industry and academia for modeling fluid flow, heat transfer, chemical reactions, and more.

---

## 🧠 Overview

OpenFOAM uses the **finite volume method** (FVM) and supports both steady and unsteady simulations. It is designed to be extremely flexible and modular, allowing users to define custom solvers, boundary conditions, and models through C++ code and configuration files.

OpenFOAM is often run via command line and is structured around a case directory that defines mesh, boundary conditions, solver settings, and physical properties.

---

## 🔧 Core Features

- Wide range of built-in solvers for incompressible/compressible flow, multiphase, heat transfer, etc.  
- Mesh generation and manipulation tools (e.g., `blockMesh`, `snappyHexMesh`)  
- Pre- and post-processing with tools like ParaView and foamToVTK  
- Parallel execution via MPI  
- Extensibility for user-defined solvers and models

---

## 🧰 Use Cases

- Aerodynamics (e.g., UAV or drone design)  
- HVAC system optimization  
- Underwater robotics and marine dynamics  
- Combustion and chemical reactors  
- Thermal management in electronics  
- Wind flow over terrain and buildings

---

## ✅ Pros

- Open-source and free to use  
- Highly customizable with access to source code  
- Strong support for parallel computing  
- Large community and extensive online resources

---

## ❌ Cons

- Steep learning curve, especially for new users  
- Requires strong understanding of CFD fundamentals and file structure  
- Debugging and customization require C++ knowledge  
- GUI support is limited; workflows are mostly command-line based

---

## 📊 Comparison Table: OpenFOAM vs Other CFD Tools

| Feature               | OpenFOAM    | [[ANSYS Fluent]] | [[SU2]]     | [[COMSOL]]   |
|----------------------|-------------|--------------|-------------|--------------|
| License              | Open-source | Commercial   | Open-source | Commercial   |
| Solver Base          | Finite Volume | Finite Volume | Finite Volume | Finite Element |
| GUI Support          | ❌ Limited  | ✅ Yes        | ❌ No       | ✅ Yes        |
| Parallelism          | ✅ MPI      | ✅ MPI        | ✅ MPI      | ✅ MPI        |
| Extensibility        | ✅ C++ API  | 🟡 Limited    | 🟡 Python    | 🟡 Moderate   |
| Multiphysics         | 🟡 Limited  | ✅ Yes        | 🟡 Limited  | ✅ Strong     |

---

## ⚙️ Common Commands (One-Liners)

- `blockMesh` — generate mesh from blockMeshDict  
- `snappyHexMesh` — generate mesh from STL surface  
- `simpleFoam` — run steady-state incompressible solver  
- `paraFoam` — launch ParaView for visualization  
- `decomposePar` — decompose mesh for parallel run  
- `reconstructPar` — reconstruct result from parallel parts  
- `foamToVTK` — convert results for use with VTK-based viewers

---

## 🔗 Related Concepts

- [[CFD]] (Computational Fluid Dynamics)  
- [[Finite Volume Method]]  
- [[ParaView]] (for visualization)  
- [[Mesh Generation]]  
- [[Turbulence Modeling]]  
- [[Navier-Stokes Equations]]  
- [[PDE Simulation]]

---

## 📚 Further Reading

- OpenFOAM User Guide and Programmer’s Guide  
- The OpenFOAM Foundation tutorials  
- CFD Online forums and wiki  
- Books like *“The OpenFOAM Technology Primer”*

---
